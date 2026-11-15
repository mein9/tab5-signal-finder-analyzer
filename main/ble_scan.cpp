#include "ble_scan.h"
#include "devices.h"

#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_hosted.h"
#include "esp_hosted_bt_host_stack.h"
#include "esp_hosted_ota.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"

static const char *TAG = "finder_ble";

static volatile BleState s_state = BleState::Starting;
static volatile int      s_err = 0;
static uint8_t           s_own_addr_type;
static volatile bool     s_scan_wanted = true;

static void start_scan();

static int gap_event(struct ble_gap_event *ev, void *arg) {
    (void)arg;
    switch (ev->type) {
    case BLE_GAP_EVENT_DISC:
        devices_feed(ev->disc.addr.val, ev->disc.addr.type, ev->disc.rssi,
                     ev->disc.data, ev->disc.length_data);
        return 0;
    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (!s_scan_wanted) return 0;
        ESP_LOGW(TAG, "scan stopped (reason %d), restarting", ev->disc_complete.reason);
        start_scan();
        return 0;
    default:
        return 0;
    }
}

static void start_scan() {
    if (!s_scan_wanted) { s_state = BleState::Paused; return; }
    struct ble_gap_disc_params p;
    memset(&p, 0, sizeof p);
    p.passive = 0;               // active: also collects names from scan responses
    p.filter_duplicates = 0;     // every packet, we need the signal-strength stream
    p.itvl = 0x50;               // 50 ms
    p.window = 0x50;             // listen 100% of the time
    int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &p, gap_event, nullptr);
    if (rc == 0) {
        s_state = BleState::Scanning;
        ESP_LOGI(TAG, "scanning");
    } else {
        s_state = BleState::ScanFailed;
        s_err = rc;
        ESP_LOGE(TAG, "ble_gap_disc failed: %d", rc);
    }
}

static void on_sync() {
    int rc = ble_hs_util_ensure_addr(0);
    if (rc == 0) rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        s_state = BleState::ScanFailed;
        s_err = rc;
        ESP_LOGE(TAG, "address setup failed: %d", rc);
        return;
    }
    start_scan();
}

static void on_reset(int reason) {
    ESP_LOGW(TAG, "NimBLE reset, reason %d", reason);
    s_state = BleState::Stack;
}

static void host_task(void *arg) {
    (void)arg;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void bring_up(void *arg) {
    (void)arg;
    s_state = BleState::Linking;
    int rc = esp_hosted_init();
    if (rc == 0) rc = esp_hosted_connect_to_slave();
    if (rc != 0) {
        s_err = rc;
        s_state = BleState::LinkFailed;
        ESP_LOGE(TAG, "ESP-Hosted link failed: %d", rc);
        vTaskDelete(nullptr);
        return;
    }

    esp_hosted_coprocessor_fwver_t ver = {};
    if (esp_hosted_get_coprocessor_fwversion(&ver) == 0)
        ESP_LOGI(TAG, "C6 firmware %u.%u.%u", (unsigned)ver.major1, (unsigned)ver.minor1,
                 (unsigned)ver.patch1);

    s_state = BleState::Controller;
    esp_hosted_bt_host_stack_cfg_t cfg = ESP_HOSTED_BT_HOST_STACK_CONFIG_DEFAULT();
    esp_err_t err = esp_hosted_bt_host_stack_setup(&cfg);
    if (err != ESP_OK) {
        s_err = err;
        s_state = BleState::ControllerFailed;
        ESP_LOGE(TAG, "Bluetooth controller on C6 failed: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }

    s_state = BleState::Stack;
    err = nimble_port_init();
    if (err != ESP_OK) {
        s_err = err;
        s_state = BleState::ScanFailed;
        ESP_LOGE(TAG, "nimble_port_init failed: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }
    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    nimble_port_freertos_init(host_task);
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------- radio update
extern const uint8_t c6_fw_start[] asm("_binary_c6_radio_fw_bin_start");
extern const uint8_t c6_fw_end[]   asm("_binary_c6_radio_fw_bin_end");

static volatile RadioUpdate s_upd = RadioUpdate::Idle;
static volatile int         s_upd_pct = 0;
static const char *volatile s_upd_msg = "";

static void update_fail(const char *msg, esp_err_t err) {
    ESP_LOGE(TAG, "radio update: %s (%s)", msg, esp_err_to_name(err));
    s_upd_msg = msg;
    s_upd = RadioUpdate::Failed;
}

static void update_task(void *) {
    const uint8_t *img = c6_fw_start;
    size_t len = (size_t)(c6_fw_end - c6_fw_start);
    // Sanity-check the bundled image: ESP app image built for the ESP32-C6.
    if (len < 0x10000 || img[0] != 0xE9 || (img[12] | (img[13] << 8)) != 0x000D) {
        update_fail("Bundled radio firmware looks wrong; nothing was sent", ESP_ERR_INVALID_ARG);
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGW(TAG, "radio update: sending %u bytes to the C6", (unsigned)len);
    s_upd_msg = "Preparing the radio chip";
    esp_err_t err = esp_hosted_slave_ota_begin();
    if (err != ESP_OK) { update_fail("Radio chip refused to start the update; nothing changed", err); vTaskDelete(nullptr); return; }

    s_upd_msg = "Sending firmware - keep the Tab5 on";
    const size_t chunk = 1400;
    for (size_t off = 0; off < len; off += chunk) {
        size_t n = len - off < chunk ? len - off : chunk;
        err = esp_hosted_slave_ota_write(img + off, (uint32_t)n);
        if (err != ESP_OK) {
            esp_hosted_slave_ota_end();
            update_fail("Transfer failed; the radio chip keeps its old firmware", err);
            vTaskDelete(nullptr);
            return;
        }
        s_upd_pct = (int)((off + n) * 100 / len);
    }

    s_upd_msg = "Checking the new firmware";
    err = esp_hosted_slave_ota_end();
    if (err != ESP_OK) { update_fail("Radio chip rejected the image; it keeps its old firmware", err); vTaskDelete(nullptr); return; }
    err = esp_hosted_slave_ota_activate();
    if (err != ESP_OK) { update_fail("Could not switch to the new firmware; old one stays", err); vTaskDelete(nullptr); return; }

    ESP_LOGW(TAG, "radio update: done, restarting to resync");
    s_upd_msg = "Done - restarting";
    s_upd = RadioUpdate::Done;
    vTaskDelay(pdMS_TO_TICKS(2500));
    esp_restart();
}

void radio_update_start() {
    if (s_upd == RadioUpdate::Running || s_state != BleState::ControllerFailed) return;
    s_upd = RadioUpdate::Running;
    s_upd_pct = 0;
    xTaskCreatePinnedToCore(update_task, "radio_upd", 8192, nullptr, 5, nullptr, 1);
}

RadioUpdate radio_update_state()   { return s_upd; }
int         radio_update_percent() { return s_upd_pct; }
const char *radio_update_message() { return s_upd_msg; }

void ble_start() {
    xTaskCreatePinnedToCore(bring_up, "ble_up", 6144, nullptr, 5, nullptr, 1);
}

BleState ble_state() { return s_state; }
int      ble_error() { return s_err; }

const char *ble_state_text() {
    switch (s_state) {
    case BleState::Starting:         return "Starting Bluetooth";
    case BleState::Linking:          return "Connecting to radio chip";
    case BleState::LinkFailed:       return "Radio chip not responding";
    case BleState::Controller:       return "Turning on Bluetooth";
    case BleState::ControllerFailed: return "Bluetooth failed on radio chip";
    case BleState::Stack:            return "Starting Bluetooth";
    case BleState::Scanning:         return "Scanning";
    case BleState::ScanFailed:       return "Scan failed";
    case BleState::Paused:           return "Scanning is off";
    }
    return "";
}

void ble_set_scanning(bool on) {
    if (on == s_scan_wanted) return;
    s_scan_wanted = on;
    if (!on && s_state == BleState::Scanning) {
        ble_gap_disc_cancel();
        s_state = BleState::Paused;
        ESP_LOGI(TAG, "scanning paused");
    } else if (on && s_state == BleState::Paused) {
        start_scan();
    }
}
