#include "wifi_scan.h"
#include "ble_scan.h"
#include "devices.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "finder_wifi";
#define WIFI_SELFTEST 0   // 1 = scan Wi-Fi from boot and log results (for testing)

static volatile WifiState s_state = WifiState::Off;
static volatile int       s_err = 0;
static volatile bool      s_wanted = false;
static volatile uint32_t  s_scans = 0;

static bool link_up() {
    BleState b = ble_state();
    // Every state after Linking means the ESP-Hosted transport came up.
    return b != BleState::Starting && b != BleState::Linking && b != BleState::LinkFailed;
}

static bool init_wifi() {
    esp_err_t err = esp_netif_init();
    if (err == ESP_OK || err == ESP_ERR_INVALID_STATE) {
        err = esp_event_loop_create_default();
        if (err == ESP_ERR_INVALID_STATE) err = ESP_OK;
    }
    if (err == ESP_OK && !esp_netif_create_default_wifi_sta()) err = ESP_FAIL;
    if (err == ESP_OK) {
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        err = esp_wifi_init(&cfg);
    }
    if (err == ESP_OK) err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
    if (err == ESP_OK) err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err == ESP_OK) err = esp_wifi_start();
    if (err != ESP_OK) {
        s_err = err;
        ESP_LOGE(TAG, "Wi-Fi start failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "Wi-Fi ready (scan only, never connects)");
    return true;
}

static void scan_task(void *) {
    while (!link_up()) vTaskDelay(pdMS_TO_TICKS(200));
    if (ble_state() == BleState::LinkFailed) { s_state = WifiState::Failed; vTaskDelete(nullptr); return; }

    // Start Wi-Fi only once someone asks for it.
    while (!s_wanted) vTaskDelay(pdMS_TO_TICKS(200));
    s_state = WifiState::Starting;
    if (!init_wifi()) { s_state = WifiState::Failed; vTaskDelete(nullptr); return; }
    s_state = WifiState::Scanning;

    static wifi_ap_record_t recs[64];
    for (;;) {
        if (!s_wanted) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }

        Target tg = target_get();
        wifi_scan_config_t cfg;
        memset(&cfg, 0, sizeof cfg);
        cfg.show_hidden = true;
        cfg.scan_type = WIFI_SCAN_TYPE_ACTIVE;
        if (tg.active && tg.wifi_channel) {
            // Tracking one network: stay on its channel for quick updates.
            cfg.channel = tg.wifi_channel;
            cfg.scan_time.active.min = 40;
            cfg.scan_time.active.max = 90;
        } else {
            cfg.scan_time.active.min = 50;
            cfg.scan_time.active.max = 110;
        }

        esp_err_t err = esp_wifi_scan_start(&cfg, true);
        if (err != ESP_OK) {
            s_err = err;
            ESP_LOGW(TAG, "scan failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        uint16_t n = sizeof recs / sizeof recs[0];
        if (esp_wifi_scan_get_ap_records(&n, recs) == ESP_OK) {
            for (uint16_t i = 0; i < n; ++i)
                devices_feed_wifi(recs[i].bssid, (const char *)recs[i].ssid, recs[i].rssi,
                                  recs[i].primary, (uint8_t)recs[i].authmode);
        }
        if (WIFI_SELFTEST && n) ESP_LOGI(TAG, "scan %lu: %u networks, e.g. \"%s\" %d dBm ch%u", (unsigned long)s_scans, n, (const char *)recs[0].ssid, recs[0].rssi, recs[0].primary);
        ++s_scans;
        s_err = 0;
        // Leave the radio to Bluetooth for a moment between full sweeps.
        vTaskDelay(pdMS_TO_TICKS(tg.active && tg.wifi_channel ? 20 : 300));
    }
}

void wifi_start() {
    xTaskCreatePinnedToCore(scan_task, "wifi_scan", 6144, nullptr, 4, nullptr, 1);
}

void wifi_set_wanted(bool wanted) { s_wanted = wanted || WIFI_SELFTEST; }
#ifndef WIFI_SELFTEST
#define WIFI_SELFTEST 0
#endif

WifiState wifi_state()      { return s_state; }
int       wifi_error()      { return s_err; }
uint32_t  wifi_scans_done() { return s_scans; }

const char *wifi_state_text() {
    switch (s_state) {
    case WifiState::Off:      return "Wi-Fi off";
    case WifiState::Starting: return "Starting Wi-Fi";
    case WifiState::Scanning: return "Scanning Wi-Fi";
    case WifiState::Failed:   return "Wi-Fi failed";
    }
    return "";
}

void wifi_demo_set_scanning() { s_state = WifiState::Scanning; }
