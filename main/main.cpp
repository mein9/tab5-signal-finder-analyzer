// Signal Finder & Analyzer for the M5Stack Tab5.
//
// Lists nearby Bluetooth LE devices and Wi-Fi networks, then guides you to one with a
// hot/cold signal meter, Geiger-style beeps, and a sweep radar that uses
// the motion sensor to show which direction the signal is strongest.

#include <M5Unified.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_rom_sys.h"
#include "esp_system.h"

#include "ble_scan.h"
#include "wifi_scan.h"
#include "devices.h"
#include "demo.h"
#include "sdr.h"
#include "ui.h"

static const char *TAG = "finder";

static void log_task(void *) {
    // Once every 3 s, print a summary to the USB serial port for debugging.
    std::vector<DeviceView> list;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(3000));
        devices_snapshot(list, Filter::All, 10000);
        ESP_LOGI(TAG, "state=%s err=%d heard=%lu devices=%u", ble_state_text(), ble_error(),
                 (unsigned long)devices_heard_total(), (unsigned)list.size());
        for (size_t i = 0; i < list.size() && i < 6; ++i)
            ESP_LOGI(TAG, "  %4.0f dBm  %-28s %s", list[i].rssi, list[i].label, list[i].detail);
    }
}

// Keeps a short record of how the app was started each time (the chip's reset
// code, and the Launcher's own boot flags at that moment). It is printed on
// the USB serial port, to help work out why a power-on did or didn't go
// through the Launcher.
struct StartRecord { uint8_t rom_reason, idf_reason, launcher_init, boot_to_app, boot_timer, ddlb; };

static void record_start() {
    constexpr size_t kKeep = 12;
    StartRecord hist[kKeep] = {};
    size_t n = 0;
    nvs_handle_t h;
    if (nvs_open("blefind", NVS_READWRITE, &h) != ESP_OK) return;
    size_t len = sizeof hist;
    if (nvs_get_blob(h, "starts", hist, &len) == ESP_OK) n = len / sizeof(StartRecord);

    StartRecord r = {};
    r.rom_reason = (uint8_t)esp_rom_get_reset_reason(0);
    r.idf_reason = (uint8_t)esp_reset_reason();
    r.launcher_init = r.boot_to_app = r.boot_timer = r.ddlb = 0xFF;      // 0xFF = not stored
    nvs_handle_t l;
    if (nvs_open("launcher", NVS_READONLY, &l) == ESP_OK) {
        nvs_get_u8(l, "init", &r.launcher_init);
        nvs_get_u8(l, "bootToApp", &r.boot_to_app);
        nvs_get_u8(l, "bootTimer", &r.boot_timer);
        nvs_get_u8(l, "DDLB", &r.ddlb);
        nvs_close(l);
    }
    // The M5Launcher flips its "init" flag every time it starts, and when it
    // finds the flag set it skips its own screen and restarts straight into
    // the installed app. Left alone, that makes every other power-on bypass
    // the Launcher. Clearing the flag here means the Launcher always shows
    // (and then follows its own "boot to app" setting).
    if (r.launcher_init == 1 && nvs_open("launcher", NVS_READWRITE, &l) == ESP_OK) {
        nvs_set_u8(l, "init", 0);
        nvs_commit(l);
        nvs_close(l);
    }
    if (n == kKeep) { memmove(hist, hist + 1, (kKeep - 1) * sizeof(StartRecord)); --n; }
    hist[n++] = r;
    nvs_set_blob(h, "starts", hist, n * sizeof(StartRecord));
    nvs_commit(h);
    nvs_close(h);

    for (size_t i = 0; i < n; ++i)
        ESP_LOGI(TAG, "start %u%s: chip reset code %u, idf reason %u, launcher init=%u bootToApp=%u bootTimer=%u DDLB=%u",
                 (unsigned)(i + 1), i + 1 == n ? " (this one)" : "", hist[i].rom_reason, hist[i].idf_reason,
                 hist[i].launcher_init, hist[i].boot_to_app, hist[i].boot_timer, hist[i].ddlb);
}

extern "C" void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Never erase here: the NVS partition is shared with the Launcher and
        // other apps. Bluetooth works without it.
        ESP_LOGW(TAG, "NVS unavailable (%s), continuing without it", esp_err_to_name(err));
    }

    record_start();

    auto cfg = M5.config();
    cfg.internal_imu = true;
    cfg.internal_spk = true;
    cfg.internal_mic = false;
    M5.begin(cfg);      // also powers the ESP32-C6 radio through the I/O expander
    ESP_LOGI(TAG, "board %d, display %dx%d, imu %d", (int)M5.getBoard(),
             (int)M5.Display.width(), (int)M5.Display.height(), (int)M5.Imu.isEnabled());

    sdr_power_on();     // USB-A port, for an RTL-SDR dongle
    devices_init();
    ui_init();
    vTaskDelay(pdMS_TO_TICKS(300));   // give the C6 a moment after power-up
#if FINDER_DEMO
    demo_start();       // made-up devices, no radios
#else
    ble_start();
    wifi_start();
#endif
    sdr_start();
    xTaskCreatePinnedToCore(log_task, "log", 4096, nullptr, 1, nullptr, 0);

    for (;;) {
        ui_loop_once();
    }
}
