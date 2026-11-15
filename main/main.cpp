// Signal Finder & Analyzer for the M5Stack Tab5.
//
// Lists nearby Bluetooth LE devices and Wi-Fi networks, then guides you to one with a
// hot/cold signal meter, Geiger-style beeps, and a sweep radar that uses
// the motion sensor to show which direction the signal is strongest.

#include <M5Unified.h>

#include "esp_log.h"
#include "nvs_flash.h"

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

extern "C" void app_main(void) {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // Never erase here: the NVS partition is shared with the Launcher and
        // other apps. Bluetooth works without it.
        ESP_LOGW(TAG, "NVS unavailable (%s), continuing without it", esp_err_to_name(err));
    }

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
