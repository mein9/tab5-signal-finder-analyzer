idf_component_register(
    SRCS         "main.cpp" "ble_scan.cpp" "devices.cpp" "ui.cpp" "wifi_scan.cpp"
                 "saved.cpp" "alerts.cpp" "sdlog.cpp"
    INCLUDE_DIRS "."
    REQUIRES     esp_driver_ppa esp_driver_spi esp_driver_sdspi fatfs sdmmc esp_hosted esp_wifi_remote esp_wifi esp_netif esp_event nvs_flash bt esp_timer
    # ESP-Hosted 3.0.6 co-processor firmware with Wi-Fi + Bluetooth for the C6,
    # built from ../c6-radio-fw. Installed on request from the app.
    EMBED_FILES  "c6_radio_fw.bin"
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers -Wno-format-truncation)
