# The phone-band feature needs the esp_rtl_sdr driver, which is AGPL-3.0 and
# is not part of this repository. If it has been placed in
# components/esp_rtl_sdr (see the README) it is built in; otherwise the app
# builds without it and the Phone section says so.
get_filename_component(finder_root "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
set(finder_sdr_requires "")
set(finder_has_sdr 0)
if(EXISTS "${finder_root}/components/esp_rtl_sdr/CMakeLists.txt")
    set(finder_sdr_requires esp_rtl_sdr)
    set(finder_has_sdr 1)
endif()

idf_component_register(
    SRCS         "main.cpp" "ble_scan.cpp" "devices.cpp" "ui.cpp" "wifi_scan.cpp"
                 "saved.cpp" "alerts.cpp" "sdlog.cpp" "clock.cpp" "sdr.cpp" "demo.cpp"
    INCLUDE_DIRS "."
    REQUIRES     ${finder_sdr_requires} esp_driver_ppa esp_driver_spi esp_driver_sdspi fatfs sdmmc
                 esp_hosted esp_wifi_remote esp_wifi esp_netif esp_event nvs_flash bt esp_timer
    # ESP-Hosted 3.0.6 co-processor firmware with Wi-Fi + Bluetooth for the C6
    # (Espressif's bleprph_wifi_coex/cp example, Apache-2.0). Installed on
    # request from the app.
    EMBED_FILES  "c6_radio_fw.bin"
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers -Wno-format-truncation)

# Demo mode (made-up devices for screenshots):  idf.py -DFINDER_DEMO=1 build
if(FINDER_DEMO)
    set(finder_demo 1)
else()
    set(finder_demo 0)
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE FINDER_HAS_SDR=${finder_has_sdr} FINDER_DEMO=${finder_demo})
