// Wi-Fi network scanning through the ESP32-C6 (ESP-Hosted + esp_wifi_remote).
#pragma once

#include <stdint.h>

enum class WifiState : uint8_t { Off, Starting, Scanning, Failed };

// Starts the scan task. It waits for the radio-chip link, then scans only while
// wifi_set_wanted(true) - so Bluetooth gets the radio to itself otherwise.
void wifi_start();
void wifi_set_wanted(bool wanted);
void wifi_demo_set_scanning();      // demo mode only

WifiState   wifi_state();
int         wifi_error();
uint32_t    wifi_scans_done();
const char *wifi_state_text();
