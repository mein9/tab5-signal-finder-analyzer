// Brings up the Bluetooth link to the Tab5's ESP32-C6 and scans continuously.
#pragma once

#include <stdint.h>

enum class BleState : uint8_t {
    Starting,       // not started yet
    Linking,        // waiting for the ESP32-C6 over SDIO
    LinkFailed,     // ESP-Hosted transport did not come up
    Controller,     // asking the C6 to turn its Bluetooth controller on
    ControllerFailed,
    Stack,          // NimBLE host starting
    Scanning,
    ScanFailed,
    Paused,         // scanning switched off by the user
};

// Starts everything in a background task; returns immediately.
void ble_start();

// Pauses or resumes scanning (the Scan button).
void ble_set_scanning(bool on);

// Radio-chip firmware update (adds Bluetooth to a Wi-Fi-only C6 build).
enum class RadioUpdate : uint8_t { Idle, Running, Done, Failed };
void        radio_update_start();      // only valid when state is ControllerFailed
RadioUpdate radio_update_state();
int         radio_update_percent();
const char *radio_update_message();

BleState    ble_state();
int         ble_error();          // last error code, for the status line
const char *ble_state_text();
