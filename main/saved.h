// Devices the user has pinned, renamed, set a left-behind alarm on, or told
// the tracker alert to ignore. Kept in flash so they survive a restart.
//
// Only call these from the UI task.
#pragma once

#include <stdint.h>
#include <stddef.h>

#include "devices.h"

enum : uint8_t {
    kSavedPin    = 1,   // show at the top of the list
    kSavedWatch  = 2,   // alarm if it stops being heard
    kSavedIgnore = 4,   // never raise a tracker alert for it
    kSavedBySig  = 8,   // matched by device type (rotating address), not address
};

struct Saved {
    uint8_t  addr[6];
    uint8_t  addr_type;
    uint8_t  flags;
    uint32_t sig;
    char     name[24];      // nickname, empty if none
};

constexpr size_t kMaxSaved = 32;

void saved_load();

// The entry for this device, or nullptr.
Saved *saved_find(const DeviceView &dev);

// The entry for this device, creating one if needed. nullptr when full.
Saved *saved_get(const DeviceView &dev);

// Writes changes to flash and drops entries with no flags and no nickname.
void saved_store();

void   saved_clear();
size_t saved_count();
Saved *saved_at(size_t i);
