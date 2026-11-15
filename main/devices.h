// Table of Bluetooth LE devices heard by the scanner, plus the signal history
// of the one device (or device type) currently being tracked.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <vector>

enum class Kind : uint8_t {
    Unknown,
    Named,        // has a broadcast name, nothing more specific
    AirPods,      // Apple "proximity pairing" (AirPods / Beats, case open or in use)
    FindMy,       // Apple Find My beacon (AirTag, or AirPods etc. away from their phone)
    Apple,        // other Apple device (iPhone, iPad, Mac, Watch...)
    Tile,
    SmartTag,     // Samsung Galaxy SmartTag
    Samsung,
    FastPair,     // Google Fast Pair earbuds / accessories
    Microsoft,
    WifiAP,       // Wi-Fi network (router, hotspot, access point)
};

// Bluetooth filters, plus the Wi-Fi section.
// `Any` is everything, Bluetooth and Wi-Fi together (used for logging).
enum class Filter : uint8_t { All, Findable, Named, Wifi, Any };

constexpr uint8_t kAddrTypeWifi = 0xFF;   // marks Wi-Fi entries in the shared table

struct DeviceView {
    uint8_t  addr[6];
    uint8_t  addr_type;
    char     label[48];     // what to show the user
    char     detail[80];    // second line (model, type, address)
    Kind     kind;
    uint32_t sig;           // non-zero: devices that rotate address share this
    float    rssi;          // smoothed, dBm
    int8_t   peak;
    uint32_t age_ms;        // since last heard
    uint32_t count;
    uint8_t  channel;       // Wi-Fi only
    char     name[32];      // broadcast name / SSID, empty if none
    uint32_t seen_ms;       // how long ago it was first heard
    uint32_t moved_s;       // seconds this Tab5 has been moving since then
    bool     away;          // Find My item separated from its owner
    bool     stable_addr;   // address does not rotate (public or static)
};

struct Sample {
    uint32_t t_ms;
    int8_t   rssi;
};

struct Target {
    bool     active = false;
    bool     by_sig = false;    // follow every device sharing `sig` (address rotation)
    uint32_t sig = 0;
    uint8_t  addr[6] = {0};
    uint8_t  wifi_channel = 0;  // non-zero: target is a Wi-Fi network on this channel
    char     label[48] = {0};
};

void devices_init();

// Called from the Bluetooth task for every advertisement heard.
void devices_feed(const uint8_t addr[6], uint8_t addr_type, int8_t rssi,
                  const uint8_t *data, size_t len);

// Called from the Wi-Fi scan task for every network in a scan result.
void devices_feed_wifi(const uint8_t bssid[6], const char *ssid, int8_t rssi,
                       uint8_t channel, uint8_t authmode);

// Copy of the table, strongest first, devices heard within `max_age_ms`.
void devices_snapshot(std::vector<DeviceView> &out, Filter filter, uint32_t max_age_ms);

// Milliseconds since a device was last heard, or UINT32_MAX if it is not in
// the table. With `by_sig`, matches any device sharing `sig`.
uint32_t devices_age_of(const uint8_t addr[6], uint8_t addr_type, uint32_t sig, bool by_sig);

// Total seconds the Tab5 has spent moving (from the motion sensor). Stored
// with each new device so "has this followed me?" can be answered later.
void devices_set_motion_s(uint32_t seconds);

uint32_t devices_heard_total();     // advertisements received since boot
size_t   devices_count(uint32_t max_age_ms);

void   target_set(const DeviceView &dev, bool follow_type);
void   target_clear();
void   target_rename(const char *label);
Target target_get();

// Signal samples for the target since `since_ms` (oldest first).
void target_samples(std::vector<Sample> &out, uint32_t since_ms);

uint32_t now_ms();
