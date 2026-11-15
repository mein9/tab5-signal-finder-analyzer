// Demo mode: fills the lists with made-up devices and networks instead of
// scanning, so screenshots don't show anything from real surroundings.
// Build with:  idf.py -DFINDER_DEMO=1 build
#include "demo.h"

#if FINDER_DEMO

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ble_scan.h"
#include "devices.h"
#include "wifi_scan.h"

namespace {

struct FakeBle {
    uint8_t     addr[6];
    uint8_t     addr_type;
    float       rssi;           // typical strength
    float       swing;          // slow drift, dB
    uint8_t     len;
    uint8_t     data[40];
};

struct FakeWifi {
    uint8_t     bssid[6];
    const char *ssid;
    float       rssi;
    uint8_t     channel, auth;
};

FakeBle  s_ble[16];
size_t   s_nble = 0;

// Appends one advertisement field: [length, type, payload...].
void field(FakeBle &d, uint8_t type, const uint8_t *payload, size_t n) {
    d.data[d.len++] = (uint8_t)(n + 1);
    d.data[d.len++] = type;
    memcpy(d.data + d.len, payload, n);
    d.len += n;
}

void name(FakeBle &d, const char *s) { field(d, 0x09, (const uint8_t *)s, strlen(s)); }

FakeBle &add(float rssi, float swing, uint8_t addr_type = 1) {
    FakeBle &d = s_ble[s_nble++];
    memset(&d, 0, sizeof d);
    esp_fill_random(d.addr, 6);
    d.addr_type = addr_type;
    d.rssi = rssi;
    d.swing = swing;
    return d;
}

void build() {
    // AirPods Pro 2: Apple "proximity pairing" message with model id 0x1420.
    {
        FakeBle &d = add(-58, 14);
        uint8_t m[29] = {0x4C, 0x00, 0x07, 0x19, 0x01, 0x14, 0x20};
        field(d, 0xFF, m, sizeof m);
    }
    // Find My item near its owner.
    {
        FakeBle &d = add(-71, 4);
        const uint8_t m[] = {0x4C, 0x00, 0x12, 0x02, 0x00, 0x01};
        field(d, 0xFF, m, sizeof m);
    }
    // Find My item separated from its owner (e.g. an AirTag).
    {
        FakeBle &d = add(-66, 3);
        uint8_t m[29] = {0x4C, 0x00, 0x12, 0x19, 0x10};
        field(d, 0xFF, m, sizeof m);
    }
    // Tile tracker.
    {
        FakeBle &d = add(-79, 3);
        const uint8_t u[] = {0xED, 0xFE};
        field(d, 0x03, u, sizeof u);
    }
    // Google Fast Pair earbuds.
    {
        FakeBle &d = add(-63, 5);
        const uint8_t sd[] = {0x2C, 0xFE, 0x12, 0x34, 0x56};
        field(d, 0x16, sd, sizeof sd);
        name(d, "Aurora Buds Pro");
    }
    {
        FakeBle &d = add(-52, 3, 0);
        const uint8_t m[] = {0x06, 0x00, 0x01, 0x09, 0x20, 0x02};
        field(d, 0xFF, m, sizeof m);
    }
    {
        FakeBle &d = add(-74, 4);
        const uint8_t m[] = {0x4C, 0x00, 0x10, 0x05, 0x01, 0x18, 0x00, 0x00, 0x00};
        field(d, 0xFF, m, sizeof m);
    }
    {
        FakeBle &d = add(-68, 3, 0);
        const uint8_t m[] = {0x75, 0x00, 0x42, 0x04};
        field(d, 0xFF, m, sizeof m);
        name(d, "Kitchen Display");
    }
    { FakeBle &d = add(-61, 4, 0); name(d, "Trail Speaker Mini"); }
    { FakeBle &d = add(-77, 5);    name(d, "Fitness Band 7"); }
    { FakeBle &d = add(-83, 3, 0); name(d, "E-Bike Lock 2041"); }
    { FakeBle &d = add(-88, 3, 0); name(d, "Greenhouse Sensor"); }
    {
        FakeBle &d = add(-86, 2);
        const uint8_t m[] = {0x57, 0x01, 0x00, 0x00};
        field(d, 0xFF, m, sizeof m);
    }
}

const FakeWifi kWifi[] = {
    {{0x02, 0x11, 0x22, 0x33, 0x44, 0x01}, "Maple Street Cafe", -47, 6, 3},
    {{0x02, 0x11, 0x22, 0x33, 0x44, 0x02}, "Maple Street Cafe Guest", -49, 6, 0},
    {{0x02, 0x5A, 0x10, 0x7C, 0x90, 0x0A}, "Blue Heron", -58, 1, 7},
    {{0x02, 0x5A, 0x10, 0x7C, 0x90, 0x0B}, "Lakeside-2G", -64, 11, 3},
    {{0x02, 0x9E, 0x45, 0x00, 0x18, 0x21}, "PrintServer-Setup", -71, 11, 0},
    {{0x02, 0x9E, 0x45, 0x00, 0x18, 0x22}, "", -76, 3, 3},
    {{0x02, 0xC4, 0x77, 0x12, 0x06, 0x30}, "Workshop", -80, 1, 4},
    {{0x02, 0xC4, 0x77, 0x12, 0x06, 0x31}, "Attic Repeater", -84, 9, 3},
};

void demo_task(void *) {
    build();
    ble_demo_set_scanning();
    wifi_demo_set_scanning();
    uint32_t tick = 0;
    for (;;) {
        float t = now_ms() / 1000.0f;
        for (size_t i = 0; i < s_nble; ++i) {
            FakeBle &d = s_ble[i];
            if ((esp_random() % 3) == 0) continue;          // not every device every time
            float r = d.rssi + d.swing * sinf(t / (4.0f + i) + i) + (int)(esp_random() % 7) - 3;
            devices_feed(d.addr, d.addr_type, (int8_t)r, d.data, d.len);
        }
        if (tick % 10 == 0) {
            for (const auto &w : kWifi) {
                float r = w.rssi + (int)(esp_random() % 5) - 2;
                devices_feed_wifi(w.bssid, w.ssid, (int8_t)r, w.channel, w.auth);
            }
        }
        ++tick;
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}

} // namespace

void demo_start() { xTaskCreatePinnedToCore(demo_task, "demo", 4096, nullptr, 3, nullptr, 1); }

#else

void demo_start() {}

#endif
