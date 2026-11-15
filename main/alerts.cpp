#include "alerts.h"

#include <stdio.h>
#include <string.h>
#include <vector>

#include "nvs.h"

#include "saved.h"

namespace {

// Tracker alert: same tracker heard for this long, while the Tab5 itself was
// moving for at least this long. A tracker sitting in the room with you while
// you sit still doesn't count.
constexpr uint32_t kFollowMs = 10 * 60 * 1000;
constexpr uint32_t kFollowMovedS = 120;
// Left-behind alarm: not heard for this long after being heard.
constexpr uint32_t kGoneMs = 60 * 1000;
// New-device watch ignores faint devices that flicker in and out.
constexpr float kNewMinRssi = -85.0f;

Alert    s_alert;
bool     s_tracker_on = true;
uint32_t s_last_tick = 0;
std::vector<DeviceView> s_list;

// Small sets of 32-bit keys.
struct KeySet {
    std::vector<uint32_t> v;
    bool has(uint32_t k) const {
        for (uint32_t x : v) if (x == k) return true;
        return false;
    }
    void add(uint32_t k) {
        if (has(k)) return;
        if (v.size() >= 512) v.erase(v.begin());
        v.push_back(k);
    }
};
KeySet s_tracker_alerted;       // trackers already alerted for
KeySet s_known;                 // new-device watch: what counts as normal
bool   s_watch = false;

// Left-behind state, one per saved slot.
bool s_heard[kMaxSaved];
bool s_alarmed[kMaxSaved];

uint32_t fnv(const void *data, size_t n, uint32_t h = 2166136261u) {
    const uint8_t *p = (const uint8_t *)data;
    for (size_t i = 0; i < n; ++i) { h ^= p[i]; h *= 16777619u; }
    return h ? h : 1;
}

uint32_t addr_key(const DeviceView &d) {
    uint32_t h = fnv(d.addr, 6);
    return fnv(&d.addr_type, 1, h);
}

// What makes a device "the same one" for the new-device watch. Phones and
// many gadgets change address every few minutes, so an address alone would
// make them look new over and over. 0 = can't be recognised, don't alert.
uint32_t identity_key(const DeviceView &d) {
    if (d.kind == Kind::WifiAP) return 0;
    if (d.name[0]) return fnv(d.name, strnlen(d.name, sizeof d.name));
    if (d.kind == Kind::AirPods && d.sig) return fnv(&d.sig, sizeof d.sig);
    if (d.kind == Kind::Tile || d.kind == Kind::SmartTag || d.away || d.stable_addr) return addr_key(d);
    return 0;
}

bool is_tracker(const DeviceView &d) {
    return d.kind == Kind::Tile || d.kind == Kind::SmartTag || (d.kind == Kind::FindMy && d.away);
}

void raise(AlertType type, const DeviceView &d, const char *title, const char *line) {
    s_alert.active = true;
    s_alert.type = type;
    s_alert.dev = d;
    s_alert.since_ms = now_ms();
    snprintf(s_alert.title, sizeof s_alert.title, "%s", title);
    snprintf(s_alert.line, sizeof s_alert.line, "%s", line);
}

const char *shown_name(const DeviceView &d) {
    Saved *s = saved_find(d);
    return s && s->name[0] ? s->name : d.label;
}

void check_trackers() {
    for (const auto &d : s_list) {
        if (!is_tracker(d) || d.age_ms > 60000) continue;
        if (d.seen_ms < kFollowMs || d.moved_s < kFollowMovedS) continue;
        uint32_t k = addr_key(d);
        if (s_tracker_alerted.has(k)) continue;
        Saved *s = saved_find(d);
        if (s && (s->flags & (kSavedIgnore | kSavedPin))) continue;      // one of yours
        s_tracker_alerted.add(k);
        char line[112];
        snprintf(line, sizeof line, "%s has been near you for %lu minutes while you were moving.",
                 d.label, (unsigned long)(d.seen_ms / 60000));
        raise(AlertType::Tracker, d, "A tracker may be travelling with you", line);
        return;
    }
}

void check_new() {
    for (const auto &d : s_list) {
        uint32_t k = identity_key(d);
        if (!k || s_known.has(k)) continue;
        if (d.age_ms > 5000 || d.count < 3 || d.rssi < kNewMinRssi) continue;
        s_known.add(k);
        char line[112];
        snprintf(line, sizeof line, "%s  (%.0f dBm)", shown_name(d), d.rssi);
        raise(AlertType::NewDevice, d, "New device nearby", line);
        return;
    }
}

void check_left_behind() {
    for (size_t i = 0; i < saved_count() && i < kMaxSaved; ++i) {
        Saved *s = saved_at(i);
        if (!(s->flags & kSavedWatch)) { s_heard[i] = s_alarmed[i] = false; continue; }
        uint32_t age = devices_age_of(s->addr, s->addr_type, s->sig, (s->flags & kSavedBySig) != 0);
        if (age < 10000) { s_heard[i] = true; s_alarmed[i] = false; continue; }
        if (!s_heard[i] || s_alarmed[i] || age < kGoneMs) continue;
        s_alarmed[i] = true;
        DeviceView d{};
        memcpy(d.addr, s->addr, 6);
        d.addr_type = s->addr_type;
        d.sig = s->sig;
        d.kind = (s->flags & kSavedBySig) ? Kind::AirPods : Kind::Named;
        snprintf(d.label, sizeof d.label, "%s", s->name[0] ? s->name : "Your device");
        char line[112];
        snprintf(line, sizeof line, "%s hasn't been heard for a minute. Did you leave it behind?", d.label);
        raise(AlertType::LeftBehind, d, "Out of range", line);
        return;
    }
}

} // namespace

void alerts_init() {
    nvs_handle_t h;
    if (nvs_open("blefind", NVS_READONLY, &h) != ESP_OK) return;
    uint8_t v;
    if (nvs_get_u8(h, "trk", &v) == ESP_OK) s_tracker_on = v != 0;
    nvs_close(h);
}

void alerts_tick() {
    uint32_t t = now_ms();
    if (t - s_last_tick < 1000) return;
    s_last_tick = t;
    if (s_alert.active) return;             // one at a time; the rest wait

    check_left_behind();
    if (s_alert.active || !(s_tracker_on || s_watch)) return;
    devices_snapshot(s_list, Filter::All, 60000);
    if (s_tracker_on) check_trackers();
    if (!s_alert.active && s_watch) check_new();
}

const Alert &alerts_current() { return s_alert; }
void alerts_dismiss() { s_alert.active = false; }

void alerts_ignore_device() {
    if (s_alert.active && s_alert.type == AlertType::Tracker) {
        if (Saved *s = saved_get(s_alert.dev)) {
            s->flags |= kSavedIgnore;
            saved_store();
        }
    }
    s_alert.active = false;
}

bool alerts_tracker_enabled() { return s_tracker_on; }

void alerts_set_tracker_enabled(bool on) {
    s_tracker_on = on;
    nvs_handle_t h;
    if (nvs_open("blefind", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "trk", on ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
}

bool alerts_watch_active() { return s_watch; }
size_t alerts_watch_known() { return s_known.v.size(); }

void alerts_watch_start() {
    s_known.v.clear();
    devices_snapshot(s_list, Filter::All, 120000);
    for (const auto &d : s_list)
        if (uint32_t k = identity_key(d)) s_known.add(k);
    s_watch = true;
}

void alerts_watch_stop() { s_watch = false; }
