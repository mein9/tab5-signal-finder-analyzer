#include "saved.h"

#include <string.h>

#include "nvs.h"

namespace {

Saved  s_saved[kMaxSaved];
size_t s_n = 0;

// AirPods and similar change address every few minutes but keep broadcasting
// the same model id, so those are matched by type. Everything else is matched
// by address.
bool use_sig(const DeviceView &d) { return d.kind == Kind::AirPods && d.sig != 0; }

bool matches(const Saved &s, const DeviceView &d) {
    if (s.flags & kSavedBySig) return d.sig != 0 && d.sig == s.sig;
    return s.addr_type == d.addr_type && memcmp(s.addr, d.addr, 6) == 0;
}

} // namespace

void saved_load() {
    s_n = 0;
    nvs_handle_t h;
    if (nvs_open("blefind", NVS_READONLY, &h) != ESP_OK) return;
    size_t len = sizeof s_saved;
    if (nvs_get_blob(h, "saved", s_saved, &len) == ESP_OK && len % sizeof(Saved) == 0)
        s_n = len / sizeof(Saved);
    nvs_close(h);
    for (size_t i = 0; i < s_n; ++i) s_saved[i].name[sizeof s_saved[i].name - 1] = 0;
}

Saved *saved_find(const DeviceView &dev) {
    for (size_t i = 0; i < s_n; ++i)
        if (matches(s_saved[i], dev)) return &s_saved[i];
    return nullptr;
}

Saved *saved_get(const DeviceView &dev) {
    if (Saved *s = saved_find(dev)) return s;
    if (s_n >= kMaxSaved) return nullptr;
    Saved &s = s_saved[s_n++];
    memset(&s, 0, sizeof s);
    memcpy(s.addr, dev.addr, 6);
    s.addr_type = dev.addr_type;
    s.sig = dev.sig;
    if (use_sig(dev)) s.flags = kSavedBySig;
    return &s;
}

void saved_store() {
    size_t w = 0;
    for (size_t i = 0; i < s_n; ++i) {
        bool keep = (s_saved[i].flags & ~kSavedBySig) != 0 || s_saved[i].name[0] != 0;
        if (keep) s_saved[w++] = s_saved[i];
    }
    s_n = w;
    nvs_handle_t h;
    if (nvs_open("blefind", NVS_READWRITE, &h) != ESP_OK) return;
    if (s_n) nvs_set_blob(h, "saved", s_saved, s_n * sizeof(Saved));
    else nvs_erase_key(h, "saved");
    nvs_commit(h);
    nvs_close(h);
}

void saved_clear() {
    s_n = 0;
    saved_store();
}

size_t saved_count() { return s_n; }
Saved *saved_at(size_t i) { return i < s_n ? &s_saved[i] : nullptr; }
