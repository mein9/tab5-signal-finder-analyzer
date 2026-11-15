#include "devices.h"

#include <string.h>
#include <math.h>
#include <stdio.h>
#include <algorithm>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"

namespace {

constexpr size_t kMaxDevices = 256;
constexpr size_t kMaxSamples = 2048;

struct Device {
    bool     used;
    uint8_t  addr[6];
    uint8_t  addr_type;
    char     name[32];
    Kind     kind;
    uint32_t sig;
    uint16_t company;
    uint8_t  apple_type;
    uint16_t model;
    uint8_t  findmy_len;
    uint8_t  channel;      // Wi-Fi
    uint8_t  auth;         // Wi-Fi, wifi_auth_mode_t
    float    ema;
    float    rank;         // strength used for list order; 0 = not set yet
    int8_t   peak;
    uint32_t first_ms;
    uint32_t last_ms;
    uint32_t count;
    uint32_t motion0;      // s_motion_s when first heard
};

constexpr float   kRankStep = 6.0f;   // dB a signal must change before its row moves

Device            s_dev[kMaxDevices];
Sample            s_samples[kMaxSamples];
size_t            s_sample_head = 0;   // next write index
size_t            s_sample_n = 0;
Target            s_target;
uint32_t          s_heard = 0;
uint32_t          s_motion_s = 0;
SemaphoreHandle_t s_lock;

struct Lock {
    Lock() { xSemaphoreTake(s_lock, portMAX_DELAY); }
    ~Lock() { xSemaphoreGive(s_lock); }
};

// Apple proximity-pairing model ids (bytes 2-3 of the 0x07 message).
// Best-effort: unknown ids fall back to "AirPods / Beats".
struct Model { uint16_t id; const char *name; };
const Model kAppleModels[] = {
    {0x0220, "AirPods (1st gen)"},   {0x0F20, "AirPods (2nd gen)"},
    {0x1320, "AirPods (3rd gen)"},   {0x1920, "AirPods 4"},
    {0x1B20, "AirPods 4 (ANC)"},     {0x0E20, "AirPods Pro"},
    {0x1420, "AirPods Pro 2"},       {0x2420, "AirPods Pro 2 (USB-C)"},
    {0x0A20, "AirPods Max"},         {0x1F20, "AirPods Max (USB-C)"},
    {0x0320, "Powerbeats3"},         {0x0520, "BeatsX"},
    {0x0620, "Beats Solo3"},         {0x0920, "Beats Studio3"},
    {0x0B20, "Powerbeats Pro"},      {0x0C20, "Beats Solo Pro"},
    {0x1020, "Beats Flex"},          {0x1120, "Beats Studio Buds"},
    {0x1220, "Beats Fit Pro"},       {0x1620, "Beats Studio Buds+"},
    {0x1720, "Beats Studio Pro"},    {0x1D20, "Powerbeats Pro 2"},
};

const char *apple_model_name(uint16_t id) {
    for (const auto &m : kAppleModels) if (m.id == id) return m.name;
    return nullptr;
}

bool same_addr(const uint8_t a[6], const uint8_t b[6]) { return memcmp(a, b, 6) == 0; }

// Rank how specific a classification is, so a later, vaguer packet
// from the same address doesn't overwrite a better one.
int rank(Kind k) {
    switch (k) {
    case Kind::AirPods:  return 9;
    case Kind::FindMy:   return 8;
    case Kind::SmartTag: return 8;
    case Kind::Tile:     return 8;
    case Kind::FastPair: return 6;
    case Kind::Apple:    return 4;
    case Kind::Samsung:  return 4;
    case Kind::Microsoft:return 3;
    case Kind::Named:    return 2;
    default:             return 0;
    }
}

struct Parsed {
    char     name[32] = {0};
    Kind     kind = Kind::Unknown;
    uint32_t sig = 0;
    uint16_t company = 0xFFFF;
    uint8_t  apple_type = 0;
    uint16_t model = 0;
    uint8_t  findmy_len = 0;
};

void parse_apple(const uint8_t *p, size_t n, Parsed &r) {
    // Apple manufacturer data is a list of [type, length, payload...].
    size_t i = 0;
    while (i + 2 <= n) {
        uint8_t type = p[i], len = p[i + 1];
        const uint8_t *pl = p + i + 2;
        size_t avail = n - (i + 2);
        if (len > avail) len = (uint8_t)avail;
        if (type == 0x07 && len >= 3) {                 // proximity pairing
            r.kind = Kind::AirPods;
            r.apple_type = 0x07;
            r.model = (uint16_t)((pl[1] << 8) | pl[2]);
            r.sig = 0x07000000u | r.model;
        } else if (type == 0x12 && rank(r.kind) < rank(Kind::FindMy)) { // Find My
            r.kind = Kind::FindMy;
            r.apple_type = 0x12;
            r.findmy_len = len;
            r.sig = 0x12000000u;
        } else if (rank(r.kind) < rank(Kind::Apple)) {
            r.kind = Kind::Apple;
            r.apple_type = type;
        }
        i += 2 + len;
    }
    if (r.kind == Kind::Unknown) r.kind = Kind::Apple;
}

void bump(Parsed &r, Kind k, uint32_t sig = 0) {
    if (rank(k) > rank(r.kind)) { r.kind = k; if (sig) r.sig = sig; }
}

Parsed parse(const uint8_t *d, size_t n) {
    Parsed r;
    size_t i = 0;
    while (i < n) {
        uint8_t l = d[i];
        if (l == 0 || i + 1 + l > n) break;
        uint8_t t = d[i + 1];
        const uint8_t *p = d + i + 2;
        size_t pl = l - 1;
        switch (t) {
        case 0x08: case 0x09: {                         // short / complete name
            size_t c = std::min(pl, sizeof(r.name) - 1);
            if (t == 0x09 || r.name[0] == 0) {
                memcpy(r.name, p, c);
                r.name[c] = 0;
                for (size_t k = 0; k < c; ++k)
                    if ((uint8_t)r.name[k] < 0x20 || (uint8_t)r.name[k] > 0x7E) r.name[k] = '?';
            }
            break;
        }
        case 0xFF:                                      // manufacturer data
            if (pl >= 2) {
                uint16_t co = (uint16_t)(p[0] | (p[1] << 8));
                r.company = co;
                if (co == 0x004C) parse_apple(p + 2, pl - 2, r);
                else if (co == 0x0075) bump(r, Kind::Samsung);
                else if (co == 0x0006) bump(r, Kind::Microsoft);
            }
            break;
        case 0x02: case 0x03:                           // 16-bit service UUIDs
        case 0x16:                                      // 16-bit service data
            for (size_t k = 0; k + 1 < pl; k += 2) {
                uint16_t u = (uint16_t)(p[k] | (p[k + 1] << 8));
                if (u == 0xFEED || u == 0xFEEC || u == 0xFD84) bump(r, Kind::Tile);
                else if (u == 0xFD5A || u == 0xFD69) bump(r, Kind::SmartTag);
                else if (u == 0xFE2C) bump(r, Kind::FastPair);
                if (t == 0x16) break;                   // service data: only first UUID
            }
            break;
        default:
            break;
        }
        i += 1 + l;
    }
    if (r.kind == Kind::Unknown && r.name[0]) r.kind = Kind::Named;
    return r;
}

void describe(const Device &d, DeviceView &v) {
    char addr[18];
    snprintf(addr, sizeof addr, "%02X:%02X:%02X:%02X:%02X:%02X",
             d.addr[5], d.addr[4], d.addr[3], d.addr[2], d.addr[1], d.addr[0]);
    const char *model = d.kind == Kind::AirPods ? apple_model_name(d.model) : nullptr;

    switch (d.kind) {
    case Kind::AirPods:
        snprintf(v.label, sizeof v.label, "%s", model ? model : "AirPods / Beats");
        snprintf(v.detail, sizeof v.detail, "Case open or in use  |  id %04X", d.model);
        break;
    case Kind::FindMy:
        snprintf(v.label, sizeof v.label, "Find My item");
        snprintf(v.detail, sizeof v.detail, "%s",
                 d.findmy_len > 2 ? "Away from its owner (AirTag, AirPods...)"
                                  : "Near its owner (AirTag, AirPods...)");
        break;
    case Kind::Apple:
        snprintf(v.label, sizeof v.label, "%s", d.name[0] ? d.name : "Apple device");
        snprintf(v.detail, sizeof v.detail, "iPhone / iPad / Mac / Watch  |  %s", addr);
        break;
    case Kind::Tile:
        snprintf(v.label, sizeof v.label, "Tile tracker");
        snprintf(v.detail, sizeof v.detail, "%s", addr);
        break;
    case Kind::SmartTag:
        snprintf(v.label, sizeof v.label, "Galaxy SmartTag");
        snprintf(v.detail, sizeof v.detail, "%s", addr);
        break;
    case Kind::Samsung:
        snprintf(v.label, sizeof v.label, "%s", d.name[0] ? d.name : "Samsung device");
        snprintf(v.detail, sizeof v.detail, "Samsung  |  %s", addr);
        break;
    case Kind::FastPair:
        snprintf(v.label, sizeof v.label, "%s", d.name[0] ? d.name : "Fast Pair device");
        snprintf(v.detail, sizeof v.detail, "Google Fast Pair (earbuds etc.)  |  %s", addr);
        break;
    case Kind::Microsoft:
        snprintf(v.label, sizeof v.label, "%s", d.name[0] ? d.name : "Windows device");
        snprintf(v.detail, sizeof v.detail, "Microsoft  |  %s", addr);
        break;
    case Kind::Named:
        snprintf(v.label, sizeof v.label, "%s", d.name);
        snprintf(v.detail, sizeof v.detail, "%s", addr);
        break;
    case Kind::WifiAP: {
        static const char *kAuth[] = {"Open", "WEP", "WPA", "WPA2", "WPA/WPA2", "WPA2 Enterprise",
                                      "WPA3", "WPA2/WPA3", "WAPI", "Open (encrypted)"};
        const char *auth = d.auth < sizeof kAuth / sizeof kAuth[0] ? kAuth[d.auth] : "Secured";
        snprintf(v.label, sizeof v.label, "%s", d.name[0] ? d.name : "Hidden network");
        snprintf(v.detail, sizeof v.detail, "Channel %u  |  %s  |  %s", (unsigned)d.channel, auth, addr);
        break;
    }
    default:
        snprintf(v.label, sizeof v.label, "Unnamed device");
        if (d.company != 0xFFFF)
            snprintf(v.detail, sizeof v.detail, "maker 0x%04X  |  %s", d.company, addr);
        else
            snprintf(v.detail, sizeof v.detail, "%s", addr);
        break;
    }
}

bool matches_target(const Device &d) {
    if (!s_target.active) return false;
    if (s_target.by_sig) return d.sig != 0 && d.sig == s_target.sig;
    return same_addr(d.addr, s_target.addr);
}

} // namespace

uint32_t now_ms() { return (uint32_t)(esp_timer_get_time() / 1000); }

void devices_init() {
    s_lock = xSemaphoreCreateMutex();
    memset(s_dev, 0, sizeof s_dev);
}

void devices_feed(const uint8_t addr[6], uint8_t addr_type, int8_t rssi,
                  const uint8_t *data, size_t len) {
    if (rssi == 127 || rssi < -110) return;            // 127 = not available
    Parsed p = parse(data, len);
    uint32_t t = now_ms();

    Lock lock;
    ++s_heard;

    Device *d = nullptr, *free_slot = nullptr, *oldest = nullptr;
    for (auto &e : s_dev) {
        if (!e.used) { if (!free_slot) free_slot = &e; continue; }
        if (e.addr_type == addr_type && same_addr(e.addr, addr)) { d = &e; break; }
        if (!oldest || e.last_ms < oldest->last_ms) oldest = &e;
    }
    if (!d) {
        d = free_slot ? free_slot : oldest;
        memset(d, 0, sizeof *d);
        d->used = true;
        memcpy(d->addr, addr, 6);
        d->addr_type = addr_type;
        d->company = 0xFFFF;
        d->ema = rssi;
        d->peak = rssi;
        d->first_ms = t;
        d->motion0 = s_motion_s;
    }

    if (p.name[0]) memcpy(d->name, p.name, sizeof d->name);
    if (p.company != 0xFFFF) d->company = p.company;
    if (rank(p.kind) >= rank(d->kind) && p.kind != Kind::Unknown) {
        d->kind = p.kind;
        if (p.sig) d->sig = p.sig;
        if (p.apple_type) d->apple_type = p.apple_type;
        if (p.model) d->model = p.model;
        if (p.findmy_len) d->findmy_len = p.findmy_len;
    }
    if (d->kind == Kind::Unknown && d->name[0]) d->kind = Kind::Named;

    // Smooth more when packets arrive in bursts.
    float a = (t - d->last_ms > 1500) ? 0.6f : 0.25f;
    d->ema = d->count ? d->ema + a * (rssi - d->ema) : rssi;
    if (rssi > d->peak) d->peak = rssi;
    d->last_ms = t;
    ++d->count;

    if (matches_target(*d)) {
        s_samples[s_sample_head] = {t, rssi};
        s_sample_head = (s_sample_head + 1) % kMaxSamples;
        if (s_sample_n < kMaxSamples) ++s_sample_n;
    }
}

void devices_feed_wifi(const uint8_t bssid[6], const char *ssid, int8_t rssi,
                       uint8_t channel, uint8_t authmode) {
    uint32_t t = now_ms();
    // Show BSSIDs in the same byte order as Bluetooth addresses (reversed in storage).
    uint8_t addr[6];
    for (int i = 0; i < 6; ++i) addr[i] = bssid[5 - i];

    Lock lock;
    Device *d = nullptr, *free_slot = nullptr, *oldest = nullptr;
    for (auto &e : s_dev) {
        if (!e.used) { if (!free_slot) free_slot = &e; continue; }
        if (e.addr_type == kAddrTypeWifi && same_addr(e.addr, addr)) { d = &e; break; }
        if (!oldest || e.last_ms < oldest->last_ms) oldest = &e;
    }
    if (!d) {
        d = free_slot ? free_slot : oldest;
        memset(d, 0, sizeof *d);
        d->used = true;
        memcpy(d->addr, addr, 6);
        d->addr_type = kAddrTypeWifi;
        d->kind = Kind::WifiAP;
        d->company = 0xFFFF;
        d->ema = rssi;
        d->peak = rssi;
        d->first_ms = t;
        d->motion0 = s_motion_s;
    }
    size_t n = strnlen(ssid, 32);
    if (n) {
        size_t c = std::min(n, sizeof d->name - 1);
        memcpy(d->name, ssid, c);
        d->name[c] = 0;
        for (size_t k = 0; k < c; ++k)
            if ((uint8_t)d->name[k] < 0x20 || (uint8_t)d->name[k] > 0x7E) d->name[k] = '?';
    }
    d->channel = channel;
    d->auth = authmode;
    // One reading per scan, so follow changes faster than for Bluetooth.
    d->ema = d->count ? d->ema + 0.5f * (rssi - d->ema) : rssi;
    if (rssi > d->peak) d->peak = rssi;
    d->last_ms = t;
    ++d->count;

    if (matches_target(*d)) {
        s_samples[s_sample_head] = {t, rssi};
        s_sample_head = (s_sample_head + 1) % kMaxSamples;
        if (s_sample_n < kMaxSamples) ++s_sample_n;
    }
}

static bool passes(const Device &d, Filter f) {
    if (f == Filter::Any) return true;
    if ((f == Filter::Wifi) != (d.kind == Kind::WifiAP)) return false;
    switch (f) {
    case Filter::Findable:
        return d.kind == Kind::AirPods || d.kind == Kind::FindMy || d.kind == Kind::Tile ||
               d.kind == Kind::SmartTag || d.kind == Kind::FastPair;
    case Filter::Named:
        return d.kind != Kind::Unknown;
    default:
        return true;
    }
}

void devices_snapshot(std::vector<DeviceView> &out, Filter filter, uint32_t max_age_ms) {
    out.clear();
    uint32_t t = now_ms();
    {
        Lock lock;
        for (auto &d : s_dev) {
            if (!d.used || t - d.last_ms > max_age_ms || !passes(d, filter)) continue;
            // The list is ordered by a strength that only follows the real one
            // once it has moved by more than kRankStep, so rows don't keep
            // swapping places over normal signal flutter.
            if (d.rank == 0 || fabsf(d.ema - d.rank) > kRankStep) d.rank = d.ema;
            DeviceView v{};
            v.rank = d.rank;
            memcpy(v.addr, d.addr, 6);
            v.addr_type = d.addr_type;
            v.kind = d.kind;
            v.sig = d.sig;
            v.rssi = d.ema;
            v.peak = d.peak;
            v.age_ms = t - d.last_ms;
            v.count = d.count;
            v.channel = d.channel;
            memcpy(v.name, d.name, sizeof v.name);
            v.seen_ms = t - d.first_ms;
            v.moved_s = s_motion_s - d.motion0;
            v.away = d.kind == Kind::FindMy && d.findmy_len > 2;
            // Public addresses, Wi-Fi, and "static random" ones (top bits 11) stay put.
            v.stable_addr = d.addr_type != 1 || (d.addr[5] & 0xC0) == 0xC0;
            describe(d, v);
            out.push_back(v);
        }
    }
    std::sort(out.begin(), out.end(),
              [](const DeviceView &a, const DeviceView &b) {
                  if (a.rank != b.rank) return a.rank > b.rank;
                  return memcmp(a.addr, b.addr, 6) < 0;
              });
}

uint32_t devices_age_of(const uint8_t addr[6], uint8_t addr_type, uint32_t sig, bool by_sig) {
    uint32_t t = now_ms(), best = UINT32_MAX;
    Lock lock;
    for (const auto &d : s_dev) {
        if (!d.used) continue;
        bool hit = by_sig ? (sig != 0 && d.sig == sig)
                          : (d.addr_type == addr_type && same_addr(d.addr, addr));
        if (hit) best = std::min(best, t - d.last_ms);
    }
    return best;
}

void devices_set_motion_s(uint32_t seconds) { Lock lock; s_motion_s = seconds; }

uint32_t devices_heard_total() { Lock lock; return s_heard; }

size_t devices_count(uint32_t max_age_ms) {
    uint32_t t = now_ms();
    size_t n = 0;
    Lock lock;
    for (const auto &d : s_dev) if (d.used && t - d.last_ms <= max_age_ms) ++n;
    return n;
}

void target_set(const DeviceView &dev, bool follow_type) {
    Lock lock;
    s_target.active = true;
    s_target.by_sig = follow_type && dev.sig != 0;
    s_target.sig = dev.sig;
    memcpy(s_target.addr, dev.addr, 6);
    s_target.wifi_channel = dev.kind == Kind::WifiAP ? dev.channel : 0;
    snprintf(s_target.label, sizeof s_target.label, "%s", dev.label);
    s_sample_head = 0;
    s_sample_n = 0;
}

void target_clear() {
    Lock lock;
    s_target.active = false;
    s_sample_n = 0;
}

void target_rename(const char *label) {
    Lock lock;
    snprintf(s_target.label, sizeof s_target.label, "%s", label);
}

Target target_get() { Lock lock; return s_target; }

void target_samples(std::vector<Sample> &out, uint32_t since_ms) {
    out.clear();
    Lock lock;
    size_t start = (s_sample_head + kMaxSamples - s_sample_n) % kMaxSamples;
    for (size_t k = 0; k < s_sample_n; ++k) {
        const Sample &s = s_samples[(start + k) % kMaxSamples];
        if ((int32_t)(s.t_ms - since_ms) >= 0) out.push_back(s);
    }
}
