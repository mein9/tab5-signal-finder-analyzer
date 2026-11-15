#include "sdr.h"

#include <M5Unified.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>

#include "esp_heap_caps.h"
#include "esp_log.h"
#if FINDER_HAS_SDR
#include "esp_rtl_sdr.h"
#endif
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace {

const char *TAG = "finder_sdr";

// 1 = start sweeping at boot and log every pass (for testing without the screen).
#define SDR_SELFTEST 0

constexpr uint32_t kRate = 2048000;                 // each slice sees ~2 MHz
constexpr uint32_t kStepHz = 2000000;
constexpr size_t   kSettleBytes = 49152;            // ~12 ms thrown away after each retune
constexpr size_t   kMeasureBytes = 65536;           // ~16 ms measured
constexpr size_t   kBlock = 2048;                   // bytes per power block (0.5 ms)
constexpr int      kGainTenthDb = 297;              // normal gain: sensitive, for distant phones
constexpr int      kLowGainTenthDb = 0;             // used when a signal overloads the receiver
constexpr int      kFullPassEvery = 8;              // when following a band: full pass every Nth time
constexpr size_t   kMaxSamples = 1024;

// Where phones transmit (uplink) on the US bands an RTL-SDR can reach.
struct BandDef { const char *name, *detail; uint32_t lo_hz, hi_hz; };
const BandDef kBands[] = {
    {"600 MHz", "Band 71  |  663-698 MHz", 663000000, 698000000},
    {"700 MHz lower", "Band 12 / 17  |  699-716 MHz", 699000000, 716000000},
    {"700 MHz upper", "Band 13 / 14  |  777-798 MHz", 777000000, 798000000},
    {"850 MHz", "Band 5 / 26  |  814-849 MHz", 814000000, 849000000},
    {"1700 MHz AWS", "Band 4 / 66  |  1710-1780 MHz", 1710000000, 1780000000},
    {"1900 MHz PCS", "Band 2 / 25  |  1850-1915 MHz", 1850000000, 1915000000},
};
constexpr size_t kNumBands = sizeof kBands / sizeof kBands[0];

struct BandLive {
    bool     in_range = true;
    bool     valid = false;
    float    level = -120, floor = 0;
    uint32_t peak_hz = 0, at_ms = 0;
};

volatile SdrState s_state = SdrState::Off;
volatile bool     s_wanted = false;
volatile int      s_focus = -1;
volatile bool     s_cycle_wanted = false;     // ask the UI task to power-cycle the USB port
char              s_status[112] = "Phone bands are off";
char              s_dongle[64] = "";
BandLive          s_live[kNumBands];
Sample            s_samples[kMaxSamples];
size_t            s_head = 0, s_n = 0;
SemaphoreHandle_t s_lock;
#if FINDER_HAS_SDR
esp_rtl_sdr_handle_t s_sdr = nullptr;
uint8_t          *s_buf = nullptr;
#endif

struct Lock {
    Lock() { xSemaphoreTake(s_lock, portMAX_DELAY); }
    ~Lock() { xSemaphoreGive(s_lock); }
};

void set_status(const char *fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    Lock lock;
    vsnprintf(s_status, sizeof s_status, fmt, ap);
    va_end(ap);
}

#if FINDER_HAS_SDR && !FINDER_DEMO

// Reads exactly `want` bytes of I/Q (or gives up after ~0.5 s).
bool read_exact(uint8_t *dst, size_t want) {
    size_t got = 0;
    int idle = 0;
    while (got < want && idle < 5) {
        size_t n = 0;
        esp_err_t err = esp_rtl_sdr_read(s_sdr, dst + got, want - got, 100, &n);
        if (err == ESP_OK && n) { got += n; idle = 0; }
        else ++idle;
    }
    return got == want;
}

// Strength of the loudest half-millisecond in the buffer, in dB below full
// scale. Phones transmit in short bursts, so the peak matters, not the average.
// *clipped is set when the receiver was overloaded.
float burst_power_db(const uint8_t *iq, size_t n, bool *clipped) {
    float best = 0;
    size_t hot = 0;
    for (size_t off = 0; off + kBlock <= n; off += kBlock) {
        const uint8_t *p = iq + off;
        int32_t si = 0, sq = 0;
        uint32_t ss = 0;
        for (size_t k = 0; k < kBlock; k += 2) {
            int i = p[k], q = p[k + 1];
            si += i;
            sq += q;
            ss += (uint32_t)(i * i + q * q);
            hot += (i <= 1 || i >= 254) + (q <= 1 || q >= 254);
        }
        // Remove the constant offset every RTL-SDR has, then take the mean power.
        const float pairs = kBlock / 2;
        float mi = si / pairs, mq = sq / pairs;
        float pw = ss / pairs - mi * mi - mq * mq;
        best = std::max(best, pw);
    }
    *clipped = hot > n / 50;
    const float full = 2.0f * 127.5f * 127.5f;
    return 10.0f * log10f(std::max(best, 0.01f) / full);
}

// Measures one 2 MHz slice. Returns false if the dongle stopped answering.
// Readings are reported with the tuner gain subtracted, so a slice measured
// at low gain lines up with the others.
bool measure(uint32_t hz, float *out_db) {
    esp_err_t err = esp_rtl_sdr_retune_hz(s_sdr, hz);
    if (err != ESP_OK) {
        *out_db = NAN;                      // can't tune here; not fatal
        return esp_rtl_sdr_get_state(s_sdr) == ESP_RTL_SDR_STATE_STREAMING;
    }
    if (!read_exact(s_buf, kSettleBytes)) return false;
    if (!read_exact(s_buf, kMeasureBytes)) return false;
    bool clipped = false;
    float db = burst_power_db(s_buf, kMeasureBytes, &clipped);
    int gain = kGainTenthDb;
    if (clipped && esp_rtl_sdr_set_tuner_gain(s_sdr, kLowGainTenthDb) == ESP_OK) {
        // Too strong for the normal gain: measure again with the gain turned down.
        gain = kLowGainTenthDb;
        bool ok = read_exact(s_buf, kSettleBytes) && read_exact(s_buf, kMeasureBytes);
        if (ok) db = burst_power_db(s_buf, kMeasureBytes, &clipped);
        esp_rtl_sdr_set_tuner_gain(s_sdr, kGainTenthDb);
        if (!ok) return false;
    }
    *out_db = db - gain / 10.0f + (clipped ? 3.0f : 0.0f);     // still clipped: at least this strong
    return true;
}

// Measures a band and records the strongest slice. With `around_hz` set, only
// that slice and its two neighbours are measured, for quick updates.
bool sweep_band(size_t b, uint32_t around_hz = 0) {
    const BandDef &def = kBands[b];
    uint32_t first = def.lo_hz + kStepHz / 2, last = def.hi_hz + kStepHz / 2;
    if (around_hz) {
        first = std::max(first, around_hz - kStepHz);
        last = std::min(last, around_hz + 2 * kStepHz);
    }
    float best = -200;
    uint32_t best_hz = 0;
    bool any = false;
    for (uint32_t hz = first; hz < last; hz += kStepHz) {
        if (!s_wanted) return true;
        float db;
        if (!measure(hz, &db)) return false;
        if (isnan(db)) continue;
        any = true;
        if (db > best) { best = db; best_hz = hz; }
    }
    uint32_t t = now_ms();
    Lock lock;
    BandLive &l = s_live[b];
    if (!around_hz) l.in_range = any;
    if (!any) return true;
    // The quiet level creeps up slowly and drops at once, so it tracks the
    // noise floor rather than the bursts.
    l.floor = l.valid ? std::min(l.floor + 0.02f, best) : best;
    l.level = best;
    l.peak_hz = best_hz;
    l.at_ms = t;
    l.valid = true;
    if (s_focus == (int)b) {
        s_samples[s_head] = {t, (int8_t)std::clamp(best, -127.0f, 0.0f)};
        s_head = (s_head + 1) % kMaxSamples;
        if (s_n < kMaxSamples) ++s_n;
    }
    return true;
}

bool wait_for_dongle() {
    size_t n = 0;
    esp_rtl_sdr_refresh_device_list(s_sdr);
    return esp_rtl_sdr_get_device_count(s_sdr, &n) == ESP_OK && n > 0;
}

void sdr_task(void *) {
    while (!s_wanted) vTaskDelay(pdMS_TO_TICKS(200));

    s_buf = (uint8_t *)heap_caps_malloc(kMeasureBytes, MALLOC_CAP_SPIRAM);
    esp_rtl_sdr_config_t cfg;
    esp_rtl_sdr_config_default(&cfg);
    cfg.delivery_mode = ESP_RTL_SDR_DELIVERY_READ;
    esp_err_t err = s_buf ? esp_rtl_sdr_install(&cfg, &s_sdr) : ESP_ERR_NO_MEM;
    if (err != ESP_OK) {
        set_status("USB didn't start (%s)", esp_rtl_sdr_err_to_name(err));
        ESP_LOGE(TAG, "install: %s", esp_rtl_sdr_err_to_name(err));
        s_state = SdrState::Failed;
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG, "driver %s ready", esp_rtl_sdr_get_version_string());
    esp_log_level_set("esp_rtl_sdr", ESP_LOG_WARN);     // it logs every device-list poll otherwise

    int misses = 0, cycles = 0;
    for (;;) {
        if (!s_wanted) {
            if (esp_rtl_sdr_get_state(s_sdr) == ESP_RTL_SDR_STATE_STREAMING) esp_rtl_sdr_stop(s_sdr, 1000);
            s_state = SdrState::Off;
            set_status("Phone bands are off");
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (esp_rtl_sdr_get_state(s_sdr) != ESP_RTL_SDR_STATE_STREAMING) {
            if (!wait_for_dongle()) {
                s_state = SdrState::NoDongle;
                size_t on_bus = 0;
                esp_rtl_sdr_usb_device_count(&on_bus);
                set_status(on_bus ? "A USB device is connected, but it isn't a supported RTL-SDR"
                                  : "Plug an RTL-SDR into the Tab5's USB-A port");
                { Lock lock; s_dongle[0] = 0; }
                // Nothing on the bus for a while: try switching the port off and
                // on (twice at most, so an empty port doesn't keep interrupting).
                if (!on_bus && ++misses % 12 == 0 && cycles < 2) {
                    ++cycles;
                    s_cycle_wanted = true;
                }
                vTaskDelay(pdMS_TO_TICKS(700));
                continue;
            }
            misses = 0;
            cycles = 0;
            s_state = SdrState::Starting;
            set_status("Starting the receiver...");
            esp_rtl_sdr_device_info_t info = {};
            esp_rtl_sdr_get_device_info(s_sdr, &info);
            err = esp_rtl_sdr_start_hz(s_sdr, kBands[0].lo_hz, kRate);
            if (err != ESP_OK) {
                set_status("The receiver didn't start (%s)", esp_rtl_sdr_err_to_name(err));
                ESP_LOGW(TAG, "start: %s", esp_rtl_sdr_err_to_name(err));
                s_state = SdrState::Failed;
                vTaskDelay(pdMS_TO_TICKS(2000));
                continue;
            }
            esp_rtl_sdr_set_rtl_agc(s_sdr, false);
            esp_rtl_sdr_set_tuner_gain_mode(s_sdr, ESP_RTL_SDR_GAIN_MODE_MANUAL);
            esp_rtl_sdr_set_tuner_gain(s_sdr, kGainTenthDb);
            {
                Lock lock;
                snprintf(s_dongle, sizeof s_dongle, "%s",
                         esp_rtl_sdr_profile_to_name(esp_rtl_sdr_get_profile(s_sdr)));
                for (auto &l : s_live) l = BandLive{};
            }
            ESP_LOGI(TAG, "dongle: %s (%s %s, %04x:%04x)", s_dongle, info.manufacturer, info.product,
                     info.vid, info.pid);
            s_state = SdrState::Running;
            set_status("Receiving with %s", s_dongle);
        }

        int focus = s_focus;
        bool ok = true;
        uint32_t t0 = now_ms();
        if (focus >= 0 && focus < (int)kNumBands) {
            // Following one band: mostly watch its strongest spot, and now and
            // then re-check the whole band in case the activity moved.
            static int pass = 0;
            static int pass_band = -1;
            if (pass_band != focus) { pass_band = focus; pass = 0; }
            uint32_t peak = s_live[focus].valid ? s_live[focus].peak_hz : 0;
            ok = sweep_band((size_t)focus, (pass++ % kFullPassEvery) ? peak : 0);
        } else {
            for (size_t b = 0; b < kNumBands && ok && s_wanted && s_focus < 0; ++b) ok = sweep_band(b);
        }
        if (!ok) {
            ESP_LOGW(TAG, "receiver stopped answering; waiting for it to come back");
            esp_rtl_sdr_stop(s_sdr, 1000);
            s_state = SdrState::NoDongle;
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (SDR_SELFTEST) {
            char line[200];
            int n = snprintf(line, sizeof line, "pass %lu ms:", (unsigned long)(now_ms() - t0));
            for (size_t b = 0; b < kNumBands; ++b) {
                if (!s_live[b].in_range) n += snprintf(line + n, sizeof line - n, "  %s n/a", kBands[b].name);
                else n += snprintf(line + n, sizeof line - n, "  %s %.0f", kBands[b].name, s_live[b].level);
            }
            ESP_LOGI(TAG, "%s", line);
        }
    }
}

#elif FINDER_DEMO

// Demo mode: made-up band activity, no dongle needed.
void sdr_task(void *) {
    const float kFloor[kNumBands] = {-48, -48, -52, -59, -43, 0};
    for (;;) {
        if (!s_wanted) {
            s_state = SdrState::Off;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }
        s_state = SdrState::Running;
        set_status("Receiving with blog_v4_r828d");
        uint32_t t = now_ms();
        {
            Lock lock;
            snprintf(s_dongle, sizeof s_dongle, "blog_v4_r828d");
            for (size_t b = 0; b < kNumBands; ++b) {
                BandLive &l = s_live[b];
                l.in_range = b != 5;
                if (!l.in_range) continue;
                float act = b == 3 ? 16 + 10 * sinf(t / 2500.0f) + (int)(esp_random() % 5)
                          : b == 4 ? 4 + (int)(esp_random() % 6)
                                   : (int)(esp_random() % 3);
                l.floor = kFloor[b];
                l.level = kFloor[b] + act;
                l.peak_hz = kBands[b].lo_hz + (b == 3 ? 23000000 : 5000000);
                l.at_ms = t;
                l.valid = true;
                if (s_focus == (int)b) {
                    s_samples[s_head] = {t, (int8_t)l.level};
                    s_head = (s_head + 1) % kMaxSamples;
                    if (s_n < kMaxSamples) ++s_n;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(350));
    }
}

#else

// Built without the optional RTL-SDR driver (see the README).
void sdr_task(void *) {
    s_state = SdrState::Failed;
    set_status("This build doesn't include the RTL-SDR driver");
    vTaskDelete(nullptr);
}

#endif

} // namespace

// USB5V_EN is pin 3 of the second I/O expander (PI4IOE5V6408 at 0x44). Only
// that bit is touched: the same chip also powers the Wi-Fi/Bluetooth radio.
static void usb_power(bool on) {
    constexpr uint8_t kAddr = 0x44, kBit = 1 << 3;
    constexpr uint32_t kHz = 400000;
    if (on) M5.In_I2C.bitOn(kAddr, 0x05, kBit, kHz);
    else M5.In_I2C.bitOff(kAddr, 0x05, kBit, kHz);
    M5.In_I2C.bitOff(kAddr, 0x07, kBit, kHz);   // not high-impedance
    M5.In_I2C.bitOn(kAddr, 0x03, kBit, kHz);    // direction: output
}

void sdr_power_on() {
    // The port stays powered across a restart of the Tab5's main chip, which
    // can leave a dongle stuck mid-stream and invisible. Switching the port
    // off briefly makes the dongle start fresh.
    usb_power(false);
    vTaskDelay(pdMS_TO_TICKS(600));
    usb_power(true);
    bool on = M5.In_I2C.readRegister8(0x44, 0x05, 400000) & (1 << 3);
    ESP_LOGI(TAG, "USB-A power %s", on ? "on" : "FAILED");
}

bool sdr_wants_power_cycle() { return s_cycle_wanted; }

void sdr_power_cycle() {
    usb_power(false);
    vTaskDelay(pdMS_TO_TICKS(600));
    usb_power(true);
    s_cycle_wanted = false;
    ESP_LOGI(TAG, "USB-A power cycled");
}

void sdr_start() {
    s_lock = xSemaphoreCreateMutex();
    if (SDR_SELFTEST) s_wanted = true;
    xTaskCreatePinnedToCore(sdr_task, "sdr", 8192, nullptr, 4, nullptr, 1);
}

void sdr_set_wanted(bool wanted) { s_wanted = wanted || SDR_SELFTEST; }

void sdr_set_focus(int band) {
    if (band == s_focus) return;
    Lock lock;
    s_focus = band;
    s_head = s_n = 0;
}

int sdr_focus() { return s_focus; }
SdrState sdr_state() { return s_state; }

const char *sdr_status_text() {
    static char copy[sizeof s_status];
    Lock lock;
    memcpy(copy, s_status, sizeof copy);
    return copy;
}

const char *sdr_dongle_name() {
    static char copy[sizeof s_dongle];
    Lock lock;
    memcpy(copy, s_dongle, sizeof copy);
    return copy;
}

size_t sdr_band_count() { return kNumBands; }

SdrBand sdr_band(size_t i) {
    SdrBand b{};
    if (i >= kNumBands) return b;
    uint32_t t = now_ms();
    Lock lock;
    const BandLive &l = s_live[i];
    b.name = kBands[i].name;
    b.detail = kBands[i].detail;
    b.lo_hz = kBands[i].lo_hz;
    b.hi_hz = kBands[i].hi_hz;
    b.in_range = l.in_range;
    b.level_db = l.level;
    b.floor_db = l.floor;
    b.peak_hz = l.peak_hz;
    b.age_ms = t - l.at_ms;
    b.valid = l.valid;
    return b;
}

void sdr_samples(std::vector<Sample> &out, uint32_t since_ms) {
    out.clear();
    Lock lock;
    size_t start = (s_head + kMaxSamples - s_n) % kMaxSamples;
    for (size_t k = 0; k < s_n; ++k) {
        const Sample &s = s_samples[(start + k) % kMaxSamples];
        if ((int32_t)(s.t_ms - since_ms) >= 0) out.push_back(s);
    }
}
