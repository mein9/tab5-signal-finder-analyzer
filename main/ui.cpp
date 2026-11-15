#include "ui.h"

#include <M5Unified.h>
#include <math.h>
#include <string.h>
#include <algorithm>
#include <vector>

#include "esp_timer.h"
#include "esp_log.h"
#include "driver/ppa.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <lgfx/v1/platforms/esp32p4/Panel_DSI.hpp>
#include <string.h>
#include "nvs.h"

#include "alerts.h"
#include "ble_scan.h"
#include "devices.h"
#include "saved.h"
#include "sdlog.h"
#include "wifi_scan.h"

namespace {

// ---------------------------------------------------------------- colours
constexpr uint32_t BG        = 0x0B0F14u;
constexpr uint32_t PANEL     = 0x161D26u;
constexpr uint32_t PANEL_HI  = 0x223040u;
constexpr uint32_t LINE      = 0x2C3A4Au;
constexpr uint32_t TEXT      = 0xE8EEF4u;
constexpr uint32_t MUTED     = 0x8A9AAAu;
constexpr uint32_t ACCENT    = 0x3AA0FFu;
constexpr uint32_t GOOD      = 0x3DDC84u;
constexpr uint32_t WARN      = 0xFFB020u;
constexpr uint32_t BAD       = 0xFF5A5Au;

M5Canvas canvas;
int W = 1280, H = 720;
bool P = false;            // portrait (tall) layout
int  Y0 = 0;               // extra header height in portrait; screens start this much lower
bool s_rotate_pending = false;
uint8_t s_canvas_rot = 3;  // rotation the canvas is currently laid out for

enum class Screen { List, Finder, Sweep, Glow, Rename, Tools, Channels };
Screen   s_screen = Screen::List;
Filter   s_filter = Filter::Findable;
bool     s_wifi_section = false;
int      s_page = 0;
uint8_t  s_volume = 55;                 // percent, 0 = off
bool     s_vol_open = false;            // volume drop-down showing
bool     s_vol_dirty = false;           // slider moved, not saved yet
bool     s_down = false;                // finger on the screen right now
int      s_cur_x = 0, s_cur_y = 0;      // where it is
bool     s_scan_on = true;
Screen   s_tools_back = Screen::List;   // where "Back" on the More screen goes
DeviceView s_target_view{};             // the device being tracked, as tapped
float    s_motion_s = 0;                // seconds the Tab5 has been moving
float    s_activity = 0;
bool     s_forget_confirm = false;
char     s_edit[24] = {0};              // nickname being typed
bool     s_shift = true;
uint8_t  s_rotation = 3;
std::vector<DeviceView> s_list;
std::vector<Sample>     s_samples;

// Touch for this frame.
bool s_tap = false;
int  s_tx = 0, s_ty = 0;

// Beeper.
uint32_t s_next_beep = 0;

// Sweep radar.
constexpr int kBins = 36;               // 10 degrees each
float    s_bin[kBins];
bool     s_bin_set[kBins];
float    s_heading = 0;                 // degrees, clockwise = turning right
float    s_bias[3] = {0, 0, 0};
bool     s_calibrating = false;
uint32_t s_cal_start = 0;
float    s_cal_sum[3] = {0, 0, 0};
int      s_cal_n = 0;
int64_t  s_last_imu_us = 0;

// ---------------------------------------------------------------- helpers
struct Rect {
    int x, y, w, h;
    bool hit(int px, int py) const { return px >= x && px < x + w && py >= y && py < y + h; }
};

bool tapped(const Rect &r) { return s_tap && r.hit(s_tx, s_ty); }

void text(const char *s, int x, int y, const lgfx::IFont *font, uint32_t color,
          textdatum_t datum = top_left, float size = 1.0f) {
    canvas.setFont(font);
    canvas.setTextSize(size);
    canvas.setTextColor(color);
    canvas.setTextDatum(datum);
    canvas.drawString(s, x, y);
}

// Draws text wrapped to `maxw` pixels. Returns the y below the last line.
int text_wrap(const char *s, int x, int y, int maxw, const lgfx::IFont *font, uint32_t color, int line_h) {
    canvas.setFont(font);
    canvas.setTextSize(1.0f);
    canvas.setTextColor(color);
    canvas.setTextDatum(top_left);
    char line[240] = "", trial[240];
    const char *p = s;
    while (*p) {
        const char *e = p;
        while (*e && *e != ' ') ++e;
        snprintf(trial, sizeof trial, "%s%s%.*s", line, line[0] ? " " : "", (int)(e - p), p);
        if (line[0] && canvas.textWidth(trial) > maxw) {
            canvas.drawString(line, x, y);
            y += line_h;
            snprintf(line, sizeof line, "%.*s", (int)(e - p), p);
        } else {
            memcpy(line, trial, sizeof line);
        }
        p = e;
        while (*p == ' ') ++p;
    }
    if (line[0]) { canvas.drawString(line, x, y); y += line_h; }
    return y;
}

void button(const Rect &r, const char *label, bool on = false) {
    canvas.fillRoundRect(r.x, r.y, r.w, r.h, 14, on ? ACCENT : PANEL_HI);
    canvas.drawRoundRect(r.x, r.y, r.w, r.h, 14, on ? ACCENT : LINE);
    text(label, r.x + r.w / 2, r.y + r.h / 2, &fonts::FreeSansBold12pt7b, on ? BG : TEXT,
         middle_center);
}

uint32_t blend(uint32_t a, uint32_t b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    auto ch = [&](int s) {
        int ca = (a >> s) & 0xFF, cb = (b >> s) & 0xFF;
        return (uint32_t)(ca + (cb - ca) * t) << s;
    };
    return ch(16) | ch(8) | ch(0);
}

// -100 dBm (cold, red) .. -40 dBm (hot, green)
uint32_t strength_color(float rssi) {
    float t = (rssi + 100.0f) / 60.0f;
    return t < 0.5f ? blend(BAD, WARN, t * 2) : blend(WARN, GOOD, (t - 0.5f) * 2);
}

void signal_bars(int x, int y, float rssi, int h = 40) {
    int lit = rssi > -55 ? 5 : rssi > -65 ? 4 : rssi > -75 ? 3 : rssi > -85 ? 2 : 1;
    uint32_t c = strength_color(rssi);
    for (int i = 0; i < 5; ++i) {
        int bh = h * (i + 1) / 5;
        canvas.fillRoundRect(x + i * 11, y + h - bh, 8, bh, 2, i < lit ? c : LINE);
    }
}

float median_rssi(const std::vector<Sample> &v, uint32_t from, uint32_t to) {
    std::vector<int> r;
    for (const auto &s : v)
        if ((int32_t)(s.t_ms - from) >= 0 && (int32_t)(to - s.t_ms) >= 0) r.push_back(s.rssi);
    if (r.empty()) return NAN;
    std::nth_element(r.begin(), r.begin() + r.size() / 2, r.end());
    return (float)r[r.size() / 2];
}

const char *proximity_text(float rssi) {
    if (rssi >= -50) return "Very close - within about 1 m";
    if (rssi >= -62) return "Close - a few metres";
    if (rssi >= -75) return "Nearby - same room or next one";
    if (rssi >= -88) return "Weak - keep looking";
    return "Very weak - at the edge of range";
}

void apply_volume() {
    M5.Speaker.setVolume((uint8_t)(s_volume * 255 / 100));
}

void save_settings() {
    nvs_handle_t h;
    if (nvs_open("blefind", NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, "rot", s_rotation);
    nvs_set_u8(h, "volp", s_volume);
    nvs_commit(h);
    nvs_close(h);
}

void load_settings() {
    nvs_handle_t h;
    if (nvs_open("blefind", NVS_READONLY, &h) != ESP_OK) return;
    uint8_t v;
    if (nvs_get_u8(h, "rot", &v) == ESP_OK && v <= 3) s_rotation = v;
    if (nvs_get_u8(h, "volp", &v) == ESP_OK && v <= 100) s_volume = v;
    else if (nvs_get_u8(h, "vol", &v) == ESP_OK && v <= 3) {              // older 4-step setting
        static const uint8_t kStep[] = {0, 25, 55, 100};
        s_volume = kStep[v];
    } else if (nvs_get_u8(h, "snd", &v) == ESP_OK && !v) s_volume = 0;    // oldest on/off setting
    nvs_close(h);
}

// The panel is natively portrait (720x1280) but the app is landscape, and its
// frame buffer lives in slow external RAM. Letting the graphics library rotate
// each frame in software took ~650 ms. Instead:
//  - frames are drawn the way the user holds the Tab5 into one of two buffers,
//  - a background task hands the finished buffer to the P4's PPA hardware,
//    which rotates it (0/90/180/270 degrees) straight into the display's
//    frame buffer and then clears it to the background colour for reuse,
//  - meanwhile the next frame is already being drawn into the other buffer.
// If the hardware path is not available, fall back to the slow software path.
int s_panel_w = 720, s_panel_h = 1280;
ppa_client_handle_t s_ppa = nullptr, s_ppa_fill = nullptr;
uint16_t *s_fb = nullptr;
bool s_hw_push = false;
void *s_buf[2] = {nullptr, nullptr};
uint8_t s_cur = 0;

// A finished frame on its way to the screen.
struct Frame { uint8_t idx, rot; uint16_t w, h; };
QueueHandle_t s_q_present = nullptr, s_q_free = nullptr;

bool hw_rotate(void *src, int w, int h, uint8_t rot) {
    ppa_srm_oper_config_t op;
    memset(&op, 0, sizeof op);
    op.in.buffer = src;
    op.in.pic_w = w;
    op.in.pic_h = h;
    op.in.block_w = w;
    op.in.block_h = h;
    op.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    op.out.buffer = s_fb;
    op.out.buffer_size = (uint32_t)s_panel_w * s_panel_h * 2;
    op.out.pic_w = s_panel_w;
    op.out.pic_h = s_panel_h;
    op.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    // Display rotation 1 turns the picture clockwise onto the panel, 3
    // counter-clockwise; 0 and 2 are the panel's own portrait, upright and flipped.
    static const ppa_srm_rotation_angle_t kAngle[] = {PPA_SRM_ROTATION_ANGLE_0, PPA_SRM_ROTATION_ANGLE_270,
                                                      PPA_SRM_ROTATION_ANGLE_180, PPA_SRM_ROTATION_ANGLE_90};
    op.rotation_angle = kAngle[rot & 3];
    op.scale_x = 1.0f;
    op.scale_y = 1.0f;
    op.mode = PPA_TRANS_MODE_BLOCKING;
    esp_err_t err = ppa_do_scale_rotate_mirror(s_ppa, &op);
    if (err != ESP_OK) ESP_LOGE("finder_ui", "PPA rotate failed: %s", esp_err_to_name(err));
    return err == ESP_OK;
}

bool hw_clear(void *dst) {
    ppa_fill_oper_config_t op;
    memset(&op, 0, sizeof op);
    op.out.buffer = dst;
    op.out.buffer_size = (uint32_t)W * H * 2;
    op.out.pic_w = W;
    op.out.pic_h = H;
    op.out.fill_cm = PPA_FILL_COLOR_MODE_RGB565;
    op.fill_block_w = W;
    op.fill_block_h = H;
    op.fill_argb_color.val = 0xFF000000u | BG;
    op.mode = PPA_TRANS_MODE_BLOCKING;
    esp_err_t err = ppa_do_fill(s_ppa_fill, &op);
    if (err != ESP_OK) ESP_LOGE("finder_ui", "PPA clear failed: %s", esp_err_to_name(err));
    return err == ESP_OK;
}

void presenter_task(void *) {
    Frame f;
    for (;;) {
        xQueueReceive(s_q_present, &f, portMAX_DELAY);
        hw_rotate(s_buf[f.idx], f.w, f.h, f.rot);
        if (!hw_clear(s_buf[f.idx])) memset(s_buf[f.idx], 0, (size_t)f.w * f.h * 2);
        xQueueSend(s_q_free, &f.idx, portMAX_DELAY);
    }
}

// Draws two marker pixels, rotates them in hardware, and checks they land
// where the display's own rotation would have put them.
bool check_hw_push() {
    canvas.fillScreen(0);
    canvas.drawPixel(0, 0, (uint16_t)0xF800);
    canvas.drawPixel(W - 1, 0, (uint16_t)0x07E0);
    if (!hw_rotate(s_buf[0], W, H, s_rotation)) return false;
    auto at = [](int x, int y) -> uint16_t {
        int px, py;
        switch (s_rotation) {
        case 0:  px = x; py = y; break;
        case 1:  px = s_panel_w - 1 - y; py = x; break;
        case 2:  px = s_panel_w - 1 - x; py = s_panel_h - 1 - y; break;
        default: px = y; py = s_panel_h - 1 - x; break;
        }
        return s_fb[py * s_panel_w + px];
    };
    uint16_t a = at(0, 0), b = at(W - 1, 0);
    bool ok = a == 0xF800 && b == 0x07E0;
    ok = ok && hw_clear(s_buf[0]) && hw_clear(s_buf[1]);
    ESP_LOGI("finder_ui", "hardware screen path: %s (%04x %04x, clear %04x)", ok ? "ok" : "NOT OK",
             a, b, ((uint16_t *)s_buf[0])[0]);
    return ok;
}

void make_canvas() {
    M5.Display.setRotation(0);
    s_panel_w = M5.Display.width();
    s_panel_h = M5.Display.height();
    M5.Display.setRotation(s_rotation);   // also sets touch coordinates
    W = M5.Display.width();
    H = M5.Display.height();
    P = H > W;
    Y0 = P ? 72 : 0;
    s_canvas_rot = s_rotation;
    if (s_hw_push) {                      // only the rotation changed; buffers stay
        canvas.setBuffer(s_buf[s_cur], W, H);
        return;
    }

    canvas.setColorDepth(lgfx::color_depth_t::rgb565_nonswapped);   // the panel's own format
    if (!s_ppa) {
        auto *panel = static_cast<lgfx::Panel_DSI *>(M5.Display.getPanel());
        s_fb = panel ? (uint16_t *)panel->config_detail().buffer : nullptr;
        size_t len = (size_t)W * H * 2;
        for (auto &b : s_buf) b = heap_caps_aligned_calloc(128, 1, len, MALLOC_CAP_SPIRAM);
        ppa_client_config_t cc;
        memset(&cc, 0, sizeof cc);
        cc.max_pending_trans_num = 1;
        cc.oper_type = PPA_OPERATION_SRM;
        bool ok = s_fb && s_buf[0] && s_buf[1] && ppa_register_client(&cc, &s_ppa) == ESP_OK;
        cc.oper_type = PPA_OPERATION_FILL;
        ok = ok && ppa_register_client(&cc, &s_ppa_fill) == ESP_OK;
        if (ok) {
            canvas.setBuffer(s_buf[0], W, H);
            s_hw_push = check_hw_push();
        }
    }
    if (s_hw_push) {
        s_q_present = xQueueCreate(1, sizeof(Frame));
        s_q_free = xQueueCreate(2, 1);
        uint8_t one = 1;
        xQueueSend(s_q_free, &one, 0);
        s_cur = 0;
        xTaskCreatePinnedToCore(presenter_task, "present", 4096, nullptr, 6, nullptr, 0);
    } else {
        ESP_LOGW("finder_ui", "using slow software screen updates");
        canvas.deleteSprite();
        canvas.setPsram(true);
        canvas.createSprite(W, H);
    }
    ESP_LOGI("finder_ui", "screen %dx%d, panel %dx%d", W, H, s_panel_w, s_panel_h);
}

// Starts a frame: the canvas is cleared and ready to draw on.
void begin_frame() {
    if (!s_hw_push) canvas.fillScreen(BG);
}

// Shows the frame just drawn.
void push_frame() {
    if (!s_hw_push) { canvas.pushSprite(&M5.Display, 0, 0); return; }
    Frame f{s_cur, s_canvas_rot, (uint16_t)W, (uint16_t)H};
    xQueueSend(s_q_present, &f, portMAX_DELAY);
    xQueueReceive(s_q_free, &s_cur, portMAX_DELAY);   // the other buffer, shown and cleared
    canvas.setBuffer(s_buf[s_cur], W, H);
}

// Touch is checked more than once a frame so a quick tap is never missed.
bool s_tap_latched = false;
int  s_latch_x = 0, s_latch_y = 0;
void poll_touch() {
    M5.update();
    auto td = M5.Touch.getDetail();
    if (td.wasPressed()) { s_tap_latched = true; s_latch_x = td.x; s_latch_y = td.y; }
    s_down = td.isPressed();
    if (s_down) { s_cur_x = td.x; s_cur_y = td.y; }
}

// ---------------------------------------------------------------- header
// Landscape: buttons sit at the right of the title row. Portrait: on a second row.
void header_buttons(Rect &scan, Rect &vol, Rect &more) {
    if (P) {
        scan = Rect{24, 84, 216, 52};
        vol = Rect{252, 84, 216, 52};
        more = Rect{480, 84, 216, 52};
    } else {
        scan = Rect{W - 506, 12, 170, 52};
        vol = Rect{W - 326, 12, 170, 52};
        more = Rect{W - 146, 12, 126, 52};
    }
}

void draw_header(const char *title) {
    canvas.fillRect(0, 0, W, 76 + Y0, PANEL);
    canvas.drawFastHLine(0, 76 + Y0, W, LINE);
    text(title, 24, 38, &fonts::FreeSansBold12pt7b, TEXT, middle_left);

    BleState st = ble_state();
    uint32_t dot = st == BleState::Scanning ? GOOD
                 : (st == BleState::LinkFailed || st == BleState::ControllerFailed ||
                    st == BleState::ScanFailed) ? BAD : WARN;
    char status[96];
    if (st == BleState::Scanning)
        snprintf(status, sizeof status, "Scanning  |  %u devices", (unsigned)devices_count(15000));
    else if (st == BleState::Paused)
        snprintf(status, sizeof status, "Scanning is off");
    else if (ble_error())
        snprintf(status, sizeof status, "%s (%d)", ble_state_text(), ble_error());
    else
        snprintf(status, sizeof status, "%s...", ble_state_text());
    int sx = P ? 330 : 372;
    canvas.fillCircle(sx, 38, 9, dot);
    text(status, sx + 20, 38, &fonts::FreeSans12pt7b, MUTED, middle_left);

    static int bat = -1;
    static uint32_t bat_t = 0;
    if (bat_t == 0 || now_ms() - bat_t > 5000) { bat = M5.Power.getBatteryLevel(); bat_t = now_ms() | 1; }
    if (bat >= 0) {
        char b[16];
        snprintf(b, sizeof b, "%d%%", bat);
        text(b, P ? W - 24 : W - 524, 38, &fonts::FreeSans12pt7b, MUTED, middle_right);
    }

    Rect scan, vol, more;
    header_buttons(scan, vol, more);
    button(scan, s_scan_on ? "Scan: on" : "Scan: off", s_scan_on);
    char vl[24];
    if (s_volume) snprintf(vl, sizeof vl, "Sound %d%%", (int)s_volume);
    else snprintf(vl, sizeof vl, "Sound off");
    button(vol, vl, s_vol_open || s_volume > 0);
    button(more, "More", s_screen == Screen::Tools);
    if (tapped(scan)) {
        s_scan_on = !s_scan_on;
        ble_set_scanning(s_scan_on);
        s_tap = false;
    }
    if (tapped(vol)) { s_vol_open = !s_vol_open; s_tap = false; }
    if (tapped(more)) {
        if (s_screen == Screen::Tools) s_screen = s_tools_back;
        else { s_tools_back = s_screen; s_screen = Screen::Tools; s_forget_confirm = false; }
        s_tap = false;
    }
}

// ---------------------------------------------------------------- radio update
bool s_confirm = false;

void radio_update_panel() {
    int x = P ? 24 : 60, y = 110 + Y0, w = W - 2 * x;
    RadioUpdate st = radio_update_state();
    canvas.fillRoundRect(x, y, w, H - y - 40, 18, PANEL);
    int tx = x + 40, ty = y + 40;

    if (st == RadioUpdate::Idle) {
        int yy = text_wrap("Bluetooth is switched off in the radio chip", tx, ty, w - 80,
                           &fonts::FreeSansBold18pt7b, WARN, 44);
        yy = text_wrap("The Tab5's ESP32-C6 radio chip is running a Wi-Fi-only firmware, so it can't scan "
                       "for Bluetooth devices.", tx, yy + 20, w - 80, &fonts::FreeSans12pt7b, TEXT, 34);
        text_wrap("This app can install the same ESP-Hosted version (3.0.6) with Wi-Fi AND Bluetooth. It takes "
                  "about a minute. The new firmware is written to a spare slot and only used once it arrives "
                  "complete; if the transfer fails, the radio chip keeps its current firmware. Don't turn the "
                  "Tab5 off while it runs. Other apps may switch it back to their own version later.",
                  tx, yy + 16, w - 80, &fonts::FreeSans12pt7b, MUTED, 34);
        Rect go{tx, H - 170, 520, 80};
        button(go, s_confirm ? "Tap again to confirm" : "Install Wi-Fi + Bluetooth firmware", s_confirm);
        if (tapped(go)) {
            if (s_confirm) radio_update_start();
            else s_confirm = true;
            s_tap = false;
        }
        if (s_confirm) {
            Rect no = P ? Rect{tx, H - 270, 220, 80} : Rect{tx + 560, H - 170, 220, 80};
            button(no, "Cancel");
            if (tapped(no)) { s_confirm = false; s_tap = false; }
        }
        return;
    }

    const char *title = st == RadioUpdate::Failed ? "Update didn't complete"
                      : st == RadioUpdate::Done   ? "Radio chip updated"
                                                  : "Updating the radio chip";
    text(title, tx, ty, &fonts::FreeSansBold18pt7b,
         st == RadioUpdate::Failed ? BAD : st == RadioUpdate::Done ? GOOD : TEXT);
    text(radio_update_message(), tx, ty + 70, &fonts::FreeSans12pt7b, TEXT);
    int bx = tx, by = ty + 130, bw = w - 80, bh = 50;
    canvas.fillRoundRect(bx, by, bw, bh, 12, BG);
    canvas.fillRoundRect(bx, by, std::max(24, bw * radio_update_percent() / 100), bh, 12,
                         st == RadioUpdate::Failed ? BAD : ACCENT);
    char pct[16];
    snprintf(pct, sizeof pct, "%d%%", radio_update_percent());
    text(pct, bx + bw / 2, by + bh / 2, &fonts::FreeSansBold12pt7b, TEXT, middle_center);
    if (st == RadioUpdate::Running)
        text("Keep the Tab5 switched on.", tx, by + 90, &fonts::FreeSansBold12pt7b, WARN);
    if (st == RadioUpdate::Done)
        text("Restarting the app to start Bluetooth...", tx, by + 90, &fonts::FreeSans12pt7b, MUTED);
}

// ---------------------------------------------------------------- list
// Starts tracking a device and opens the hot/cold screen.
void open_finder(const DeviceView &d) {
    s_target_view = d;
    DeviceView shown = d;
    Saved *sv = saved_find(d);
    if (sv && sv->name[0]) snprintf(shown.label, sizeof shown.label, "%s", sv->name);
    // AirPods and Find My items near their owner keep changing address, so
    // those are followed by type. Everything else is followed by address.
    bool by_type = d.kind == Kind::AirPods || (d.kind == Kind::FindMy && !d.away);
    target_set(shown, by_type);
    s_screen = Screen::Finder;
    s_next_beep = 0;
}

void screen_list() {
    draw_header("Signal Finder & Analyzer");
    if (ble_state() == BleState::ControllerFailed || radio_update_state() != RadioUpdate::Idle) {
        radio_update_panel();
        return;
    }

    // Section switch: Bluetooth | Wi-Fi
    Rect bt{24, 92 + Y0, 200, 56}, wf{232, 92 + Y0, 200, 56};
    button(bt, "Bluetooth", !s_wifi_section);
    button(wf, "Wi-Fi", s_wifi_section);
    if (tapped(bt)) { s_wifi_section = false; s_page = 0; s_tap = false; }
    if (tapped(wf)) { s_wifi_section = true; s_page = 0; s_tap = false; }
    if (!P) canvas.drawFastVLine(452, 96, 48, LINE);

    if (!s_wifi_section) {
        const char *names[] = {P ? "Trackers" : "Trackers & earbuds", "Named", "Everything"};
        const Filter vals[] = {Filter::Findable, Filter::Named, Filter::All};
        for (int i = 0; i < 3; ++i) {
            Rect r = P ? Rect{24 + i * 228, 156 + Y0, 216, 56} : Rect{472 + i * 264, 92, 252, 56};
            button(r, names[i], s_filter == vals[i]);
            if (tapped(r)) { s_filter = vals[i]; s_page = 0; s_tap = false; }
        }
    } else {
        char ws[96];
        if (wifi_state() == WifiState::Scanning)
            snprintf(ws, sizeof ws, "Networks nearby, strongest first  |  %lu scans",
                     (unsigned long)wifi_scans_done());
        else if (wifi_state() == WifiState::Failed)
            snprintf(ws, sizeof ws, "Wi-Fi didn't start on the radio chip (error %d)", wifi_error());
        else
            snprintf(ws, sizeof ws, "%s...", wifi_state_text());
        text(ws, P ? 24 : 476, P ? 184 + Y0 : 120, &fonts::FreeSans12pt7b,
             wifi_state() == WifiState::Failed ? BAD : MUTED, middle_left);
    }

    devices_snapshot(s_list, s_wifi_section ? Filter::Wifi : s_filter,
                     s_wifi_section ? 30000 : 15000);
    // Pinned devices first, each group still strongest first.
    std::stable_partition(s_list.begin(), s_list.end(), [](const DeviceView &d) {
        Saved *sv = saved_find(d);
        return sv && (sv->flags & kSavedPin);
    });

    const int top = 164 + Y0 + (P ? 64 : 0), row_h = 80, foot = 80;
    const int rows = std::max(1, (H - top - foot) / row_h);
    const int pages = std::max(1, (int)((s_list.size() + rows - 1) / rows));
    s_page = std::min(s_page, pages - 1);

    if (s_list.empty()) {
        const char *msg;
        if (!s_scan_on) msg = "Scanning is off. Tap Scan to start.";
        else if (s_wifi_section)
            msg = wifi_state() == WifiState::Failed ? "Wi-Fi scanning isn't available." : "Looking for Wi-Fi networks...";
        else if (ble_state() != BleState::Scanning) msg = "Waiting for Bluetooth to start...";
        else if (s_filter == Filter::Findable)
            msg = "No trackers or earbuds heard yet. Open the AirPods case, or try Everything.";
        else msg = "Nothing heard yet.";
        text_wrap(msg, 48, top + 100, W - 96, &fonts::FreeSans12pt7b, MUTED, 34);
    }

    for (int i = 0; i < rows; ++i) {
        size_t idx = (size_t)(s_page * rows + i);
        if (idx >= s_list.size()) break;
        const DeviceView &d = s_list[idx];
        Rect r{24, top + i * row_h, W - 48, row_h - 8};
        canvas.fillRoundRect(r.x, r.y, r.w, r.h, 12, PANEL);
        signal_bars(r.x + 20, r.y + 16, d.rssi);
        Saved *sv = saved_find(d);
        const char *label = sv && sv->name[0] ? sv->name : d.label;
        canvas.setClipRect(r.x + 100, r.y, r.w - 100 - 150, r.h);   // keep long text off the reading
        text(label, r.x + 100, r.y + 10, &fonts::FreeSansBold12pt7b, TEXT);
        int tagx = r.x + 100 + canvas.textWidth(label, &fonts::FreeSansBold12pt7b) + 18;
        if (sv && (sv->flags & kSavedPin)) {
            text("PINNED", tagx, r.y + 16, &fonts::FreeSans9pt7b, ACCENT);
            tagx += 92;
        }
        if (sv && (sv->flags & kSavedWatch)) text("ALARM SET", tagx, r.y + 16, &fonts::FreeSans9pt7b, WARN);
        char det[160];
        if (sv && sv->name[0]) snprintf(det, sizeof det, "%s  |  %s", d.label, d.detail);
        else snprintf(det, sizeof det, "%s", d.detail);
        text(det, r.x + 100, r.y + 42, &fonts::FreeSans9pt7b, MUTED);
        canvas.clearClipRect();
        char rs[32];
        snprintf(rs, sizeof rs, "%.0f dBm", d.rssi);
        text(rs, r.x + r.w - 24, r.y + 12, &fonts::FreeSansBold12pt7b, strength_color(d.rssi),
             top_right);
        char ago[32];
        if (d.age_ms < 2000) snprintf(ago, sizeof ago, "now");
        else snprintf(ago, sizeof ago, "%lus ago", (unsigned long)(d.age_ms / 1000));
        text(ago, r.x + r.w - 24, r.y + 44, &fonts::FreeSans9pt7b, MUTED, top_right);

        if (tapped(r)) {
            s_tap = false;
        }
    }

    Rect prev{24, H - 70, 200, 56}, next{W - 224, H - 70, 200, 56};
    char pg[96];
    snprintf(pg, sizeof pg, P ? "Page %d of %d" : "Page %d of %d  |  tap a device to find it", s_page + 1, pages);
    text(pg, W / 2, H - 42, &fonts::FreeSans12pt7b, MUTED, middle_center);
    if (s_page > 0) { button(prev, "< Prev"); if (tapped(prev)) { --s_page; s_tap = false; } }
    if (s_page < pages - 1) { button(next, "Next >"); if (tapped(next)) { ++s_page; s_tap = false; } }
}

// ---------------------------------------------------------------- shared finder bits
struct Reading {
    float now = NAN;      // median of the last 1.5 s
    float before = NAN;   // median of 6 s .. 3 s ago
    uint32_t last_ms = 0;
};

Reading read_target(uint32_t t) {
    Reading r;
    target_samples(s_samples, t - 30000);
    r.now = median_rssi(s_samples, t - 1500, t);
    r.before = median_rssi(s_samples, t - 6000, t - 3000);
    if (!s_samples.empty()) r.last_ms = s_samples.back().t_ms;
    return r;
}

void beeper(const Reading &r, uint32_t t) {
    if (!s_volume || isnan(r.now)) return;
    if ((int32_t)(t - s_next_beep) < 0) return;
    float k = std::clamp((r.now + 95.0f) / 55.0f, 0.0f, 1.0f);      // 0 far .. 1 close
    uint32_t interval = (uint32_t)(1400 - k * 1310);                // 1400 ms .. 90 ms
    float freq = 700 + k * 1500;
    M5.Speaker.tone(freq, 35);
    s_next_beep = t + interval;
}

void draw_target_title(const Target &tg, const char *suffix) {
    Rect back{24, 92 + Y0, 150, 56};
    button(back, "< Back");
    if (tapped(back)) {
        if (s_screen == Screen::Sweep) s_screen = Screen::Finder;
        else { s_screen = Screen::List; target_clear(); }
        s_tap = false;
    }
    char t[96];
    snprintf(t, sizeof t, "%s%s", tg.label, suffix);
    canvas.setClipRect(196, 92 + Y0, W - 196 - 16, 70);
    text(t, 196, 100 + Y0, &fonts::FreeSansBold18pt7b, TEXT);
    char sub[96];
    if (tg.wifi_channel)
        snprintf(sub, sizeof sub, "Wi-Fi network on channel %u - updates a few times a second",
                 (unsigned)tg.wifi_channel);
    else
        snprintf(sub, sizeof sub, "%s",
                 tg.by_sig ? "Following this type of device - its address keeps changing"
                           : "Following this exact device");
    text(sub, 196, 136 + Y0, &fonts::FreeSans9pt7b, MUTED);
    canvas.clearClipRect();
}

// ---------------------------------------------------------------- finder
void screen_finder() {
    draw_header("Signal Finder & Analyzer");
    Target tg = target_get();
    uint32_t t = now_ms();
    Reading r = read_target(t);
    draw_target_title(tg, "");

    int y0 = 176 + Y0;
    bool lost = isnan(r.now);
    uint32_t since = r.last_ms ? (t - r.last_ms) / 1000 : 0;

    // Big number + description.
    char big[24];
    if (lost) snprintf(big, sizeof big, "--");
    else snprintf(big, sizeof big, "%.0f", r.now);
    uint32_t col = lost ? MUTED : strength_color(r.now);
    text(big, 60, y0, &fonts::FreeSansBold24pt7b, col, top_left, 3.0f);
    canvas.setTextSize(3.0f);
    int big_w = canvas.textWidth(big, &fonts::FreeSansBold24pt7b);
    text("dBm", 60 + big_w + 16, y0 + 20, &fonts::FreeSans18pt7b, MUTED);

    const char *line1, *line2 = "";
    char lostmsg[96];
    if (lost) {
        line1 = r.last_ms ? "Signal lost" : "Listening...";
        if (r.last_ms) {
            snprintf(lostmsg, sizeof lostmsg, "Last heard %lus ago. Walk back toward where it was stronger.",
                     (unsigned long)since);
            line2 = lostmsg;
        } else {
            line2 = "Nothing from this device yet. AirPods only broadcast now and then.";
        }
    } else {
        line1 = proximity_text(r.now);
        if (!isnan(r.before)) {
            float d = r.now - r.before;
            line2 = d >= 3 ? "Getting warmer" : d <= -3 ? "Getting colder" : "About the same";
        }
    }
    // The description sits beside the number in landscape, under it in portrait.
    int tx = P ? 60 : W / 2 + 20, ty = P ? y0 + 170 : y0;
    text(line1, tx, ty + 14, &fonts::FreeSansBold18pt7b, TEXT);
    if (!lost && !isnan(r.before)) {
        float d = r.now - r.before;
        uint32_t c = d >= 3 ? GOOD : d <= -3 ? BAD : MUTED;
        int ax = tx + 14, ay = ty + 92;
        if (d >= 3) canvas.fillTriangle(ax - 14, ay + 10, ax + 14, ay + 10, ax, ay - 14, c);
        else if (d <= -3) canvas.fillTriangle(ax - 14, ay - 10, ax + 14, ay - 10, ax, ay + 14, c);
        else canvas.fillRect(ax - 14, ay - 4, 28, 8, c);
        text(line2, tx + 44, ay, &fonts::FreeSansBold12pt7b, c, middle_left);
    } else {
        text_wrap(line2, tx, ty + 66, W - tx - 40, &fonts::FreeSans12pt7b, MUTED, 30);
    }

    // Meter: -100 .. -30 dBm.
    int my = P ? ty + 150 : y0 + 170, mx = 60, mw = W - 120, mh = 46;
    canvas.fillRoundRect(mx, my, mw, mh, 12, PANEL);
    if (!lost) {
        float k = std::clamp((r.now + 100.0f) / 70.0f, 0.0f, 1.0f);
        int fw = std::max(24, (int)(mw * k));
        for (int x = 0; x < fw; x += 4)
            canvas.fillRect(mx + x, my, std::min(4, fw - x), mh,
                            strength_color(-100.0f + 70.0f * x / mw));
    }
    canvas.drawRoundRect(mx, my, mw, mh, 12, LINE);
    text("cold", mx, my + mh + 8, &fonts::FreeSans9pt7b, MUTED);
    text("hot", mx + mw, my + mh + 8, &fonts::FreeSans9pt7b, MUTED, top_right);

    // Graph of the last 30 s.
    int gy = my + mh + 44, gh = H - gy - (P ? 172 : 96), gx = mx, gw = mw;
    canvas.fillRoundRect(gx, gy, gw, gh, 12, PANEL);
    for (int db = -90; db <= -40; db += 10) {
        int yy = gy + gh - (int)((db + 100) * gh / 70.0f);
        canvas.drawFastHLine(gx + 8, yy, gw - 16, LINE);
        char l[8];
        snprintf(l, sizeof l, "%d", db);
        text(l, gx + 12, yy - 2, &fonts::Font0, MUTED, bottom_left, 1.5f);
    }
    for (const auto &s : s_samples) {
        float age = (t - s.t_ms) / 30000.0f;
        if (age > 1) continue;
        int xx = gx + gw - 12 - (int)(age * (gw - 24));
        int yy = gy + gh - (int)((std::clamp((float)s.rssi, -100.0f, -30.0f) + 100) * gh / 70.0f);
        canvas.fillCircle(xx, yy, 3, strength_color(s.rssi));
    }
    text("last 30 seconds", gx + gw - 12, gy + 8, &fonts::FreeSans9pt7b, MUTED, top_right);

    // Actions along the bottom.
    Saved *sv = saved_find(s_target_view);
    bool pinned = sv && (sv->flags & kSavedPin);
    bool watched = sv && (sv->flags & kSavedWatch);
    bool is_wifi = tg.wifi_channel != 0;
    // One row of five in landscape; three over two in portrait.
    const int bw = P ? 216 : 236, bgap = P ? 12 : 13, by = H - 76;
    auto slot = [&](int i) {
        return P ? Rect{24 + (i % 3) * (bw + bgap), i < 3 ? by - 74 : by, bw, 60}
                 : Rect{24 + i * (bw + bgap), by, bw, 60};
    };
    Rect b_pin = slot(0), b_name = slot(1), b_alarm = slot(2), b_glow = slot(3), sweep = slot(4);
    button(b_pin, pinned ? "Unpin" : "Pin to top", pinned);
    button(b_name, "Rename");
    button(b_glow, "Color view");
    button(sweep, "Sweep radar");
    if (is_wifi) {
        canvas.drawRoundRect(b_alarm.x, b_alarm.y, b_alarm.w, b_alarm.h, 14, LINE);
        text("Alarm: Bluetooth only", b_alarm.x + bw / 2, b_alarm.y + 30, &fonts::FreeSans9pt7b, MUTED, middle_center);
    } else {
        button(b_alarm, watched ? "Alarm: on" : "Alarm if lost", watched);
    }
    if (tapped(b_pin) || (!is_wifi && tapped(b_alarm))) {
        uint8_t bit = tapped(b_pin) ? kSavedPin : kSavedWatch;
        if (Saved *e = saved_get(s_target_view)) {
            e->flags ^= bit;
            saved_store();
        }
        s_tap = false;
    }
    if (tapped(b_name)) {
        snprintf(s_edit, sizeof s_edit, "%s", sv ? sv->name : "");
        s_shift = s_edit[0] == 0;
        s_screen = Screen::Rename;
        s_tap = false;
    }
    if (tapped(b_glow)) { s_screen = Screen::Glow; s_tap = false; }
    if (tapped(sweep)) {
        s_screen = Screen::Sweep;
        for (int i = 0; i < kBins; ++i) { s_bin[i] = -127; s_bin_set[i] = false; }
        s_heading = 0;
        s_calibrating = true;
        s_cal_start = t;
        s_cal_n = 0;
        s_cal_sum[0] = s_cal_sum[1] = s_cal_sum[2] = 0;
        s_tap = false;
    }

    beeper(r, t);
}

// ---------------------------------------------------------------- sweep radar
void update_heading() {
    int64_t us = esp_timer_get_time();
    float dt = s_last_imu_us ? (us - s_last_imu_us) / 1e6f : 0;
    s_last_imu_us = us;
    if (!M5.Imu.isEnabled()) return;
    M5.Imu.update();
    float gx, gy, gz, ax, ay, az;
    if (!M5.Imu.getGyro(&gx, &gy, &gz) || !M5.Imu.getAccel(&ax, &ay, &az)) return;

    // Is the Tab5 being carried around? Shaking of ~0.1 g or turning of
    // ~40 deg/s counts. The total feeds the tracker-following alert.
    {
        float g = sqrtf(ax * ax + ay * ay + az * az);
        float turn = sqrtf((gx - s_bias[0]) * (gx - s_bias[0]) + (gy - s_bias[1]) * (gy - s_bias[1]) +
                           (gz - s_bias[2]) * (gz - s_bias[2]));
        float act = fabsf(g - 1.0f) * 10.0f + turn / 40.0f;
        s_activity += (act - s_activity) * 0.1f;
        if (s_activity > 0.6f && dt > 0 && dt < 0.25f) {
            uint32_t before = (uint32_t)s_motion_s;
            s_motion_s += dt;
            if ((uint32_t)s_motion_s != before) devices_set_motion_s((uint32_t)s_motion_s);
        }
    }

    if (s_calibrating) {
        s_cal_sum[0] += gx; s_cal_sum[1] += gy; s_cal_sum[2] += gz; ++s_cal_n;
        if (now_ms() - s_cal_start > 1200 && s_cal_n > 10) {
            for (int i = 0; i < 3; ++i) s_bias[i] = s_cal_sum[i] / s_cal_n;
            s_calibrating = false;
            s_heading = 0;
        }
        return;
    }
    gx -= s_bias[0]; gy -= s_bias[1]; gz -= s_bias[2];
    float an = sqrtf(ax * ax + ay * ay + az * az);
    if (an < 0.3f || dt <= 0 || dt > 0.25f) return;
    // Accelerometer points "up"; spin about that axis is turning left/right.
    float yaw_rate = (gx * ax + gy * ay + gz * az) / an;   // deg/s, + = turning left
    s_heading -= yaw_rate * dt;                              // + = turning right
    while (s_heading < 0) s_heading += 360;
    while (s_heading >= 360) s_heading -= 360;
}

void screen_sweep() {
    draw_header("Signal Finder & Analyzer");
    Target tg = target_get();
    uint32_t t = now_ms();
    Reading r = read_target(t);
    draw_target_title(tg, P ? "" : "  -  sweep radar");

    if (!M5.Imu.isEnabled()) {
        text("The motion sensor isn't available, so the sweep can't track direction.",
             W / 2, H / 2, &fonts::FreeSansBold12pt7b, BAD, middle_center);
        beeper(r, t);
        return;
    }

    // Record the signal into the bin we're facing.
    float cur = median_rssi(s_samples, t - 900, t);
    if (!s_calibrating && !isnan(cur)) {
        int b = ((int)(s_heading / (360.0f / kBins))) % kBins;
        s_bin[b] = s_bin_set[b] ? s_bin[b] * 0.6f + cur * 0.4f : cur;
        s_bin_set[b] = true;
    }

    float lo = 0, hi = -200;
    int best = -1, filled = 0;
    for (int i = 0; i < kBins; ++i) {
        if (!s_bin_set[i]) continue;
        ++filled;
        if (s_bin[i] > hi) { hi = s_bin[i]; best = i; }
        if (filled == 1 || s_bin[i] < lo) lo = s_bin[i];
    }

    // Dial on the left with text beside it in landscape; dial on top, text under it in portrait.
    int R = P ? 250 : std::min(300, (H - 220) / 2);
    int cx = P ? W / 2 : 420, cy = P ? 250 + Y0 + R : 180 + (H - 180) / 2 - 10;
    canvas.fillCircle(cx, cy, R + 6, PANEL);
    // Wedges, drawn so the direction you face is always at the top.
    for (int i = 0; i < kBins; ++i) {
        float a0 = i * 10.0f - s_heading, a1 = a0 + 10.0f;
        uint32_t c = LINE;
        float rr = R * 0.35f;
        if (s_bin_set[i]) {
            float k = hi > lo ? (s_bin[i] - lo) / (hi - lo) : 1.0f;
            c = strength_color(-95.0f + 55.0f * k);
            rr = R * (0.35f + 0.65f * k);
        }
        // fillArc angles: 0 = 3 o'clock, clockwise. Our 0 = 12 o'clock.
        canvas.fillArc(cx, cy, (int)rr, 40, a0 - 90, a1 - 90 - 0.8f, c);
    }
    for (int k = 1; k <= 3; ++k) canvas.drawCircle(cx, cy, R * k / 3, LINE);
    canvas.fillCircle(cx, cy, 34, BG);
    // "You are facing" marker.
    canvas.fillTriangle(cx - 22, cy - R - 26, cx + 22, cy - R - 26, cx, cy - R + 4, TEXT);
    text("facing", cx, cy - R - 32, &fonts::FreeSans9pt7b, MUTED, bottom_center);

    int px = P ? 40 : cx + R + 70;
    const int pb = P ? cy + R + 30 - 190 : 0;     // moves the text block down in portrait
    char line[96];
    if (s_calibrating) {
        text("Hold still...", px, 200 + pb, &fonts::FreeSansBold18pt7b, WARN);
        text("Measuring the motion sensor.", px, 250 + pb, &fonts::FreeSans12pt7b, MUTED);
    } else {
        text("Turn slowly in a full circle", px, 190 + pb, &fonts::FreeSansBold18pt7b, TEXT);
        text("Hold the Tab5 in front of your chest. Your body", px, 236 + pb, &fonts::FreeSans12pt7b, MUTED);
        text("blocks signal from behind, so it reads strongest", px, 266 + pb, &fonts::FreeSans12pt7b, MUTED);
        text("when you face the device.", px, 296 + pb, &fonts::FreeSans12pt7b, MUTED);

        snprintf(line, sizeof line, "Circle covered: %d%%", filled * 100 / kBins);
        text(line, px, 350 + pb, &fonts::FreeSansBold12pt7b, filled == kBins ? GOOD : TEXT);

        if (best >= 0 && filled >= 6) {
            float rel = best * 10.0f + 5.0f - s_heading;
            while (rel > 180) rel -= 360;
            while (rel < -180) rel += 360;
            if (fabsf(rel) <= 15) snprintf(line, sizeof line, "Strongest: straight ahead");
            else snprintf(line, sizeof line, "Strongest: %.0f deg to your %s", fabsf(rel),
                          rel > 0 ? "right" : "left");
            text(line, px, 400 + pb, &fonts::FreeSansBold18pt7b, GOOD);
            // Arrow from the centre toward the strongest bin.
            float a = (rel - 90.0f) * (float)M_PI / 180.0f;
            int ex = cx + (int)(cosf(a) * (R - 10)), ey = cy + (int)(sinf(a) * (R - 10));
            canvas.drawWideLine(cx, cy, ex, ey, 7, TEXT);
            canvas.fillCircle(ex, ey, 14, TEXT);
            snprintf(line, sizeof line, "best %.0f dBm, weakest %.0f dBm", hi, lo);
            text(line, px, 446 + pb, &fonts::FreeSans12pt7b, MUTED);
            if (hi - lo < 6)
                text("Small difference - try again more slowly, or move.", px, 476 + pb,
                     &fonts::FreeSans12pt7b, WARN);
        }
        snprintf(line, sizeof line, "Now: %s", isnan(cur) ? "no signal" : "");
        if (!isnan(cur)) snprintf(line, sizeof line, "Now: %.0f dBm", cur);
        text(line, px, 520 + pb, &fonts::FreeSansBold12pt7b, isnan(cur) ? MUTED : strength_color(cur));
    }

    Rect reset{P ? W - 264 : px, H - 90, 240, 60};
    button(reset, "Start over");
    if (tapped(reset)) {
        for (int i = 0; i < kBins; ++i) { s_bin[i] = -127; s_bin_set[i] = false; }
        s_calibrating = true;
        s_cal_start = t;
        s_cal_n = 0;
        s_cal_sum[0] = s_cal_sum[1] = s_cal_sum[2] = 0;
        s_tap = false;
    }
    beeper(r, t);
}

// ---------------------------------------------------------------- color view
// The whole screen is one colour, blue (far) to red (close), readable at
// arm's length. Silent, for quiet places.
void screen_glow() {
    Target tg = target_get();
    uint32_t t = now_ms();
    Reading r = read_target(t);
    bool lost = isnan(r.now);
    float k = lost ? 0 : std::clamp((r.now + 95.0f) / 55.0f, 0.0f, 1.0f);
    uint32_t col = lost ? 0x20262Cu : blend(0x1038B8u, 0xE02818u, k);
    canvas.fillScreen(col);
    text(tg.label, W / 2, 70, &fonts::FreeSansBold18pt7b, 0xFFFFFFu, middle_center);
    char big[24];
    if (lost) snprintf(big, sizeof big, "--");
    else snprintf(big, sizeof big, "%.0f", r.now);
    text(big, W / 2, H / 2 - 30, &fonts::FreeSansBold24pt7b, 0xFFFFFFu, middle_center, 4.0f);
    text(lost ? (r.last_ms ? "Signal lost" : "Listening...") : proximity_text(r.now), W / 2, H / 2 + 130,
         &fonts::FreeSansBold18pt7b, 0xFFFFFFu, middle_center);
    text("blue = far     red = close     tap anywhere to go back", W / 2, H - 50, &fonts::FreeSans12pt7b,
         0xFFFFFFu, middle_center);
    if (s_tap) { s_screen = Screen::Finder; s_tap = false; }
}

// ---------------------------------------------------------------- rename
void screen_rename() {
    text("Name this device", 24, 20, &fonts::FreeSansBold18pt7b, TEXT);
    char sub[160];
    bool sticks = s_target_view.kind == Kind::AirPods || s_target_view.stable_addr || s_target_view.away;
    snprintf(sub, sizeof sub, "%s%s", s_target_view.label,
             sticks ? "" : "   -   this device changes its address every few minutes, so the name only lasts until then");
    text_wrap(sub, 24, 66, W - 48, &fonts::FreeSans9pt7b, sticks ? MUTED : WARN, 22);

    Rect field{24, P ? 150 : 108, W - 48, 84};
    canvas.fillRoundRect(field.x, field.y, field.w, field.h, 14, PANEL);
    canvas.drawRoundRect(field.x, field.y, field.w, field.h, 14, ACCENT);
    text(s_edit[0] ? s_edit : "Type a name", field.x + 28, field.y + 42, &fonts::FreeSansBold18pt7b,
         s_edit[0] ? TEXT : MUTED, middle_left);
    if (s_edit[0] && (now_ms() / 500) % 2) {
        int cxp = field.x + 28 + canvas.textWidth(s_edit, &fonts::FreeSansBold18pt7b) + 6;
        canvas.fillRect(cxp, field.y + 20, 3, 44, ACCENT);
    }

    // Ten keys across, sized to the screen width.
    const int pitch = P ? 68 : 120, kw = pitch - 8, kh = 84, rowp = 92, x0 = (W - 10 * pitch + 8) / 2,
              y0 = field.y + 106;
    auto u = [&](int v) { return v * pitch / 120; };
    static const char *kRows[] = {"1234567890", "qwertyuiop", "asdfghjkl", "zxcvbnm"};
    const int kIndent[] = {0, 0, pitch / 2, pitch};
    size_t len = strlen(s_edit);
    for (int row = 0; row < 4; ++row) {
        for (int i = 0; kRows[row][i]; ++i) {
            char c = kRows[row][i];
            if (s_shift && c >= 'a' && c <= 'z') c = (char)(c - 32);
            Rect k{x0 + kIndent[row] + i * pitch, y0 + row * rowp, kw, kh};
            char lbl[2] = {c, 0};
            button(k, lbl);
            if (tapped(k)) {
                if (len < sizeof s_edit - 1) { s_edit[len++] = c; s_edit[len] = 0; }
                s_shift = false;
                s_tap = false;
            }
        }
    }
    Rect shift{x0, y0 + 3 * rowp, kw, kh}, del{x0 + 8 * pitch, y0 + 3 * rowp, 2 * pitch - 8, kh};
    button(shift, "Aa", s_shift);
    button(del, "Delete");
    if (tapped(shift)) { s_shift = !s_shift; s_tap = false; }
    if (tapped(del)) { if (len) s_edit[--len] = 0; s_shift = len == 0; s_tap = false; }

    int yb = y0 + 4 * rowp;
    Rect cancel{x0, yb, u(220), kh}, space{x0 + u(240), yb, u(520), kh}, clear{x0 + u(780), yb, u(180), kh},
         save{x0 + u(980), yb, u(220), kh};
    button(cancel, "Cancel");
    button(space, "Space");
    button(clear, "Clear");
    button(save, "Save", true);
    if (tapped(space)) {
        if (len && len < sizeof s_edit - 1) { s_edit[len++] = ' '; s_edit[len] = 0; s_shift = true; }
        s_tap = false;
    }
    if (tapped(clear)) { s_edit[0] = 0; s_shift = true; s_tap = false; }
    if (tapped(cancel)) { s_screen = Screen::Finder; s_tap = false; }
    if (tapped(save)) {
        while (len && s_edit[len - 1] == ' ') s_edit[--len] = 0;
        if (Saved *e = saved_get(s_target_view)) {
            snprintf(e->name, sizeof e->name, "%s", s_edit);
            saved_store();
        }
        target_rename(s_edit[0] ? s_edit : s_target_view.label);
        s_screen = Screen::Finder;
        s_tap = false;
    }
}

// ---------------------------------------------------------------- Wi-Fi channels
void screen_channels() {
    draw_header("Signal Finder & Analyzer");
    Rect back{24, 92 + Y0, 150, 56};
    button(back, "< Back");
    if (tapped(back)) { s_screen = Screen::Tools; s_tap = false; }
    text("Wi-Fi channels", 196, 100 + Y0, &fonts::FreeSansBold18pt7b, TEXT);
    text_wrap("How many networks sit on each 2.4 GHz channel. Bar colour = the strongest one there.", 196,
              136 + Y0, W - 220, &fonts::FreeSans9pt7b, MUTED, 22);

    devices_snapshot(s_list, Filter::Wifi, 30000);
    int cnt[15] = {0};
    float best[15];
    for (auto &b : best) b = -127;
    int most = 0;
    for (const auto &d : s_list) {
        if (d.channel < 1 || d.channel > 14) continue;
        ++cnt[d.channel];
        best[d.channel] = std::max(best[d.channel], d.rssi);
        most = std::max(most, cnt[d.channel]);
    }

    const int gx = 60, gy = 190 + Y0 + (P ? 24 : 0), gw = W - 120, gh = H - gy - 150, n = 13, slot = gw / n;
    canvas.fillRoundRect(gx - 16, gy - 10, gw + 32, gh + 64, 14, PANEL);
    for (int c = 1; c <= n; ++c) {
        int x = gx + (c - 1) * slot + 10, w = slot - 20;
        char l[8];
        snprintf(l, sizeof l, "%d", c);
        text(l, x + w / 2, gy + gh + 12, &fonts::FreeSansBold12pt7b, c == 1 || c == 6 || c == 11 ? TEXT : MUTED,
             top_center);
        if (!cnt[c]) { canvas.fillRect(x, gy + gh - 4, w, 4, LINE); continue; }
        int bh = std::max(12, (gh - 40) * cnt[c] / std::max(1, most));
        canvas.fillRoundRect(x, gy + gh - bh, w, bh, 8, strength_color(best[c]));
        snprintf(l, sizeof l, "%d", cnt[c]);
        text(l, x + w / 2, gy + gh - bh - 8, &fonts::FreeSansBold12pt7b, TEXT, bottom_center);
    }

    char msg[128];
    if (wifi_state() == WifiState::Failed) {
        snprintf(msg, sizeof msg, "Wi-Fi scanning isn't available.");
    } else if (!s_scan_on) {
        snprintf(msg, sizeof msg, "Scanning is off - turn Scan on to see channels.");
    } else if (s_list.empty()) {
        snprintf(msg, sizeof msg, "Looking for Wi-Fi networks...");
    } else {
        // Channels 1, 6 and 11 don't overlap each other. Score each by how
        // many networks sit on or near it, weighting strong ones more.
        int pick = 1;
        float low = 1e9f;
        for (int c : {1, 6, 11}) {
            float score = 0;
            for (const auto &d : s_list) {
                int dist = abs((int)d.channel - c);
                if (dist > 4) continue;
                score += (5 - dist) / 5.0f * std::clamp((d.rssi + 95.0f) / 55.0f, 0.1f, 1.0f);
            }
            if (score < low) { low = score; pick = c; }
        }
        snprintf(msg, sizeof msg, P ? "Least crowded: channel %d   (%u networks)"
                                   : "Least crowded for your own router: channel %d   (%u networks seen)", pick,
                 (unsigned)s_list.size());
    }
    text(msg, W / 2, H - 40, &fonts::FreeSansBold12pt7b, GOOD, middle_center);
}

// ---------------------------------------------------------------- More
// One settings row: title, description, and a button on the right.
bool tool_row(int i, const char *title, const char *desc, const char *btn, bool on, uint32_t desc_color = MUTED) {
    const int top = 164 + Y0, rh = P ? 124 : 86;
    Rect r{24, top + i * rh, W - 48, rh - 10};
    canvas.fillRoundRect(r.x, r.y, r.w, r.h, 12, PANEL);
    text(title, r.x + 24, r.y + 10, &fonts::FreeSansBold12pt7b, TEXT);
    text_wrap(desc, r.x + 24, r.y + 42, r.w - 24 - 252, &fonts::FreeSans9pt7b, desc_color, 22);
    Rect b{r.x + r.w - 236, r.y + (r.h - 56) / 2, 220, 56};
    button(b, btn, on);
    if (tapped(b)) { s_tap = false; return true; }
    return false;
}

void screen_tools() {
    draw_header("Signal Finder & Analyzer");
    Rect back{24, 92 + Y0, 150, 56};
    button(back, "< Back");
    if (tapped(back)) { s_screen = s_tools_back; s_tap = false; }
    text("More", 196, 106 + Y0, &fonts::FreeSansBold18pt7b, TEXT);

    char d[160];
    if (tool_row(0, "Tracker alert",
                 "Warns you if an AirTag, Tile or SmartTag stays near you for 10+ minutes while you're moving.",
                 alerts_tracker_enabled() ? "On" : "Off", alerts_tracker_enabled()))
        alerts_set_tracker_enabled(!alerts_tracker_enabled());

    if (alerts_watch_active())
        snprintf(d, sizeof d, "Watching. %u devices count as normal - anything else that shows up raises an alert.",
                 (unsigned)alerts_watch_known());
    else
        snprintf(d, sizeof d, "Marks what's around right now as normal, then alerts when a new Bluetooth device appears.");
    if (tool_row(1, "New-device watch", d, alerts_watch_active() ? "Stop" : "Start", alerts_watch_active())) {
        if (alerts_watch_active()) alerts_watch_stop();
        else alerts_watch_start();
    }

    if (tool_row(2, "Log to SD card", sdlog_status(), sdlog_active() ? "Stop" : "Start", sdlog_active())) {
        if (sdlog_active()) sdlog_stop();
        else sdlog_start();
    }

    if (tool_row(3, "Wi-Fi channel chart", "Shows which channels nearby networks use, and the least crowded one.",
                 "Open", false))
        s_screen = Screen::Channels;

    snprintf(d, sizeof d, "%u saved (pins, names, alarms). Change them from a device's finder screen.",
             (unsigned)saved_count());
    if (tool_row(4, "Saved devices", d, s_forget_confirm ? "Tap to confirm" : "Forget all", s_forget_confirm)) {
        if (s_forget_confirm) { saved_clear(); s_forget_confirm = false; }
        else if (saved_count()) s_forget_confirm = true;
    }

    if (tool_row(5, "Screen", "Turns the screen a quarter turn each tap: wide, tall, wide upside-down, tall upside-down.",
                 "Rotate", false)) {
        s_rotation = (s_rotation + 1) % 4;
        save_settings();
        s_rotate_pending = true;          // applied at the start of the next frame
    }
}

// ---------------------------------------------------------------- volume drop-down
// Opens under the Sound button. Drag the slider; it saves when you let go.
Rect vol_button() { Rect a, b, c; header_buttons(a, b, c); return b; }
Rect vol_panel()  { return P ? Rect{24, 146, W - 48, 156} : Rect{W - 566, 82, 546, 156}; }
constexpr int kVolTrackInset = 36, kVolTrackRight = 120;

void vol_input() {
    bool has_header = s_screen != Screen::Glow && s_screen != Screen::Rename;
    if (!has_header) s_vol_open = false;
    if (!s_vol_open) { s_vol_dirty = false; return; }
    Rect p = vol_panel();
    int tx0 = p.x + kVolTrackInset, tw = p.w - kVolTrackInset - kVolTrackRight;
    if (s_down && p.hit(s_cur_x, s_cur_y)) {
        int v = (s_cur_x - tx0) * 100 / tw;
        v = std::clamp(v, 0, 100);
        if (v < 3) v = 0;                       // easy to reach "off"
        if (v != s_volume) {
            s_volume = (uint8_t)v;
            apply_volume();
            s_vol_dirty = true;
        }
    } else if (!s_down && s_vol_dirty) {
        save_settings();
        if (s_volume) M5.Speaker.tone(1400, 70);    // hear the new level
        s_vol_dirty = false;
    }
    if (s_tap) {
        if (p.hit(s_tx, s_ty)) s_tap = false;                       // used by the slider
        else if (!vol_button().hit(s_tx, s_ty)) { s_vol_open = false; s_tap = false; }  // tap away closes
    }
}

void vol_draw() {
    if (!s_vol_open) return;
    Rect p = vol_panel();
    canvas.fillRoundRect(p.x, p.y, p.w, p.h, 16, PANEL_HI);
    canvas.drawRoundRect(p.x, p.y, p.w, p.h, 16, ACCENT);
    text("Volume", p.x + kVolTrackInset, p.y + 18, &fonts::FreeSansBold12pt7b, TEXT);
    text("beeps and alarms", p.x + kVolTrackInset + 110, p.y + 22, &fonts::FreeSans9pt7b, MUTED);
    int tx0 = p.x + kVolTrackInset, tw = p.w - kVolTrackInset - kVolTrackRight, ty = p.y + 98;
    int kx = tx0 + tw * s_volume / 100;
    canvas.fillRoundRect(tx0, ty - 7, tw, 14, 7, BG);
    if (s_volume) canvas.fillRoundRect(tx0, ty - 7, std::max(14, kx - tx0), 14, 7, ACCENT);
    canvas.fillCircle(kx, ty, 24, TEXT);
    canvas.fillCircle(kx, ty, 17, s_volume ? ACCENT : MUTED);
    char v[16];
    if (s_volume) snprintf(v, sizeof v, "%d%%", (int)s_volume);
    else snprintf(v, sizeof v, "Off");
    text(v, p.x + p.w - 24, ty, &fonts::FreeSansBold18pt7b, s_volume ? TEXT : MUTED, middle_right);
}

// ---------------------------------------------------------------- alerts
// The alert banner sits on top of whatever screen is showing. Its buttons are
// handled before the screen underneath so a tap can't fall through.
Rect alert_panel() { return P ? Rect{24, H - 290, W - 48, 266} : Rect{40, H - 236, W - 80, 212}; }

void alert_input() {
    const Alert &a = alerts_current();
    if (!a.active) return;
    Rect p = alert_panel();
    const int abw = P ? 200 : 216, astep = abw + 16;
    Rect find{p.x + p.w - 3 * astep, p.y + p.h - 78, abw, 60}, mine{p.x + p.w - 2 * astep, p.y + p.h - 78, abw, 60},
         no{p.x + p.w - astep, p.y + p.h - 78, abw, 60};
    if (tapped(find)) {
        DeviceView d = a.dev;
        alerts_dismiss();
        open_finder(d);
    } else if (a.type == AlertType::Tracker && tapped(mine)) {
        alerts_ignore_device();
    } else if (tapped(no)) {
        alerts_dismiss();
    }
    if (s_tap && p.hit(s_tx, s_ty)) s_tap = false;
}

void alert_draw() {
    const Alert &a = alerts_current();
    if (!a.active) return;
    uint32_t edge = a.type == AlertType::NewDevice ? ACCENT : BAD;
    Rect p = alert_panel();
    canvas.fillRoundRect(p.x, p.y, p.w, p.h, 18, PANEL_HI);
    canvas.drawRoundRect(p.x, p.y, p.w, p.h, 18, edge);
    canvas.drawRoundRect(p.x + 1, p.y + 1, p.w - 2, p.h - 2, 17, edge);
    text(a.title, p.x + 32, p.y + 22, &fonts::FreeSansBold18pt7b, edge);
    text_wrap(a.line, p.x + 32, p.y + 74, p.w - 64, &fonts::FreeSans12pt7b, TEXT, 30);
    const int abw = P ? 200 : 216, astep = abw + 16;
    Rect find{p.x + p.w - 3 * astep, p.y + p.h - 78, abw, 60}, mine{p.x + p.w - 2 * astep, p.y + p.h - 78, abw, 60},
         no{p.x + p.w - astep, p.y + p.h - 78, abw, 60};
    button(find, "Find it", true);
    if (a.type == AlertType::Tracker) button(mine, "It's mine");
    button(no, "Dismiss");

    // Alarm sound: two-tone for trackers and left-behind (20 s), a short chirp for new devices.
    static uint32_t next = 0;
    uint32_t t = now_ms(), el = t - a.since_ms;
    uint32_t limit = a.type == AlertType::NewDevice ? 900 : 20000;
    if (s_volume && el < limit && (int32_t)(t - next) >= 0) {
        M5.Speaker.tone((el / 450) % 2 ? 1300 : 1900, 160);
        next = t + 450;
    }
}

} // namespace

void ui_init() {
    load_settings();
    saved_load();
    alerts_init();
    M5.Display.setBrightness(170);
    apply_volume();
    make_canvas();
}

void ui_loop_once() {
    int64_t t0 = esp_timer_get_time();
    if (s_rotate_pending) { s_rotate_pending = false; make_canvas(); }
    poll_touch();
    s_tap = s_tap_latched;
    s_tx = s_latch_x;
    s_ty = s_latch_y;
    s_tap_latched = false;

    update_heading();

    // Only scan Wi-Fi when it's being looked at - it takes radio time from Bluetooth.
    Target tg = target_get();
    bool wifi_view = (s_screen == Screen::List && s_wifi_section) || s_screen == Screen::Channels ||
                     (tg.active && tg.wifi_channel);
    wifi_set_wanted(s_scan_on && wifi_view);

    alerts_tick();
    sdlog_tick();
    alert_input();
    vol_input();

    int64_t t1 = esp_timer_get_time();
    begin_frame();
    switch (s_screen) {
    case Screen::List:   screen_list();   break;
    case Screen::Finder: screen_finder(); break;
    case Screen::Sweep:  screen_sweep();  break;
    case Screen::Glow:   screen_glow();   break;
    case Screen::Rename: screen_rename(); break;
    case Screen::Tools:  screen_tools();  break;
    case Screen::Channels: screen_channels(); break;
    }
    vol_draw();
    alert_draw();
    int64_t t2 = esp_timer_get_time();
    poll_touch();
    push_frame();
    int64_t t3 = esp_timer_get_time();
    static int64_t a_in = 0, a_draw = 0, a_push = 0, last = 0; static int frames = 0;
    a_in += t1 - t0; a_draw += t2 - t1; a_push += t3 - t2; ++frames;
    if (t3 - last > 3000000) {
        ESP_LOGI("finder_ui", "%d frames in 3 s: input %lld ms, draw %lld ms, hand-off %lld ms per frame; moving %.0f s (activity %.2f)",
                 frames, a_in / frames / 1000, a_draw / frames / 1000, a_push / frames / 1000,
                 s_motion_s, s_activity);
        a_in = a_draw = a_push = 0; frames = 0; last = t3;
    }
    vTaskDelay(1);
}
