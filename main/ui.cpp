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
#include "esp_partition.h"
#include "esp_system.h"

#include "alerts.h"
#include "ble_scan.h"
#include "clock.h"
#include "devices.h"
#include "errlog.h"
#include "saved.h"
#include "sdlog.h"
#include "sdr.h"
#include "selfcopy.h"
#include "kbd.h"
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

enum class Screen { List, Finder, Sweep, Glow, Rename, Tools, Channels, Clock, History, Locked, Phone };
Screen   s_screen = Screen::List;
Filter   s_filter = Filter::Findable;
bool     s_wifi_section = false;
bool     s_phone_section = false;        // list shows phone bands (RTL-SDR)
int      s_phone_band = 0;               // band open on the phone finder screen
float    s_scroll = 0;                   // list scroll position, pixels
float    s_scroll_vel = 0;               // pixels per second it keeps gliding after a flick
int64_t  s_scroll_us = 0;                // when the list last moved, for frame-rate independent motion
float    s_scroll_at_press = 0;
bool     s_list_press = false;           // the finger went down on the list
bool     s_list_dragged = false;         // ...and has moved far enough to count as a scroll
int      s_list_last_y = 0;
int      s_key = 0;                      // this frame's key from the Tab5 Keyboard (0: none)
int      s_phone_sel = -1;               // Phone section: highlighted band
bool     s_sel_on = false;               // a list row is highlighted (keyboard)
int      s_sel_idx = 0;
uint8_t  s_sel_addr[6] = {0};
uint8_t  s_sel_type = 0;
bool     s_release = false;              // the finger was lifted this frame
bool     s_click_now = false;            // this frame's tap came from a PC: press and release at once
uint8_t  s_volume = 55;                 // percent, 0 = off
bool     s_vol_open = false;            // volume drop-down showing
bool     s_vol_dirty = false;           // slider moved, not saved yet
bool     s_down = false;                // finger on the screen right now
uint32_t s_down_since = 0;              // when it went down
uint32_t s_last_touch = 0;              // last time the screen was touched
uint32_t s_last_repeat = 0;
bool     s_saver = true;                // battery saver: dim when left alone
bool     s_dimmed = false;
int      s_brightness = 170;
Screen   s_lock_back = Screen::List;    // where to return after pocket mode
struct tm s_clk = {};                   // time being edited on the clock screen
int      s_cur_x = 0, s_cur_y = 0;      // where it is
bool     s_scan_on = true;
Screen   s_tools_back = Screen::List;   // where "Back" on the More screen goes
DeviceView s_target_view{};             // the device being tracked, as tapped
float    s_motion_s = 0;                // seconds the Tab5 has been moving
float    s_activity = 0;
bool     s_forget_confirm = false;
bool     s_launcher_confirm = false;     // "Launcher" tapped once
bool     s_off_confirm = false;          // "Power off" tapped once
int      s_report = 0;                   // error report pop-up: 0 none, 1 asking, 2 showing the result
uint32_t s_report_until = 0;
char     s_note[160] = "";               // text of the result banner (s_report == 2)
int      s_copy = 0;                     // copy app to SD: 0 idle, 1 show "copying", 2 copy now, 4 confirm window
bool     s_copy_delete = false;          // also delete the older copy afterwards
bool     s_copy_has_old = false;         // an older copy is on the card
bool     s_shot_wanted = false;          // a PC asked for a screenshot over USB
char     s_old_names[80] = "";
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

// Gives a button a keyboard key: pressing it counts as a tap on the button.
// Letters match either case.
void hot(const Rect &r, int key);

// Keyboard focus. Every button drawn in a frame is noted; the arrow keys move
// a highlight to the nearest button in that direction (using the buttons of
// the frame before), and Enter presses the highlighted one.
constexpr int kMaxNav = 72;
Rect s_nav[kMaxNav], s_nav_prev[kMaxNav];
int  s_nav_n = 0, s_nav_prev_n = 0;
bool s_foc_on = false;
int  s_foc_x = 0, s_foc_y = 0;          // a point inside the highlighted button

void focus_outline(const Rect &r) {
    canvas.drawRoundRect(r.x - 3, r.y - 3, r.w + 6, r.h + 6, 15, ACCENT);
    canvas.drawRoundRect(r.x - 2, r.y - 2, r.w + 4, r.h + 4, 14, ACCENT);
}

// Moves the highlight from (px, py) to the nearest button in the direction of
// an arrow key. False if there is none that way.
bool nav_move(int px, int py, int key) {
    int best = -1;
    long best_score = 0;
    Rect from{px, py, 1, 1};                // the button the highlight is on now, if any
    for (int i = 0; i < s_nav_prev_n; ++i)
        if (s_nav_prev[i].hit(px, py)) from = s_nav_prev[i];
    const bool vertical = key == K_UP || key == K_DOWN;
    for (int i = 0; i < s_nav_prev_n; ++i) {
        const Rect &r = s_nav_prev[i];
        if (r.hit(px, py)) continue;
        int dx = r.x + r.w / 2 - px, dy = r.y + r.h / 2 - py;
        int along = key == K_UP ? -dy : key == K_DOWN ? dy : key == K_LEFT ? -dx : dx;
        int across = (key == K_UP || key == K_DOWN) ? abs(dx) : abs(dy);
        if (along <= 4) continue;
        // Buttons in the same row (or column) win, then ones roughly straight
        // ahead; ones off to the side only count when there is nothing else.
        bool in_line = vertical ? (r.x < from.x + from.w && from.x < r.x + r.w)
                                : (r.y < from.y + from.h && from.y < r.y + r.h);
        if (!vertical && !in_line && across > along) continue;     // sideways keys don't jump rows
        long score = along + 2L * across + (in_line ? 0L : across <= along ? 100000L : 200000L);
        if (best < 0 || score < best_score) { best = i; best_score = score; }
    }
    if (best < 0) return false;
    s_foc_x = s_nav_prev[best].x + s_nav_prev[best].w / 2;
    s_foc_y = s_nav_prev[best].y + s_nav_prev[best].h / 2;
    s_foc_on = true;
    return true;
}

void hot(const Rect &r, int key);

// Called for every button as it is drawn.
void nav_button(const Rect &r) {
    if (s_nav_n < kMaxNav) s_nav[s_nav_n++] = r;
    if (s_foc_on && r.hit(s_foc_x, s_foc_y)) {
        focus_outline(r);
        hot(r, K_ENTER);
    }
}

void hot(const Rect &r, int key) {
    int c = s_key;
    if (key >= 'a' && key <= 'z' && c >= 'A' && c <= 'Z') c += 32;
    if (!c || c != key) return;
    s_key = 0;
    s_tap = true;
    s_click_now = true;
    s_tx = r.x + r.w / 2;
    s_ty = r.y + r.h / 2;
}

// Like tapped(), but keeps firing while the finger stays on the button.
bool tapped_repeat(const Rect &r) {
    if (tapped(r)) { s_last_repeat = now_ms(); return true; }
    uint32_t t = now_ms();
    if (s_down && r.hit(s_cur_x, s_cur_y) && t - s_down_since > 450 && t - s_last_repeat > 110) {
        s_last_repeat = t;
        return true;
    }
    return false;
}

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
    nav_button(r);
}

// A button that sits on a pop-up window. Pop-ups use the same shade as a
// normal button, so these are drawn darker with a brighter outline.
void popup_button(const Rect &r, const char *label, bool on = false) {
    canvas.fillRoundRect(r.x, r.y, r.w, r.h, 14, on ? ACCENT : BG);
    canvas.drawRoundRect(r.x, r.y, r.w, r.h, 14, on ? ACCENT : MUTED);
    text(label, r.x + r.w / 2, r.y + r.h / 2, &fonts::FreeSansBold12pt7b, on ? BG : TEXT, middle_center);
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
    nvs_set_u8(h, "sav", s_saver ? 1 : 0);
    nvs_commit(h);
    nvs_close(h);
}

void load_settings() {
    nvs_handle_t h;
    if (nvs_open("blefind", NVS_READONLY, &h) != ESP_OK) return;
    uint8_t v;
    if (nvs_get_u8(h, "rot", &v) == ESP_OK && v <= 3) s_rotation = v;
    if (nvs_get_u8(h, "sav", &v) == ESP_OK) s_saver = v != 0;
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
struct Frame {
    uint8_t  idx, rot;
    uint16_t w, h;
    uint16_t band_y, band_h;    // band to paint in the buffer once it is cleared
    uint16_t part_y, part_h;    // only these rows changed since the last frame (part_h 0: all of it)
};

// Filling big areas from the CPU is slow (the frame buffers live in PSRAM), so
// the list's row background is painted by the display hardware when a buffer
// is cleared: a band of PANEL colour across the list area. Each buffer
// remembers the band it was given; a screen that wants something else fixes
// the difference itself (see prepare_band).
uint16_t s_band_y[2] = {0, 0}, s_band_h[2] = {0, 0};
uint16_t s_want_y = 0, s_want_h = 0;     // band the next cleared buffer should get
uint16_t s_part_y = 0, s_part_h = 0;     // rows of this frame to send to the screen (0: all)
int      s_plain_frames = 0;             // frames in a row that showed just the list, nothing on top
bool     s_list_moving = false;          // the list is being dragged or is gliding
int      s_band_reset = 0;               // frames left to clear in software (after a rotation)
QueueHandle_t s_q_present = nullptr, s_q_free = nullptr;

// Copies a frame to the screen, turned to match the display rotation. With
// ph > 0 only rows py .. py+ph are copied: moving that much memory is what
// limits the frame rate, so a scrolling list sends just its own area.
bool hw_rotate(void *src, int w, int h, uint8_t rot, int py = 0, int ph = 0) {
    ppa_srm_oper_config_t op;
    memset(&op, 0, sizeof op);
    op.in.buffer = src;
    op.in.pic_w = w;
    op.in.pic_h = h;
    if (ph <= 0 || ph > h) { py = 0; ph = h; }
    op.in.block_w = w;
    op.in.block_h = ph;
    op.in.block_offset_y = py;
    switch (rot & 3) {      // where those rows land on the panel
    case 0:  op.out.block_offset_y = py; break;
    case 1:  op.out.block_offset_x = s_panel_w - py - ph; break;
    case 2:  op.out.block_offset_y = s_panel_h - py - ph; break;
    default: op.out.block_offset_x = py; break;
    }
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

bool hw_fill(void *dst, int w, int h, int y, int bh, uint32_t color) {
    ppa_fill_oper_config_t op;
    memset(&op, 0, sizeof op);
    op.out.buffer = dst;
    op.out.buffer_size = (uint32_t)w * h * 2;
    op.out.pic_w = w;
    op.out.pic_h = h;
    op.out.block_offset_y = y;
    op.out.fill_cm = PPA_FILL_COLOR_MODE_RGB565;
    op.fill_block_w = w;
    op.fill_block_h = bh;
    op.fill_argb_color.val = 0xFF000000u | color;
    op.mode = PPA_TRANS_MODE_BLOCKING;
    esp_err_t err = ppa_do_fill(s_ppa_fill, &op);
    if (err != ESP_OK) ESP_LOGE("finder_ui", "PPA clear failed: %s", esp_err_to_name(err));
    return err == ESP_OK;
}

void presenter_task(void *) {
    Frame f;
    for (;;) {
        xQueueReceive(s_q_present, &f, portMAX_DELAY);
        hw_rotate(s_buf[f.idx], f.w, f.h, f.rot, f.part_y, f.part_h);
        bool band = false;
        if (!hw_fill(s_buf[f.idx], f.w, f.h, 0, f.h, BG)) memset(s_buf[f.idx], 0, (size_t)f.w * f.h * 2);
        else if (f.band_h) band = hw_fill(s_buf[f.idx], f.w, f.h, f.band_y, f.band_h, PANEL);
        s_band_y[f.idx] = band ? f.band_y : 0;
        s_band_h[f.idx] = band ? f.band_h : 0;
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
    ok = ok && hw_fill(s_buf[0], W, H, 0, H, BG) && hw_fill(s_buf[1], W, H, 0, H, BG);
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
    s_band_reset = 3;                     // buffers in flight were prepared for the old shape
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
    if (!s_hw_push || s_band_reset > 0) {
        canvas.fillScreen(BG);
        s_band_h[s_cur] = 0;
        s_want_h = 0;
        if (s_band_reset > 0) --s_band_reset;
    }
}

// Makes sure rows y .. y+h of this frame start out PANEL-coloured across the
// whole width (and nothing else does), and asks for the same in later frames.
// h = 0: no band.
void prepare_band(int y, int h) {
    int hy = s_band_y[s_cur], hh = s_band_h[s_cur];
    if (hh && hy != y) { canvas.fillRect(0, hy, W, hh, BG); hh = 0; }
    if (hh > h) canvas.fillRect(0, y + h, W, hh - h, BG);
    else if (hh < h) canvas.fillRect(0, y + hh, W, h - hh, PANEL);
    s_band_y[s_cur] = y;
    s_band_h[s_cur] = h;
    if (s_band_reset == 0) { s_want_y = y; s_want_h = h; }
}

// Shows the frame just drawn.
void push_frame() {
    if (!s_hw_push) { canvas.pushSprite(&M5.Display, 0, 0); return; }
    Frame f{s_cur, s_canvas_rot, (uint16_t)W, (uint16_t)H, s_want_y, s_want_h, s_part_y, s_part_h};
    xQueueSend(s_q_present, &f, portMAX_DELAY);
    xQueueReceive(s_q_free, &s_cur, portMAX_DELAY);   // the other buffer, shown and cleared
    canvas.setBuffer(s_buf[s_cur], W, H);
}

// Touch is checked more than once a frame so a quick tap is never missed.
bool s_tap_latched = false;
bool s_release_latched = false;
int  s_latch_x = 0, s_latch_y = 0;
void poll_touch() {
    M5.update();
    auto td = M5.Touch.getDetail();
    if (s_down && !td.isPressed()) s_release_latched = true;
    if (td.wasPressed()) {
        s_tap_latched = true;
        s_latch_x = td.x;
        s_latch_y = td.y;
        s_down_since = now_ms();
    }
    s_down = td.isPressed();
    if (s_down) { s_cur_x = td.x; s_cur_y = td.y; s_last_touch = now_ms(); }
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
    if (s_screen == Screen::List) {
        hot(more, 'm');
        hot(scan, ' ');
    }
    if (tapped(scan)) {
        s_scan_on = !s_scan_on;
        ble_set_scanning(s_scan_on);
        s_tap = false;
    }
    if (tapped(vol)) { s_vol_open = !s_vol_open; s_tap = false; }
    if (tapped(more)) {
        if (s_screen == Screen::Tools) s_screen = s_tools_back;
        else { s_tools_back = s_screen; s_screen = Screen::Tools; s_forget_confirm = false; s_off_confirm = false; s_launcher_confirm = false; }
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
        text_wrap("This app can install ESP-Hosted 2.12.13 with Wi-Fi AND Bluetooth, which the M5Launcher can use too. It takes "
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

// ---------------------------------------------------------------- phone bands
// How far a band is above its own quiet level, and what that suggests.
float phone_activity(const SdrBand &b) { return b.valid ? std::max(0.0f, b.level_db - b.floor_db) : 0.0f; }

uint32_t activity_color(float db) {
    float t = std::clamp(db / 35.0f, 0.0f, 1.0f);
    return t < 0.5f ? blend(MUTED, WARN, t * 2) : blend(WARN, GOOD, (t - 0.5f) * 2);
}

const char *activity_text(float db) {
    if (db >= 30) return "Very strong - a phone is transmitting right here";
    if (db >= 18) return "Strong - a phone is transmitting close by";
    if (db >= 8)  return "Some activity - a phone is transmitting nearby";
    return "Quiet - nothing transmitting nearby right now";
}

// The Phone section of the list: one row per cellular band.
void phone_list(int sx, int sy) {
    if (s_key == K_UP && s_phone_sel <= 0) {            // above the first band: the buttons at the top
        nav_move(100, 164 + Y0, K_UP);
        s_phone_sel = -1;
        s_key = 0;
    }
    SdrState st = sdr_state();
    uint32_t sc = st == SdrState::Running ? MUTED : st == SdrState::Failed ? BAD : WARN;
    canvas.setClipRect(sx, sy - 24, W - sx - 16, 48);
    text(s_scan_on ? sdr_status_text() : "Scanning is off", sx, sy, &fonts::FreeSans12pt7b, sc, middle_left);
    canvas.clearClipRect();

    const int top = 164 + Y0 + (P ? 64 : 0), row_h = 80;
    if (st != SdrState::Running) {
        int y = text_wrap("This section measures how strongly phones are transmitting on each mobile band, "
                          "using an RTL-SDR dongle plugged into the Tab5's USB-A port.",
                          48, top + 40, W - 96, &fonts::FreeSans12pt7b, TEXT, 34);
        text_wrap("It shows strength only. Nothing is decoded, so it can't tell whose phone it is.",
                  48, y + 16, W - 96, &fonts::FreeSans12pt7b, MUTED, 34);
        return;
    }

    if (s_key == K_DOWN || s_key == K_UP) {              // keyboard: move the highlight
        int n = (int)sdr_band_count();
        s_phone_sel = s_phone_sel < 0 ? 0 : std::min(std::max(s_phone_sel + (s_key == K_DOWN ? 1 : -1), 0), n - 1);
        s_key = 0;
    }
    if (s_tap || s_foc_on) s_phone_sel = -1;
    for (size_t i = 0; i < sdr_band_count(); ++i) {
        SdrBand b = sdr_band(i);
        Rect r{24, top + (int)i * row_h, W - 48, row_h - 8};
        if (r.y + r.h > H - 60) break;
        canvas.fillRoundRect(r.x, r.y, r.w, r.h, 12, PANEL);
        float act = phone_activity(b);
        text(b.name, r.x + 24, r.y + 10, &fonts::FreeSansBold12pt7b, b.in_range ? TEXT : MUTED);
        char det[96];
        if (!b.in_range) snprintf(det, sizeof det, "%s  |  outside this dongle's range", b.detail);
        else if (!b.valid) snprintf(det, sizeof det, "%s  |  measuring...", b.detail);
        else snprintf(det, sizeof det, "%s  |  strongest near %.0f MHz", b.detail, b.peak_hz / 1e6f);
        canvas.setClipRect(r.x, r.y, r.w - (P ? 150 : 520), r.h);
        text(det, r.x + 24, r.y + 42, &fonts::FreeSans9pt7b, MUTED);
        canvas.clearClipRect();
        if (b.in_range && b.valid) {
            if (!P) {       // activity bar
                int bx = r.x + r.w - 500, bwid = 330;
                canvas.fillRoundRect(bx, r.y + 26, bwid, 20, 10, BG);
                int fw = (int)(bwid * std::clamp(act / 40.0f, 0.0f, 1.0f));
                if (fw > 10) canvas.fillRoundRect(bx, r.y + 26, fw, 20, 10, activity_color(act));
            }
            char v[24];
            snprintf(v, sizeof v, "+%.0f dB", act);
            text(v, r.x + r.w - 24, r.y + 12, &fonts::FreeSansBold12pt7b, activity_color(act), top_right);
            text(act >= 8 ? "active" : "quiet", r.x + r.w - 24, r.y + 44, &fonts::FreeSans9pt7b, MUTED, top_right);
        }
        if ((int)i == s_phone_sel) {
            focus_outline(Rect{r.x + 3, r.y + 3, r.w - 6, r.h - 6});
            hot(r, K_ENTER);
        }
        if (b.in_range && tapped(r)) {
            s_phone_band = (int)i;
            s_screen = Screen::Phone;
            s_next_beep = 0;
            s_tap = false;
            break;
        }
    }
    text(P ? "tap a band to follow it" : "dB above each band's quiet level  |  tap a band to follow it",
         W / 2, H - 42, &fonts::FreeSans12pt7b, MUTED, middle_center);
}

// Hot/cold screen for one phone band.
void screen_phone() {
    draw_header("Signal Finder & Analyzer");
    SdrBand b = sdr_band((size_t)s_phone_band);
    Rect back{24, 92 + Y0, 150, 56};
    button(back, "< Back");
    hot(back, K_ESC);
    if (tapped(back)) { s_screen = Screen::List; s_tap = false; }
    char sub[112];
    if (b.valid) snprintf(sub, sizeof sub, "%s  |  strongest near %.0f MHz", b.detail, b.peak_hz / 1e6f);
    else snprintf(sub, sizeof sub, "%s", b.detail);
    canvas.setClipRect(196, 92 + Y0, W - 196 - 16, 70);
    text(b.name, 196, 100 + Y0, &fonts::FreeSansBold18pt7b, TEXT);
    text(sub, 196, 136 + Y0, &fonts::FreeSans9pt7b, MUTED);
    canvas.clearClipRect();

    uint32_t t = now_ms();
    bool live = sdr_state() == SdrState::Running && b.valid && b.age_ms < 5000;
    float act = phone_activity(b);
    int y0 = 176 + Y0;

    char big[24];
    if (live) snprintf(big, sizeof big, "+%.0f", act);
    else snprintf(big, sizeof big, "--");
    uint32_t col = live ? activity_color(act) : MUTED;
    text(big, 60, y0, &fonts::FreeSansBold24pt7b, col, top_left, 3.0f);
    canvas.setTextSize(3.0f);
    int big_w = canvas.textWidth(big, &fonts::FreeSansBold24pt7b);
    text("dB", 60 + big_w + 16, y0 + 20, &fonts::FreeSans18pt7b, MUTED);

    int tx = P ? 60 : W / 2 + 20, ty = P ? y0 + 170 : y0;
    if (live) {
        text_wrap(activity_text(act), tx, ty + 10, W - tx - 40, &fonts::FreeSansBold12pt7b, TEXT, 32);
        text_wrap("Above this band's quiet level. Phones transmit in bursts, so start a call or a speed "
                  "test on the phone you're looking for.", tx, ty + 78, W - tx - 40, &fonts::FreeSans9pt7b,
                  MUTED, 22);
    } else {
        text_wrap(s_scan_on ? sdr_status_text() : "Scanning is off", tx, ty + 10, W - tx - 40,
                  &fonts::FreeSansBold12pt7b, WARN, 32);
    }

    // Meter: 0 .. 40 dB above quiet.
    int my = P ? ty + 170 : y0 + 170, mx = 60, mw = W - 120, mh = 46;
    canvas.fillRoundRect(mx, my, mw, mh, 12, PANEL);
    if (live) {
        int fw = std::max(24, (int)(mw * std::clamp(act / 40.0f, 0.0f, 1.0f)));
        for (int x = 0; x < fw; x += 4)
            canvas.fillRect(mx + x, my, std::min(4, fw - x), mh, activity_color(40.0f * x / mw));
    }
    canvas.drawRoundRect(mx, my, mw, mh, 12, LINE);
    text("quiet", mx, my + mh + 8, &fonts::FreeSans9pt7b, MUTED);
    text("strong", mx + mw, my + mh + 8, &fonts::FreeSans9pt7b, MUTED, top_right);

    // Graph of the last 30 s (raw level).
    int gy = my + mh + 44, gh = H - gy - 30, gx = mx, gw = mw;
    canvas.fillRoundRect(gx, gy, gw, gh, 12, PANEL);
    sdr_samples(s_samples, t - 30000);
    float lo = b.floor_db - 3, span = 46;
    for (const auto &sm : s_samples) {
        float age = (t - sm.t_ms) / 30000.0f;
        if (age > 1) continue;
        float k = std::clamp((sm.rssi - lo) / span, 0.0f, 1.0f);
        int xx = gx + gw - 12 - (int)(age * (gw - 24));
        int yy = gy + gh - 10 - (int)(k * (gh - 20));
        canvas.fillCircle(xx, yy, 4, activity_color(sm.rssi - b.floor_db));
    }
    text("last 30 seconds", gx + gw - 12, gy + 8, &fonts::FreeSans9pt7b, MUTED, top_right);

    // Beeps speed up with activity, like the Bluetooth finder.
    if (s_volume && live && act >= 6 && (int32_t)(t - s_next_beep) >= 0) {
        float k = std::clamp((act - 6.0f) / 30.0f, 0.0f, 1.0f);
        M5.Speaker.tone(700 + k * 1500, 35);
        s_next_beep = t + (uint32_t)(1200 - k * 1100);
    }
}

// ---------------------------------------------------------------- list
// Starts tracking a device and opens the hot/cold screen.
void phone_list(int sx, int sy);
void save_report();
void show_note(const char *text);
void open_copy_window();

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

bool list_shows_radio_panel() {
    return ble_state() == BleState::ControllerFailed || radio_update_state() != RadioUpdate::Idle;
}

void screen_list() {
    draw_header("Signal Finder & Analyzer");
    if (list_shows_radio_panel()) {
        radio_update_panel();
        return;
    }

    // Section switch: Bluetooth | Wi-Fi | Phone
    const int sw = P ? 216 : 190, sstep = P ? 228 : 198;
    Rect bt{24, 92 + Y0, sw, 56}, wf{24 + sstep, 92 + Y0, sw, 56}, ph{24 + 2 * sstep, 92 + Y0, sw, 56};
    button(bt, "Bluetooth", !s_wifi_section && !s_phone_section);
    button(wf, "Wi-Fi", s_wifi_section);
    button(ph, "Phone", s_phone_section);
    hot(s_phone_section ? bt : s_wifi_section ? ph : wf, K_TAB);
    if (tapped(bt)) { s_wifi_section = false; s_phone_section = false; s_scroll = 0; s_scroll_vel = 0; s_tap = false; }
    if (tapped(wf)) { s_wifi_section = true; s_phone_section = false; s_scroll = 0; s_scroll_vel = 0; s_tap = false; }
    if (tapped(ph)) { s_wifi_section = false; s_phone_section = true; s_scroll = 0; s_scroll_vel = 0; s_tap = false; }
    if (!P) canvas.drawFastVLine(628, 96, 48, LINE);
    const int fx = P ? 24 : 644, fy = P ? 184 + Y0 : 120;     // where the per-section status line goes

    if (s_phone_section) { phone_list(fx, fy); return; }

    if (!s_wifi_section) {
        const char *names[] = {"Trackers", "Named", "Everything"};
        const Filter vals[] = {Filter::Findable, Filter::Named, Filter::All};
        for (int i = 0; i < 3; ++i) {
            Rect r = P ? Rect{24 + i * 228, 156 + Y0, 216, 56} : Rect{644 + i * 208, 92, 196, 56};
            button(r, names[i], s_filter == vals[i]);
            hot(r, '1' + i);
            if (tapped(r)) { s_filter = vals[i]; s_scroll = 0; s_scroll_vel = 0; s_tap = false; }
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
        text(ws, fx, fy, &fonts::FreeSans12pt7b,
             wifi_state() == WifiState::Failed ? BAD : MUTED, middle_left);
    }

    devices_snapshot(s_list, s_wifi_section ? Filter::Wifi : s_filter,
                     s_wifi_section ? 30000 : 15000);
    // Pinned devices first, each group still strongest first.
    std::stable_partition(s_list.begin(), s_list.end(), [](const DeviceView &d) {
        Saved *sv = saved_find(d);
        return sv && (sv->flags & kSavedPin);
    });

    // The rows scroll under the finger. A row opens when the finger is lifted
    // without having dragged; a drag moves the list, and a flick keeps it gliding.
    const int top = 164 + Y0 + (P ? 64 : 0), row_h = 80, foot = 56;
    const int view_h = H - top - foot;
    const Rect view{0, top, W, view_h};
    const float max_scroll = std::max(0, (int)s_list.size() * row_h - view_h);

    bool click = false;
    int cx = 0, cy = 0;
    if (s_tap && view.hit(s_tx, s_ty)) {
        if (s_click_now) { click = true; cx = s_tx; cy = s_ty; }
        else {
            s_sel_on = false;
            s_list_press = true;
            s_list_dragged = false;
            s_scroll_at_press = s_scroll;
            s_list_last_y = s_ty;
            s_scroll_vel = 0;
        }
        s_tap = false;
    } else if (s_tap) {
        s_list_press = false;
    }
    const int64_t now_us = esp_timer_get_time();
    const float dt = std::min(0.2f, std::max(0.005f, (now_us - s_scroll_us) / 1e6f));
    s_scroll_us = now_us;
    if (s_list_press && s_down) {
        if (abs(s_cur_y - s_latch_y) > 14) s_list_dragged = true;
        if (s_list_dragged) {
            s_scroll = s_scroll_at_press - (s_cur_y - s_latch_y);
            s_scroll_vel = 0.5f * s_scroll_vel + 0.5f * (s_list_last_y - s_cur_y) / dt;
        }
        s_list_last_y = s_cur_y;
    } else if (s_list_press) {              // lifted
        if (!s_list_dragged && s_release) { click = true; cx = s_cur_x; cy = s_cur_y; }
        if (!s_list_dragged) s_scroll_vel = 0;
        s_list_press = false;
    } else if (fabsf(s_scroll_vel) > 20) {  // gliding after a flick
        s_scroll += s_scroll_vel * dt;
        s_scroll_vel *= expf(-3.0f * dt);
    } else {
        s_scroll_vel = 0;
    }
    // Keyboard: the arrows move a highlight through the list, Enter opens it.
    int sel = -1;
    if (s_foc_on) s_sel_on = false;                     // the highlight is on a button instead
    if (s_sel_on) {
        for (size_t i = 0; i < s_list.size(); ++i)
            if (s_list[i].addr_type == s_sel_type && memcmp(s_list[i].addr, s_sel_addr, 6) == 0) { sel = (int)i; break; }
        if (sel < 0) sel = std::min(s_sel_idx, (int)s_list.size() - 1);    // it left: stay nearby
    }
    if (s_key == K_UP && sel <= 0) {                    // above the first row: the buttons at the top
        nav_move(100, top, K_UP);
        s_sel_on = false;
        sel = -1;
        s_key = 0;
    }
    if ((s_key == K_DOWN || s_key == K_UP) && !s_list.empty()) {
        if (sel < 0) sel = std::min((int)s_list.size() - 1, ((int)s_scroll + row_h - 1) / row_h);
        else sel = std::min(std::max(sel + (s_key == K_DOWN ? 1 : -1), 0), (int)s_list.size() - 1);
        s_sel_on = true;
        if (sel * row_h < s_scroll) s_scroll = sel * row_h;
        if ((sel + 1) * row_h - 8 > s_scroll + view_h) s_scroll = (sel + 1) * row_h - 8 - view_h;
        s_scroll_vel = 0;
        s_key = 0;
    }
    if (sel >= 0) {
        s_sel_idx = sel;
        memcpy(s_sel_addr, s_list[sel].addr, 6);
        s_sel_type = s_list[sel].addr_type;
    } else {
        s_sel_on = false;
    }
    int key_open = -1;
    if (s_key == K_ENTER && sel >= 0) { key_open = sel; s_key = 0; }
    if (s_scroll < 0 || s_scroll > max_scroll) s_scroll_vel = 0;
    s_scroll = std::min(std::max(s_scroll, 0.0f), max_scroll);

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

    const int scroll = (int)s_scroll;
    bool opened = false;
    s_list_moving = (s_list_press && s_list_dragged) || s_scroll_vel != 0;
    s_part_y = top;
    s_part_h = view_h;
    // Row backgrounds come ready-painted as one band; only the gaps, the side
    // margins and the rounded corners are drawn here.
    const int content_h = std::min(view_h, std::max(0, (int)s_list.size() * row_h - 8 - scroll));
    prepare_band(top, content_h);
    canvas.fillRect(0, top, 24, content_h, BG);
    canvas.fillRect(W - 32, top, 32, content_h, BG);
    for (size_t idx = scroll / row_h; idx < s_list.size(); ++idx) {
        const DeviceView &d = s_list[idx];
        Rect r{24, top + (int)idx * row_h - scroll, W - 56, row_h - 8};
        if (r.y >= top + view_h) break;
        canvas.setClipRect(0, top, W, view_h);                      // rows stay inside the list area
        canvas.fillRect(r.x, r.y + r.h, r.w, row_h - r.h, BG);      // gap below the row
        for (int dy = 0; dy < 12; ++dy) {                           // round the corners
            int cut = 12 - (int)sqrtf(144.0f - (12 - dy) * (12 - dy));
            canvas.drawFastHLine(r.x, r.y + dy, cut, BG);
            canvas.drawFastHLine(r.x + r.w - cut, r.y + dy, cut, BG);
            canvas.drawFastHLine(r.x, r.y + r.h - 1 - dy, cut, BG);
            canvas.drawFastHLine(r.x + r.w - cut, r.y + r.h - 1 - dy, cut, BG);
        }
        if ((int)idx == sel) {                                      // keyboard highlight
            canvas.drawRoundRect(r.x, r.y, r.w, r.h, 12, ACCENT);
            canvas.drawRoundRect(r.x + 1, r.y + 1, r.w - 2, r.h - 2, 11, ACCENT);
        }
        signal_bars(r.x + 20, r.y + 16, d.rssi);
        Saved *sv = saved_find(d);
        const char *label = sv && sv->name[0] ? sv->name : d.label;
        // keep long text off the reading, and inside the list area
        const int ty0 = std::max(r.y, top), ty1 = std::min(r.y + r.h, top + view_h);
        if (ty1 > ty0) {
            canvas.setClipRect(r.x + 100, ty0, r.w - 100 - 150, ty1 - ty0);
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
            canvas.setClipRect(0, top, W, view_h);
        }
        char rs[32];
        snprintf(rs, sizeof rs, "%.0f dBm", d.rssi);
        text(rs, r.x + r.w - 24, r.y + 12, &fonts::FreeSansBold12pt7b, strength_color(d.rssi),
             top_right);
        char ago[32];
        if (d.age_ms < 2000) snprintf(ago, sizeof ago, "now");
        else snprintf(ago, sizeof ago, "%lus ago", (unsigned long)(d.age_ms / 1000));
        text(ago, r.x + r.w - 24, r.y + 44, &fonts::FreeSans9pt7b, MUTED, top_right);

        if ((click && view.hit(cx, cy) && r.hit(cx, cy)) || (int)idx == key_open) {
            canvas.clearClipRect();
            open_finder(d);
            opened = true;
            break;      // the list is no longer on screen
        }
    }
    canvas.clearClipRect();
    if (opened) return;

    if (max_scroll > 0) {                   // scroll bar at the right edge
        const int bh = std::max(40, (int)((float)view_h * view_h / (s_list.size() * row_h)));
        const int by = top + (int)((view_h - bh) * (s_scroll / max_scroll));
        canvas.fillRoundRect(W - 22, top, 8, view_h - 8, 4, PANEL);
        canvas.fillRoundRect(W - 22, by, 8, bh - 8, 4, MUTED);
    }
    char pg[96];
    snprintf(pg, sizeof pg, kbd_present() ? "%d found  |  arrows and Enter to pick one, Tab for the next section"
                            : max_scroll > 0 ? "%d found  |  drag to scroll, tap one to find it"
                                             : "%d found  |  tap one to find it", (int)s_list.size());
    text(pg, W / 2, H - 30, &fonts::FreeSans12pt7b, MUTED, middle_center);
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
    hot(back, K_ESC);
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
    hot(b_pin, 'p');
    if (!is_wifi) hot(b_alarm, 'a');
    hot(b_name, 'r');
    hot(b_glow, 'c');
    hot(sweep, 's');
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
    if (s_tap || s_key) { s_screen = Screen::Finder; s_tap = false; s_key = 0; }
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
    if (s_key >= 32 && s_key < 127) {                   // typed on the Tab5 Keyboard
        if (len < sizeof s_edit - 1 && (len || s_key != ' ')) { s_edit[len++] = (char)s_key; s_edit[len] = 0; }
        s_shift = false;
        s_key = 0;
    } else if (s_key == K_BS || s_key == K_DEL) {
        if (len) s_edit[--len] = 0;
        s_shift = len == 0;
        s_key = 0;
    }
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
    hot(cancel, K_ESC);
    if (tapped(cancel)) { s_screen = Screen::Finder; s_tap = false; }
    hot(save, K_ENTER);
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
    hot(back, K_ESC);
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
// Two columns of five in landscape, one column of ten in portrait.
bool tool_row(int i, const char *title, const char *desc, const char *btn, bool on, uint32_t desc_color = MUTED) {
    const int top = 164 + Y0, rh = P ? 104 : 108, colw = P ? W - 48 : (W - 48 - 16) / 2, bwid = P ? 200 : 170;
    Rect r = P ? Rect{24, top + i * rh, colw, rh - 10} : Rect{24 + (i / 5) * (colw + 16), top + (i % 5) * rh, colw, rh - 10};
    canvas.fillRoundRect(r.x, r.y, r.w, r.h, 12, PANEL);
    text(title, r.x + 24, r.y + 10, &fonts::FreeSansBold12pt7b, TEXT);
    canvas.setClipRect(r.x, r.y, r.w, r.h);
    text_wrap(desc, r.x + 24, r.y + 42, r.w - 24 - bwid - 32, &fonts::FreeSans9pt7b, desc_color, 22);
    canvas.clearClipRect();
    Rect b{r.x + r.w - bwid - 16, r.y + (r.h - 56) / 2, bwid, 56};
    button(b, btn, on);
    if (tapped(b)) { s_tap = false; return true; }
    return false;
}

// The M5Launcher is the "factory" app; this app sits in an OTA slot next to it.
bool launcher_installed() {
    static int have = -1;
    if (have < 0)
        have = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, nullptr) &&
               esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, nullptr);
    return have == 1;
}

// Leaves this app and starts the Launcher: with the boot selection cleared,
// the bootloader starts the factory app. The Launcher sets the selection
// again when an app is started from its list.
void go_to_launcher() {
    const esp_partition_t *ota =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_OTA, nullptr);
    if (!ota) return;
    sdlog_stop();                           // finish the log file cleanly
    save_settings();
    if (esp_partition_erase_range(ota, 0, ota->size) != ESP_OK) return;
    M5.Display.setBrightness(0);
    esp_restart();
}

void screen_tools() {
    draw_header("Signal Finder & Analyzer");
    const bool lnch = launcher_installed();
    Rect back{24, 92 + Y0, (P && lnch) ? 100 : 150, 56};
    button(back, "< Back");
    hot(back, K_ESC);
    if (tapped(back)) { s_screen = s_tools_back; s_tap = false; }
    if (!P) text("More", 196, 106 + Y0, &fonts::FreeSansBold18pt7b, TEXT);

    // Title-row buttons: Copy to SD, Save report, Launcher, Power off.
    const int tbw = P ? (lnch ? 136 : 164) : 220, tgap = P ? (lnch ? 6 : 8) : 12;
    Rect off{W - 24 - tbw, 92 + Y0, tbw, 56}, lch{off.x - tgap - tbw, 92 + Y0, tbw, 56},
         rep{(lnch ? lch.x : off.x) - tgap - tbw, 92 + Y0, tbw, 56}, cpy{rep.x - tgap - tbw, 92 + Y0, tbw, 56};

    // Leave the app and go to the Launcher. Two taps, like Power off.
    if (lnch) {
        button(lch, s_launcher_confirm ? "Confirm" : "Launcher", s_launcher_confirm);
        if (tapped(lch)) {
            s_tap = false;
            if (!s_launcher_confirm) s_launcher_confirm = true;
            else { go_to_launcher(); s_launcher_confirm = false; }
        } else if (s_tap) {
            s_launcher_confirm = false;     // any other tap cancels
        }
    }

    // Put the installed version on the SD card, ready to reinstall from the Launcher.
    button(cpy, "Copy to SD");
    if (tapped(cpy)) {
        open_copy_window();
        s_tap = false;
    }

    // Save an error report to the SD card, for sending to the developer.
    button(rep, "Save report");
    if (tapped(rep)) {
        save_report();
        s_tap = false;
    }

    // Power off. Two taps, so a stray touch can't switch the Tab5 off.
    button(off, s_off_confirm ? "Confirm" : "Power off", s_off_confirm);
    if (tapped(off)) {
        s_tap = false;
        if (!s_off_confirm) {
            s_off_confirm = true;
        } else {
            sdlog_stop();                   // finish the log file cleanly
            save_settings();
            M5.Display.setBrightness(0);
            M5.Power.powerOff();            // does not return if it worked
            M5.Display.setBrightness(s_brightness);
            s_off_confirm = false;
        }
    } else if (s_tap) {
        s_off_confirm = false;              // any other tap cancels
    }

    char d[160];
    if (tool_row(0, "Tracker alert", "Warns if an AirTag, Tile or SmartTag stays with you for 10+ minutes while moving.",
                 alerts_tracker_enabled() ? "On" : "Off", alerts_tracker_enabled()))
        alerts_set_tracker_enabled(!alerts_tracker_enabled());

    if (alerts_watch_active())
        snprintf(d, sizeof d, "Watching. %u devices count as normal.", (unsigned)alerts_watch_known());
    else
        snprintf(d, sizeof d, "Marks what's here now as normal, then alerts on new Bluetooth devices.");
    if (tool_row(1, "New-device watch", d, alerts_watch_active() ? "Stop" : "Start", alerts_watch_active())) {
        if (alerts_watch_active()) alerts_watch_stop();
        else alerts_watch_start();
    }

    snprintf(d, sizeof d, "%u alerts since power-on.", (unsigned)alerts_history_count());
    if (tool_row(2, "Alert history", d, "Open", false)) s_screen = Screen::History;

    if (tool_row(3, "Pocket mode", "Locks the touch screen. Alerts and alarms keep working.", "Lock", false)) {
        s_lock_back = s_tools_back;
        s_screen = Screen::Locked;
        s_down_since = now_ms();
    }

    if (tool_row(4, "Battery saver", "Dims the screen after a minute without a touch.", s_saver ? "On" : "Off", s_saver)) {
        s_saver = !s_saver;
        save_settings();
    }

    if (tool_row(5, "Log to SD card", sdlog_status(), sdlog_active() ? "Stop" : "Start", sdlog_active())) {
        if (sdlog_active()) sdlog_stop();
        else sdlog_start();
    }

    if (tool_row(6, "Wi-Fi channel chart", "Networks per channel, and the least crowded one.", "Open", false))
        s_screen = Screen::Channels;

    char now[24];
    clock_format(now, sizeof now, false);
    if (now[0]) snprintf(d, sizeof d, "%s  -  stamps the SD log and alerts.", now);
    else snprintf(d, sizeof d, "Not set. The SD log shows seconds since power-on.");
    if (tool_row(7, "Clock", d, "Set", false, now[0] ? MUTED : WARN)) {
        clock_now(&s_clk);
        s_clk.tm_sec = 0;
        s_screen = Screen::Clock;
    }

    snprintf(d, sizeof d, "%u saved (pins, names, alarms).", (unsigned)saved_count());
    if (tool_row(8, "Saved devices", d, s_forget_confirm ? "Confirm" : "Forget all", s_forget_confirm)) {
        if (s_forget_confirm) { saved_clear(); s_forget_confirm = false; }
        else if (saved_count()) s_forget_confirm = true;
    }

    if (tool_row(9, "Screen", "Quarter turn each tap: wide, tall, and both upside-down.", "Rotate", false)) {
        s_rotation = (s_rotation + 1) % 4;
        save_settings();
        s_rotate_pending = true;          // applied at the start of the next frame
    }
}

// ---------------------------------------------------------------- clock
int days_in_month(int year, int mon0) {
    static const int kDays[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    bool leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
    return mon0 == 1 && leap ? 29 : kDays[mon0];
}

void screen_clock() {
    text("Set the date and time", 24, 20, &fonts::FreeSansBold18pt7b, TEXT);
    text_wrap("Kept by the Tab5's clock chip, and used to stamp the SD log and alerts. Hold + or - to go faster.",
              24, 70, W - 48, &fonts::FreeSans9pt7b, MUTED, 22);

    static const char *kNames[] = {"Year", "Month", "Day", "Hour", "Minute"};
    int *field[] = {&s_clk.tm_year, &s_clk.tm_mon, &s_clk.tm_mday, &s_clk.tm_hour, &s_clk.tm_min};
    const int pitch = (W - 48) / 5, cw = pitch - 12, y0 = P ? 220 : 150;
    for (int i = 0; i < 5; ++i) {
        int x = 24 + i * pitch + 6;
        text(kNames[i], x + cw / 2, y0, &fonts::FreeSans12pt7b, MUTED, top_center);
        Rect up{x, y0 + 40, cw, 84}, dn{x, y0 + 250, cw, 84};
        button(up, "+");
        button(dn, "-");
        int shown = i == 0 ? *field[i] + 1900 : i == 1 ? *field[i] + 1 : *field[i];
        char v[8];
        snprintf(v, sizeof v, i == 0 ? "%d" : "%02d", shown);
        text(v, x + cw / 2, y0 + 187, &fonts::FreeSansBold24pt7b, TEXT, middle_center);
        int step = tapped_repeat(up) ? 1 : tapped_repeat(dn) ? -1 : 0;
        if (step) {
            *field[i] += step;
            if (i == 2) {       // the day wraps within the month
                int last = days_in_month(s_clk.tm_year + 1900, s_clk.tm_mon);
                if (s_clk.tm_mday > last) s_clk.tm_mday = 1;
                else if (s_clk.tm_mday < 1) s_clk.tm_mday = last;
            }
            s_tap = false;
        }
    }
    // Keep every field in range; wrap around at the ends.
    auto wrap = [](int &v, int lo, int hi) { if (v > hi) v = lo; else if (v < lo) v = hi; };
    wrap(s_clk.tm_year, 124, 199);              // 2024 .. 2099
    wrap(s_clk.tm_mon, 0, 11);
    wrap(s_clk.tm_hour, 0, 23);
    wrap(s_clk.tm_min, 0, 59);
    int dim = days_in_month(s_clk.tm_year + 1900, s_clk.tm_mon);
    if (s_clk.tm_mday > dim) s_clk.tm_mday = dim;      // e.g. the 31st after switching to a shorter month
    if (s_clk.tm_mday < 1) s_clk.tm_mday = 1;

    Rect cancel{24, H - 100, 220, 76}, save{W - 244, H - 100, 220, 76};
    button(cancel, "Cancel");
    button(save, "Save", true);
    hot(cancel, K_ESC);
    if (tapped(cancel)) { s_screen = Screen::Tools; s_tap = false; }
    if (tapped(save)) {
        clock_set(s_clk);
        s_screen = Screen::Tools;
        s_tap = false;
    }
}

// ---------------------------------------------------------------- alert history
void screen_history() {
    draw_header("Signal Finder & Analyzer");
    Rect back{24, 92 + Y0, 150, 56};
    button(back, "< Back");
    hot(back, K_ESC);
    if (tapped(back)) { s_screen = Screen::Tools; s_tap = false; }
    text("Alert history", 196, 106 + Y0, &fonts::FreeSansBold18pt7b, TEXT);

    const int top = 164 + Y0, rh = P ? 112 : 88;
    size_t n = alerts_history_count();
    if (!n) {
        text_wrap("No alerts since the Tab5 was switched on. Tracker, new-device and left-behind alerts are listed here, newest first.",
                  48, top + 60, W - 96, &fonts::FreeSans12pt7b, MUTED, 34);
        return;
    }
    uint32_t t = now_ms();
    for (size_t i = 0; i < n; ++i) {
        Rect r{24, top + (int)i * rh, W - 48, rh - 10};
        if (r.y + r.h > H - 10) break;
        const AlertRecord &a = alerts_history_at(i);
        uint32_t col = a.type == AlertType::NewDevice ? ACCENT : BAD;
        canvas.fillRoundRect(r.x, r.y, r.w, r.h, 12, PANEL);
        canvas.fillRoundRect(r.x, r.y, 8, r.h, 4, col);
        char when[40];
        uint32_t mins = (t - a.at_ms) / 60000;
        if (a.clock[0]) snprintf(when, sizeof when, "%s", a.clock);
        else if (mins < 1) snprintf(when, sizeof when, "just now");
        else if (mins < 120) snprintf(when, sizeof when, "%lu min ago", (unsigned long)mins);
        else snprintf(when, sizeof when, "%lu h ago", (unsigned long)(mins / 60));
        text(when, r.x + r.w - 20, r.y + 12, &fonts::FreeSans9pt7b, MUTED, top_right);
        canvas.setClipRect(r.x, r.y, r.w - 190, r.h);
        text(a.title, r.x + 28, r.y + 10, &fonts::FreeSansBold12pt7b, col);
        canvas.clearClipRect();
        canvas.setClipRect(r.x, r.y, r.w, r.h);
        text_wrap(a.line, r.x + 28, r.y + 42, r.w - 56, &fonts::FreeSans9pt7b, TEXT, 22);
        canvas.clearClipRect();
    }
}

// ---------------------------------------------------------------- pocket mode
// Touch does nothing except a two-second hold on the circle. Scanning, the
// tracker alert, the new-device watch and left-behind alarms all keep running.
void screen_locked() {
    canvas.fillScreen(0x000000u);
    const Alert &a = alerts_current();
    int cx = W / 2, cy = H / 2 + (P ? 120 : 70), R = 96;
    if (a.active) {
        uint32_t col = a.type == AlertType::NewDevice ? ACCENT : BAD;
        text(a.title, W / 2, P ? 200 : 70, &fonts::FreeSansBold18pt7b, col, middle_center);
        text_wrap(a.line, 48, P ? 250 : 110, W - 96, &fonts::FreeSans12pt7b, TEXT, 32);
    } else {
        text("Pocket mode", W / 2, P ? 240 : 90, &fonts::FreeSansBold18pt7b, MUTED, middle_center);
        text("Alerts and alarms are still on.", W / 2, P ? 290 : 136, &fonts::FreeSans12pt7b, MUTED, middle_center);
    }
    bool on_circle = s_down && (s_cur_x - cx) * (s_cur_x - cx) + (s_cur_y - cy) * (s_cur_y - cy) <= (R + 30) * (R + 30);
    static uint32_t hold_from = 0;
    uint32_t t = now_ms();
    if (!on_circle) hold_from = 0;
    else if (!hold_from) hold_from = t;
    float k = hold_from ? std::clamp((t - hold_from) / 2000.0f, 0.0f, 1.0f) : 0.0f;
    canvas.fillCircle(cx, cy, R, PANEL);
    canvas.drawCircle(cx, cy, R, LINE);
    if (k > 0) canvas.fillArc(cx, cy, R + 14, R + 2, -90, -90 + 360 * k, ACCENT);
    text("Hold", cx, cy - 14, &fonts::FreeSansBold12pt7b, TEXT, middle_center);
    text("2 seconds", cx, cy + 18, &fonts::FreeSans9pt7b, MUTED, middle_center);
    text(a.active ? "Hold the circle to unlock and respond" : "Hold the circle to unlock", cx, cy + R + 50,
         &fonts::FreeSans12pt7b, MUTED, middle_center);
    if (k >= 1.0f) {
        hold_from = 0;
        s_screen = s_lock_back;
        s_last_touch = t;
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
    if (s_key) {                                        // keyboard: Left/Right change it, Enter or Esc closes
        if (s_key == K_LEFT || s_key == K_RIGHT) {
            s_volume = (uint8_t)std::clamp((int)s_volume + (s_key == K_RIGHT ? 5 : -5), 0, 100);
            apply_volume();
            s_vol_dirty = true;
        } else if (s_key == K_ESC || s_key == K_ENTER) {
            s_vol_open = false;
        }
        s_key = 0;
    }
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

// ---------------------------------------------------------------- error report
// What the app is doing right now, for the top of an error report.
void save_report() {
    char st[420];
    Target tg = target_get();
    snprintf(st, sizeof st,
             "Bluetooth:    %s (error %d)\n"
             "Wi-Fi:        %s (error %d)\n"
             "Phone bands:  %s\n"
             "Screen:       %d, rotation %d, %dx%d\n"
             "Scan %s, volume %d%%, battery saver %s, battery %d%%\n"
             "Tracking:     %s\n"
             "SD log:       %s\n"
             "Saved devices: %u, alerts so far: %u",
             ble_state_text(), ble_error(), wifi_state_text(), wifi_error(), sdr_status_text(), (int)s_screen,
             (int)s_rotation, W, H, s_scan_on ? "on" : "off", (int)s_volume, s_saver ? "on" : "off",
             (int)M5.Power.getBatteryLevel(), !tg.active ? "nothing" : tg.wifi_channel ? "a Wi-Fi network" : "a Bluetooth device",
             sdlog_status(), (unsigned)saved_count(), (unsigned)alerts_history_count());
    if (errlog_save(st)) errlog_dismiss();
    snprintf(s_note, sizeof s_note, "%s", errlog_message());
    s_report = 2;
    s_report_until = now_ms() + 6000;
}

// After a crash, offers to save a report. Drawn over whatever screen is showing.
Rect report_panel() { return P ? Rect{24, H - 300, W - 48, 276} : Rect{40, H - 246, W - 80, 222}; }

void report_input() {
    if (!s_report) return;
    Rect p = report_panel();
    if (s_report == 1) {
        const int bw2 = P ? 300 : 260;
        Rect save{p.x + p.w - 2 * (bw2 + 16), p.y + p.h - 78, bw2, 60}, no{p.x + p.w - (bw2 + 16), p.y + p.h - 78, bw2, 60};
        if (tapped(save)) save_report();
        else if (tapped(no)) { errlog_dismiss(); s_report = 0; }
    } else if (s_tap || (int32_t)(now_ms() - s_report_until) >= 0) {
        s_report = 0;                       // result message: any tap, or time, closes it
    }
    if (s_tap && p.hit(s_tx, s_ty)) s_tap = false;
}

void report_draw() {
    if (!s_report) return;
    Rect p = report_panel();
    if (s_report == 2) p = Rect{p.x, p.y + p.h - 110, p.w, 110};
    canvas.fillRoundRect(p.x, p.y, p.w, p.h, 18, PANEL_HI);
    canvas.drawRoundRect(p.x, p.y, p.w, p.h, 18, ACCENT);
    canvas.drawRoundRect(p.x + 1, p.y + 1, p.w - 2, p.h - 2, 17, ACCENT);
    if (s_report == 2) {
        text_wrap(s_note, p.x + 32, p.y + 26, p.w - 64, &fonts::FreeSansBold12pt7b, TEXT, 32);
        return;
    }
    text_wrap(errlog_crash_text(), p.x + 32, p.y + 20, p.w - 64, &fonts::FreeSansBold18pt7b, WARN, 40);
    text_wrap("Save an error report to the SD card? You can send it to the developer. It has app logs "
              "and crash details, and no names of nearby devices or networks.",
              p.x + 32, p.y + (P ? 106 : 68), p.w - 64, &fonts::FreeSans12pt7b, TEXT, 30);
    const int bw2 = P ? 300 : 260;
    Rect save{p.x + p.w - 2 * (bw2 + 16), p.y + p.h - 78, bw2, 60}, no{p.x + p.w - (bw2 + 16), p.y + p.h - 78, bw2, 60};
    popup_button(save, "Save report", true);
    popup_button(no, "Not now");
}

// ---------------------------------------------------------------- copy app to SD
// Shows a banner with `text` for a few seconds.
void show_note(const char *text) {
    snprintf(s_note, sizeof s_note, "%s", text);
    s_report = 2;
    s_report_until = now_ms() + 7000;
}

// Opens the confirm window: what the copy is for, and whether to delete an
// older copy as well.
void open_copy_window() {
    s_copy_delete = false;
    s_copy_has_old = app_old_copies(s_old_names, sizeof s_old_names) > 0;
    s_copy = 4;
}

// Runs the copy. It takes several seconds, so a "copying" frame is drawn
// first (s_copy 1 -> 2), then the work happens between frames.
void copy_step() {
    if (s_copy == 1) { s_copy = 2; return; }
    if (s_copy != 2) return;
    char msg[160];
    bool ok = app_copy_to_sd(msg, sizeof msg);
    if (ok && s_copy_delete) {
        size_t n = app_remove_old_copies();
        size_t len = strlen(msg);
        snprintf(msg + len, sizeof msg - len, n ? ". Old copy deleted." : ". The old copy couldn't be deleted.");
    }
    s_copy = 0;
    s_copy_delete = false;
    show_note(msg);
}

Rect copy_panel() {
    const int h = P ? 470 : 330;
    return P ? Rect{24, (H - h) / 2, W - 48, h} : Rect{120, (H - h) / 2 + 20, W - 240, h};
}

// The three controls in the confirm window.
void copy_rects(const Rect &p, Rect &opt, Rect &go, Rect &cancel) {
    const int bw2 = P ? 300 : 260;
    go = Rect{p.x + p.w - 2 * (bw2 + 16), p.y + p.h - 80, bw2, 60};
    cancel = Rect{p.x + p.w - (bw2 + 16), p.y + p.h - 80, bw2, 60};
    opt = Rect{p.x + 32, p.y + p.h - 164, P ? p.w - 64 : 520, 60};
}

void copy_input() {
    if (s_copy != 4) return;
    Rect p = copy_panel(), opt, go, cancel;
    copy_rects(p, opt, go, cancel);
    hot(go, K_ENTER);
    hot(cancel, K_ESC);
    s_key = 0;
    if (s_copy_has_old && tapped(opt)) s_copy_delete = !s_copy_delete;
    else if (tapped(go)) s_copy = 1;
    else if (tapped(cancel)) s_copy = 0;
    s_tap = false;                          // the window is modal: nothing behind it reacts
}

void copy_draw() {
    if (s_copy == 0) return;
    if (s_copy != 4) {                      // copying: a slim banner at the bottom
        Rect b = P ? Rect{24, H - 134, W - 48, 110} : Rect{40, H - 134, W - 80, 110};
        canvas.fillRoundRect(b.x, b.y, b.w, b.h, 18, PANEL_HI);
        canvas.drawRoundRect(b.x, b.y, b.w, b.h, 18, ACCENT);
        text_wrap("Copying the app to the SD card. This takes a little while...", b.x + 32, b.y + 26, b.w - 64,
                  &fonts::FreeSansBold12pt7b, TEXT, 32);
        return;
    }
    Rect p = copy_panel(), opt, go, cancel;
    copy_rects(p, opt, go, cancel);
    canvas.fillRoundRect(p.x, p.y, p.w, p.h, 18, PANEL_HI);
    canvas.drawRoundRect(p.x, p.y, p.w, p.h, 18, ACCENT);
    canvas.drawRoundRect(p.x + 1, p.y + 1, p.w - 2, p.h - 2, 17, ACCENT);
    text("Update the app file on the SD card", p.x + 32, p.y + 22, P ? &fonts::FreeSansBold12pt7b : &fonts::FreeSansBold18pt7b,
         TEXT);
    char body[260];
    snprintf(body, sizeof body,
             "This puts the version you are running now onto the SD card as signal_finder.bin, replacing "
             "the one there. It keeps the card up to date, so the Launcher can reinstall the current "
             "version later.");
    int y = text_wrap(body, p.x + 32, p.y + (P ? 62 : 74), p.w - 64, &fonts::FreeSans12pt7b, TEXT, 30);
    if (s_copy_has_old) {
        char old[160];
        snprintf(old, sizeof old, "An older copy is also on the card: %s", s_old_names);
        text_wrap(old, p.x + 32, y + 6, p.w - 64, &fonts::FreeSans9pt7b, MUTED, 22);
        popup_button(opt, s_copy_delete ? "Delete the old copy too: Yes" : "Delete the old copy too: No", s_copy_delete);
    } else {
        text_wrap("No older copy of the app was found on the card.", p.x + 32, y + 6, p.w - 64, &fonts::FreeSans9pt7b,
                  MUTED, 22);
    }
    popup_button(go, "Copy to SD", true);
    popup_button(cancel, "Cancel");
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
    popup_button(find, "Find it", true);
    if (a.type == AlertType::Tracker) popup_button(mine, "It's mine");
    popup_button(no, "Dismiss");
}

// Alarm sound: two-tone for trackers and left-behind (20 s), a short chirp for new devices.
void alert_sound() {
    const Alert &a = alerts_current();
    if (!a.active) return;
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
    clock_init();
    // After a crash, offer to save a report - but only if there is a card to save it to.
    if (errlog_crashed() && sdlog_mount_card()) s_report = 1;
    M5.Display.setBrightness(s_brightness);
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
    s_release = s_release_latched;
    s_release_latched = false;
    s_click_now = false;
    bool locked = s_screen == Screen::Locked;
    // The tap that wakes a dimmed screen does nothing else; pocket mode ignores taps.
    if (s_dimmed || locked) s_tap = false;

    update_heading();

    // Only scan Wi-Fi when it's being looked at - it takes radio time from Bluetooth.
    Target tg = target_get();
    bool wifi_view = (s_screen == Screen::List && s_wifi_section) || s_screen == Screen::Channels ||
                     (tg.active && tg.wifi_channel);
    wifi_set_wanted(s_scan_on && wifi_view);
    // The RTL-SDR only runs while the Phone section or a phone band is on screen.
    sdr_set_wanted(s_scan_on && ((s_screen == Screen::List && s_phone_section) || s_screen == Screen::Phone));
    sdr_set_focus(s_screen == Screen::Phone ? s_phone_band : -1);
    if (sdr_wants_power_cycle()) sdr_power_cycle();     // I2C belongs to this task
    int ux = 0, uy = 0, usb_key = 0;
    switch (usb_command_take(&ux, &uy)) {               // requests from a PC over USB
    case UsbCommand::Tap:
        s_tap = true;
        s_click_now = true;
        s_tx = ux;
        s_ty = uy;
        s_last_touch = now_ms();
        break;
    case UsbCommand::Flick:
        s_scroll_vel = ux;
        break;
    case UsbCommand::Key:
        usb_key = ux;
        break;
    case UsbCommand::Screenshot:
        s_shot_wanted = true;
        break;
    case UsbCommand::CopyAppToSd:
        if (s_copy == 0) s_copy = 1;
        break;
    case UsbCommand::RadioFirmware:
        radio_update_force();
        break;
    case UsbCommand::RemoveOldCopies: {
        size_t n = app_remove_old_copies();
        s_copy = 0;
        show_note(n ? "Older copy deleted from the SD card" : "No older copy on the SD card");
        break;
    }
    default: break;
    }

    kbd_poll();
    s_key = usb_key ? usb_key : kbd_take();
    if (s_key) {
        if (!locked) s_last_touch = now_ms();
        if (s_dimmed || locked) s_key = 0;      // like a touch: it only wakes the screen
    }
    // Keyboard focus: work from the buttons the last frame drew.
    memcpy(s_nav_prev, s_nav, sizeof(Rect) * s_nav_n);
    s_nav_prev_n = s_nav_n;
    s_nav_n = 0;
    if (s_tap) s_foc_on = false;                // touch takes over
    if (s_foc_on) {                             // the highlighted button may be gone (new screen)
        bool there = false;
        for (int i = 0; i < s_nav_prev_n; ++i) there = there || s_nav_prev[i].hit(s_foc_x, s_foc_y);
        s_foc_on = there;
    }
    if (s_key >= K_UP && s_key <= K_RIGHT && !s_vol_open && s_copy == 0) {
        const bool list = s_screen == Screen::List;
        if (s_foc_on) {
            if (nav_move(s_foc_x, s_foc_y, s_key)) s_key = 0;
            else if (list && s_key == K_DOWN) s_foc_on = false;     // below the buttons: into the list
            else s_key = 0;
        } else if (!list) {                     // first arrow: start at the top-left button under the header
            int best = -1;
            for (int i = 0; i < s_nav_prev_n; ++i) {
                const Rect &r = s_nav_prev[i];
                if (r.y < 80 + Y0) continue;
                if (best < 0 || r.y < s_nav_prev[best].y - 8 ||
                    (abs(r.y - s_nav_prev[best].y) <= 8 && r.x < s_nav_prev[best].x)) best = i;
            }
            if (best >= 0) {
                s_foc_x = s_nav_prev[best].x + s_nav_prev[best].w / 2;
                s_foc_y = s_nav_prev[best].y + s_nav_prev[best].h / 2;
                s_foc_on = true;
            }
            s_key = 0;
        }
    }

    alerts_tick();
    sdlog_tick();
    if (locked) s_vol_open = false;
    else { copy_input(); report_input(); alert_input(); vol_input(); }

    int64_t t1 = esp_timer_get_time();
    begin_frame();
    s_list_moving = false;
    // Only the device list uses the ready-painted band; anything else starts plain.
    if (!(s_screen == Screen::List && !s_phone_section && !list_shows_radio_panel())) prepare_band(0, 0);
    switch (s_screen) {
    case Screen::List:   screen_list();   break;
    case Screen::Finder: screen_finder(); break;
    case Screen::Sweep:  screen_sweep();  break;
    case Screen::Glow:   screen_glow();   break;
    case Screen::Rename: screen_rename(); break;
    case Screen::Tools:  screen_tools();  break;
    case Screen::Channels: screen_channels(); break;
    case Screen::Clock:    screen_clock();    break;
    case Screen::History:  screen_history();  break;
    case Screen::Locked:   screen_locked();   break;
    case Screen::Phone:    screen_phone();    break;
    }
    // While the list moves and nothing else is showing, only the list area goes
    // to the screen. That needs the frame before to have been a plain list too.
    {
        bool plain = s_list_moving && s_screen == Screen::List && !s_vol_open && !alerts_current().active &&
                     s_report == 0 && s_copy == 0 && s_band_reset == 0 && !s_shot_wanted;
        if (!(plain && s_plain_frames >= 2)) s_part_h = 0;
        bool list_only = s_screen == Screen::List && !s_vol_open && !alerts_current().active && s_report == 0 &&
                         s_copy == 0 && s_band_reset == 0;
        s_plain_frames = list_only ? s_plain_frames + 1 : 0;
    }
    vol_draw();
    if (!locked) { alert_draw(); report_draw(); copy_draw(); }
    alert_sound();

    // Screen brightness: full in use, dimmed when left alone (battery saver),
    // nearly off in pocket mode unless an alert is showing.
    {
        uint32_t t = now_ms();
        bool in_use = s_screen == Screen::Finder || s_screen == Screen::Sweep || s_screen == Screen::Glow ||
                      s_screen == Screen::Rename || s_screen == Screen::Clock || s_screen == Screen::Phone;
        bool alert = alerts_current().active;
        locked = s_screen == Screen::Locked;
        s_dimmed = s_saver && !locked && !in_use && !alert && t - s_last_touch > 60000;
        int want = locked ? (alert ? 140 : 12) : s_dimmed ? 30 : 170;
        if (want != s_brightness) { M5.Display.setBrightness(want); s_brightness = want; }
    }
    int64_t t2 = esp_timer_get_time();
    poll_touch();
    if (s_shot_wanted) {    // half-size screenshot of the frame just drawn, as RGB565 hex rows
        s_shot_wanted = false;
        const uint16_t *px = (const uint16_t *)canvas.getBuffer();
        static char row[4 * 640 + 2];
        printf("\nSHOT screen %d %d\n", W / 2, H / 2);
        for (int y = 0; y + 2 <= H; y += 2) {
            int n = 0;
            for (int x = 0; x + 2 <= W; x += 2) n += sprintf(row + n, "%04x", px[y * W + x]);
            row[n] = 0;
            puts(row);
        }
        printf("ENDSHOT\n");
    }
    push_frame();
    copy_step();            // after the "copying" banner is on screen
    int64_t t3 = esp_timer_get_time();
    static int64_t a_in = 0, a_draw = 0, a_push = 0, last = 0; static int frames = 0;
    a_in += t1 - t0; a_draw += t2 - t1; a_push += t3 - t2; ++frames;
    static int64_t worst = 0;
    worst = std::max(worst, t3 - t0);
    if (t3 - last > 3000000) {
        ESP_LOGI("finder_ui", "%d frames in 3 s: input %lld ms, draw %lld ms, hand-off %lld ms per frame, longest %lld ms; moving %.0f s (activity %.2f)",
                 frames, a_in / frames / 1000, a_draw / frames / 1000, a_push / frames / 1000, worst / 1000,
                 s_motion_s, s_activity);
        a_in = a_draw = a_push = 0; frames = 0; last = t3; worst = 0;
    }
    if (s_dimmed || locked) {
        // Nothing much to show: redraw about five times a second to save power,
        // but keep checking for a touch so waking up feels instant.
        for (int i = 0; i < 8 && !s_tap_latched && !s_down; ++i) {
            vTaskDelay(pdMS_TO_TICKS(22));
            poll_touch();
        }
    } else {
        vTaskDelay(1);
    }
}
