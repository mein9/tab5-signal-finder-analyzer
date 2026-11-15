#include "kbd.h"

#include <M5Unified.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

namespace {
const char *TAG = "finder_kbd";

constexpr uint8_t  kAddr = 0x6D;
constexpr uint32_t kHz = 100000;
constexpr int      kSda = 0, kScl = 1;

// Registers (from M5Stack's Tab5 Keyboard I2C protocol).
constexpr uint8_t kRegEventNum = 0x02;   // events waiting; writing 0 empties the queue
constexpr uint8_t kRegMode = 0x10;       // 0 matrix positions, 1 HID codes, 2 characters
constexpr uint8_t kRegHidEvent = 0x30;   // modifier, key code (0 = released, FF FF = nothing)
constexpr uint8_t kRegVersion = 0xFE;
constexpr uint8_t kModeHid = 1;

bool     s_bus = false;
bool     s_present = false;
uint32_t s_next_probe = 0;
int      s_fails = 0;

int      s_queue[16];
int      s_head = 0, s_count = 0;

int      s_held = 0;            // key being held, for repeat
uint32_t s_repeat_at = 0;

uint32_t now_ms() { return (uint32_t)(esp_timer_get_time() / 1000); }

void push(int key) {
    if (!key || s_count == 16) return;
    s_queue[(s_head + s_count++) % 16] = key;
}

// USB HID keyboard usage code -> character or K_* code. US layout.
int translate(uint8_t mod, uint8_t code) {
    const bool shift = mod & 0x22;
    if (code >= 0x04 && code <= 0x1D) return (shift ? 'A' : 'a') + (code - 0x04);
    if (code >= 0x1E && code <= 0x27) return (shift ? "!@#$%^&*()" : "1234567890")[code - 0x1E];
    switch (code) {
    case 0x28: case 0x58: return K_ENTER;
    case 0x29: return K_ESC;
    case 0x2A: return K_BS;
    case 0x2B: return K_TAB;
    case 0x2C: return ' ';
    case 0x2D: return shift ? '_' : '-';
    case 0x2E: return shift ? '+' : '=';
    case 0x2F: return shift ? '{' : '[';
    case 0x30: return shift ? '}' : ']';
    case 0x31: return shift ? '|' : '\\';
    case 0x33: return shift ? ':' : ';';
    case 0x34: return shift ? '"' : '\'';
    case 0x35: return shift ? '~' : '`';
    case 0x36: return shift ? '<' : ',';
    case 0x37: return shift ? '>' : '.';
    case 0x38: return shift ? '?' : '/';
    case 0x4C: return K_DEL;
    case 0x4F: return K_RIGHT;
    case 0x50: return K_LEFT;
    case 0x51: return K_DOWN;
    case 0x52: return K_UP;
    default:   return 0;
    }
}

bool probe() {
    uint8_t ver = 0;
    if (!M5.Ex_I2C.readRegister(kAddr, kRegVersion, &ver, 1, kHz)) return false;
    if (!M5.Ex_I2C.writeRegister8(kAddr, kRegMode, kModeHid, kHz)) return false;
    M5.Ex_I2C.writeRegister8(kAddr, kRegEventNum, 0, kHz);      // start with an empty queue
    ESP_LOGI(TAG, "Tab5 Keyboard found (firmware %u)", ver);
    return true;
}
}  // namespace

void kbd_start() {
    if (M5.Ex_I2C.getPort() == M5.In_I2C.getPort()) {
        ESP_LOGW(TAG, "no spare I2C port for the keyboard");
        return;
    }
    M5.Ex_I2C.release();
    // Pull-ups so the lines sit idle (and a check fails fast) with no keyboard attached.
    gpio_set_pull_mode((gpio_num_t)kSda, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)kScl, GPIO_PULLUP_ONLY);
    s_bus = M5.Ex_I2C.begin(M5.Ex_I2C.getPort(), kSda, kScl);
    if (!s_bus) ESP_LOGW(TAG, "keyboard I2C bus didn't start");
}

void kbd_poll() {
    if (!s_bus) return;
    uint32_t t = now_ms();
    if (!s_present) {
        if ((int32_t)(t - s_next_probe) < 0) return;
        s_next_probe = t + 2000;
        s_present = probe();
        s_fails = 0;
        s_held = 0;
        return;
    }
    uint8_t n = 0;
    if (!M5.Ex_I2C.readRegister(kAddr, kRegEventNum, &n, 1, kHz)) {
        if (++s_fails >= 5) {
            ESP_LOGI(TAG, "Tab5 Keyboard gone");
            s_present = false;
            s_held = 0;
            s_next_probe = t + 2000;
        }
        return;
    }
    s_fails = 0;
    for (int i = 0; i < n && i < 8; ++i) {
        uint8_t ev[2] = {0xFF, 0xFF};
        if (!M5.Ex_I2C.readRegister(kAddr, kRegHidEvent, ev, 2, kHz)) break;
        if (ev[0] == 0xFF && ev[1] == 0xFF) break;
        if (ev[1] == 0) { s_held = 0; continue; }       // released
        int key = translate(ev[0], ev[1]);
        if (!key) continue;
        push(key);
        s_held = key;
        s_repeat_at = t + 450;
    }
    if (s_held && (int32_t)(t - s_repeat_at) >= 0) {    // held: repeat
        push(s_held);
        s_repeat_at = t + 90;
    }
}

bool kbd_present() { return s_present; }

int kbd_take() {
    if (!s_count) return 0;
    int key = s_queue[s_head];
    s_head = (s_head + 1) % 16;
    --s_count;
    return key;
}
