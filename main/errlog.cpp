#include "errlog.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "esp_app_desc.h"
#include "esp_attr.h"
#include "esp_chip_info.h"
#include "esp_core_dump.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "esp_rom_sys.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"

#include "clock.h"
#include "sdlog.h"

namespace {

constexpr uint32_t kMagic = 0x4C4F4731;         // "LOG1"
constexpr size_t   kRing = 12 * 1024;

// Lives in memory that is not cleared by a restart, so after a crash the
// lines leading up to it are still there. (It is lost when power is removed.)
struct Ring {
    uint32_t magic;
    uint32_t head;          // next write position
    uint32_t used;          // bytes valid, up to kRing
    char     data[kRing];
};
__NOINIT_ATTR Ring s_ring;

char     *s_prev = nullptr;         // log of the run before this one
size_t    s_prev_len = 0;
bool      s_crashed = false;
char      s_crash_text[64] = "";
char      s_message[112] = "";
vprintf_like_t s_orig_vprintf = nullptr;
portMUX_TYPE   s_mux = portMUX_INITIALIZER_UNLOCKED;

void ring_put(const char *s, size_t n) {
    portENTER_CRITICAL(&s_mux);
    for (size_t i = 0; i < n; ++i) {
        s_ring.data[s_ring.head] = s[i];
        s_ring.head = (s_ring.head + 1) % kRing;
    }
    s_ring.used = s_ring.used + n > kRing ? kRing : s_ring.used + n;
    portEXIT_CRITICAL(&s_mux);
}

// Copies the ring, oldest first. Returns the number of bytes.
size_t ring_copy(char *out) {
    portENTER_CRITICAL(&s_mux);
    size_t n = s_ring.used, start = (s_ring.head + kRing - n) % kRing;
    for (size_t i = 0; i < n; ++i) out[i] = s_ring.data[(start + i) % kRing];
    portEXIT_CRITICAL(&s_mux);
    return n;
}

int log_hook(const char *fmt, va_list ap) {
    char line[160];
    va_list copy;
    va_copy(copy, ap);
    int n = vsnprintf(line, sizeof line, fmt, copy);
    va_end(copy);
    if (n > 0) ring_put(line, (size_t)n < sizeof line ? (size_t)n : sizeof line - 1);
    return s_orig_vprintf ? s_orig_vprintf(fmt, ap) : vprintf(fmt, ap);
}

const char *reset_text(esp_reset_reason_t r) {
    switch (r) {
    case ESP_RST_POWERON:   return "power-on";
    case ESP_RST_EXT:       return "reset pin";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_PANIC:     return "crash (panic)";
    case ESP_RST_INT_WDT:   return "crash (interrupt watchdog)";
    case ESP_RST_TASK_WDT:  return "crash (task watchdog)";
    case ESP_RST_WDT:       return "watchdog";
    case ESP_RST_DEEPSLEEP: return "wake from sleep";
    case ESP_RST_BROWNOUT:  return "low battery or power dip";
    case ESP_RST_USB:       return "USB restart";
    default:                return "unknown";
    }
}

bool core_dump_present() {
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    return esp_core_dump_image_check() == ESP_OK;
#else
    return false;
#endif
}

// Next unused "<prefix>NNNNN.<ext>" in /sd/FINDLOG (8.3 names).
void next_name(char *out, size_t n, const char *prefix, const char *ext) {
    for (int i = 1; i < 100000; ++i) {
        snprintf(out, n, "/sd/FINDLOG/%s%05d.%s", prefix, i, ext);
        struct stat st;
        if (stat(out, &st) != 0) return;
    }
}

} // namespace

void errlog_init() {
    esp_reset_reason_t why = esp_reset_reason();
    bool survived = s_ring.magic == kMagic && s_ring.head < kRing && s_ring.used <= kRing;
    if (survived && s_ring.used) {
        s_prev = (char *)heap_caps_malloc(kRing, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_prev) s_prev = (char *)malloc(kRing);
        if (s_prev) s_prev_len = ring_copy(s_prev);
    }
    s_ring.magic = kMagic;
    s_ring.head = 0;
    s_ring.used = 0;
    s_orig_vprintf = esp_log_set_vprintf(log_hook);

    // A crash: the chip says so, or a crash dump is waiting in flash.
    bool crash_reset = why == ESP_RST_PANIC || why == ESP_RST_INT_WDT || why == ESP_RST_TASK_WDT;
    s_crashed = crash_reset || core_dump_present();
    if (s_crashed)
        snprintf(s_crash_text, sizeof s_crash_text, "The app stopped unexpectedly last time");
    ESP_LOGI("finder_err", "started by: %s (chip code %d)%s", reset_text(why), (int)esp_rom_get_reset_reason(0),
             s_crashed ? " - previous run crashed" : "");
}

bool errlog_crashed() { return s_crashed; }
const char *errlog_crash_text() { return s_crash_text; }
const char *errlog_message() { return s_message; }

void errlog_dismiss() {
    s_crashed = false;
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    if (core_dump_present()) esp_core_dump_image_erase();
#endif
}

bool errlog_save(const char *status) {
    if (!sdlog_mount_card()) {
        snprintf(s_message, sizeof s_message, "No SD card found, so the report wasn't saved");
        return false;
    }
    mkdir("/sd/FINDLOG", 0775);
    char path[40];
    next_name(path, sizeof path, "ERR", "TXT");
    FILE *f = fopen(path, "w");
    if (!f) {
        snprintf(s_message, sizeof s_message, "Couldn't create a file on the SD card");
        return false;
    }

    const esp_app_desc_t *app = esp_app_get_description();
    esp_chip_info_t chip;
    esp_chip_info(&chip);
    char now[24];
    clock_format(now, sizeof now, true);
    fprintf(f, "Signal Finder & Analyzer - error report\n");
    fprintf(f, "This file has app logs, crash details and settings. It has no names or\n");
    fprintf(f, "addresses of nearby devices or networks.\n\n");
    fprintf(f, "Saved:        %s\n", now[0] ? now : "(clock not set)");
    fprintf(f, "App:          %s %s, built %s %s\n", app->project_name, app->version, app->date, app->time);
    fprintf(f, "ESP-IDF:      %s\n", app->idf_ver);
    fprintf(f, "Chip:         ESP32-P4 revision v%d.%d\n", chip.revision / 100, chip.revision % 100);
    fprintf(f, "Started by:   %s (chip code %d)\n", reset_text(esp_reset_reason()), (int)esp_rom_get_reset_reason(0));
    fprintf(f, "Running for:  %lld s\n", (long long)(esp_timer_get_time() / 1000000));
    fprintf(f, "Free memory:  %u KB internal, %u KB external\n",
            (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
            (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
    fprintf(f, "Last run:     %s\n", s_crashed ? s_crash_text : "ended normally");
    if (status && status[0]) fprintf(f, "\n--- Status now ---\n%s\n", status);

    char core_path[40] = "";
#if CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH
    if (core_dump_present()) {
        fprintf(f, "\n--- Crash details ---\n");
#if CONFIG_ESP_COREDUMP_DATA_FORMAT_ELF
        esp_core_dump_summary_t *sum = (esp_core_dump_summary_t *)malloc(sizeof(esp_core_dump_summary_t));
        if (sum && esp_core_dump_get_summary(sum) == ESP_OK) {
            fprintf(f, "Task:         %s\n", sum->exc_task);
            fprintf(f, "Program counter: 0x%08lx\n", (unsigned long)sum->exc_pc);
            fprintf(f, "Cause (mcause):  0x%08lx   address (mtval): 0x%08lx\n", (unsigned long)sum->ex_info.mcause,
                    (unsigned long)sum->ex_info.mtval);
            fprintf(f, "Return address:  0x%08lx   stack pointer:   0x%08lx\n", (unsigned long)sum->ex_info.ra,
                    (unsigned long)sum->ex_info.sp);
            fprintf(f, "App build id:    %.16s...\n", (const char *)sum->app_elf_sha256);
        }
        free(sum);
#endif
        // The raw crash dump, for the developer's tools.
        size_t addr = 0, size = 0;
        const esp_partition_t *part =
            esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, nullptr);
        if (part && esp_core_dump_image_get(&addr, &size) == ESP_OK && size && size <= part->size) {
            next_name(core_path, sizeof core_path, "COR", "BIN");      // 8.3 names: COR00001.BIN
            FILE *c = fopen(core_path, "wb");
            if (c) {
                static uint8_t buf[1024];
                size_t off = addr - part->address;
                for (size_t done = 0; done < size; done += sizeof buf) {
                    size_t n = size - done < sizeof buf ? size - done : sizeof buf;
                    if (esp_partition_read(part, off + done, buf, n) != ESP_OK) break;
                    fwrite(buf, 1, n, c);
                }
                fclose(c);
                fprintf(f, "Full crash dump saved as %s (%u bytes)\n", core_path + 4, (unsigned)size);
            } else {
                core_path[0] = 0;
            }
        }
    }
#endif

    if (s_prev && s_prev_len) {
        fprintf(f, "\n--- Log from the run before this one ---\n");
        fwrite(s_prev, 1, s_prev_len, f);
        if (s_prev[s_prev_len - 1] != '\n') fputc('\n', f);
    }
    char *cur = (char *)heap_caps_malloc(kRing, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (cur) {
        size_t n = ring_copy(cur);
        fprintf(f, "\n--- Log from this run ---\n");
        fwrite(cur, 1, n, f);
        free(cur);
    }
    bool ok = fflush(f) == 0 && fsync(fileno(f)) == 0;
    fclose(f);
    if (!ok) {
        snprintf(s_message, sizeof s_message, "The SD card write failed");
        return false;
    }
    snprintf(s_message, sizeof s_message, "Saved to %s on the SD card", path + 4);
    ESP_LOGI("finder_err", "%s%s%s", s_message, core_path[0] ? " + " : "", core_path[0] ? core_path + 4 : "");
    return true;
}
