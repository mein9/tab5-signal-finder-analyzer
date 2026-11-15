#include "selfcopy.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "driver/usb_serial_jtag.h"
#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_image_format.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_rom_crc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sdlog.h"

namespace {

const char *TAG = "finder_copy";

const char *kTarget = "/sd/signal_finder.bin";
const char *kTemp = "/sd/sigfind.tmp";
// Names this app's file has had before. The user can choose to delete these.
const char *kOldNames[] = {"/sd/ble-finder.bin", "/sd/ble_finder.bin", "/sd/signal-finder.bin"};

volatile UsbCommand s_command = UsbCommand::None;
volatile int s_arg_x = 0, s_arg_y = 0;

void command_task(void *) {
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    if (!usb_serial_jtag_is_driver_installed() && usb_serial_jtag_driver_install(&cfg) != ESP_OK) {
        ESP_LOGW(TAG, "USB commands unavailable");
        vTaskDelete(nullptr);
        return;
    }
    char line[40];
    size_t n = 0;
    for (;;) {
        uint8_t c;
        if (usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(500)) != 1) continue;
        if (c == '\r' || c == '\n') {
            line[n] = 0;
            if (strcmp(line, "sdcopy") == 0) {
                s_command = UsbCommand::CopyAppToSd;
                ESP_LOGI(TAG, "USB command: copy the app to the SD card");
            } else if (strcmp(line, "sdclean") == 0) {
                s_command = UsbCommand::RemoveOldCopies;
                ESP_LOGI(TAG, "USB command: delete older copies from the SD card");
            } else if (strcmp(line, "shot") == 0) {
                s_command = UsbCommand::Screenshot;
            } else if (strncmp(line, "flick ", 6) == 0) {
                s_arg_x = atoi(line + 6);
                s_command = UsbCommand::Flick;
            } else if (strncmp(line, "tap ", 4) == 0) {
                int x = 0, y = 0;
                if (sscanf(line + 4, "%d %d", &x, &y) == 2) {
                    s_arg_x = x;
                    s_arg_y = y;
                    s_command = UsbCommand::Tap;
                }
            } else if (n) {
                ESP_LOGW(TAG, "unknown USB command");
            }
            n = 0;
        } else if (n < sizeof line - 1 && c >= 0x20 && c < 0x7F) {
            line[n++] = (char)c;
        } else {
            n = 0;      // too long or not text: start over
        }
    }
}

} // namespace

void usb_commands_start() { xTaskCreatePinnedToCore(command_task, "usbcmd", 4096, nullptr, 2, nullptr, 0); }

UsbCommand usb_command_take(int *x, int *y) {
    UsbCommand c = s_command;
    s_command = UsbCommand::None;
    *x = s_arg_x;
    *y = s_arg_y;
    return c;
}

bool app_copy_to_sd(char *msg, size_t n) {
    if (!sdlog_mount_card()) {
        snprintf(msg, n, "No SD card found, so the app wasn't copied");
        return false;
    }
    const esp_partition_t *part = esp_ota_get_running_partition();
    esp_partition_pos_t pos = {part ? part->address : 0, part ? part->size : 0};
    esp_image_metadata_t meta;
    if (!part || esp_image_get_metadata(&pos, &meta) != ESP_OK || meta.image_len == 0 || meta.image_len > part->size) {
        snprintf(msg, n, "Couldn't read the installed app");
        return false;
    }
    const size_t total = meta.image_len;
    const size_t kChunk = 16 * 1024;
    uint8_t *buf = (uint8_t *)heap_caps_malloc(kChunk, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        snprintf(msg, n, "Not enough memory to copy the app");
        return false;
    }

    // Write to a temporary name first, so a failed copy never leaves a broken
    // signal_finder.bin behind.
    unlink(kTemp);
    FILE *f = fopen(kTemp, "wb");
    bool ok = f != nullptr;
    uint32_t crc_flash = 0;
    for (size_t off = 0; ok && off < total; off += kChunk) {
        size_t len = total - off < kChunk ? total - off : kChunk;
        ok = esp_partition_read(part, off, buf, len) == ESP_OK && fwrite(buf, 1, len, f) == len;
        crc_flash = esp_rom_crc32_le(crc_flash, buf, len);
        vTaskDelay(1);
    }
    if (f) {
        ok = ok && fflush(f) == 0 && fsync(fileno(f)) == 0;
        fclose(f);
    }

    // Read it back and compare with what is in flash.
    uint32_t crc_file = 0;
    size_t read_total = 0;
    if (ok) {
        f = fopen(kTemp, "rb");
        ok = f != nullptr;
        size_t got;
        while (ok && (got = fread(buf, 1, kChunk, f)) > 0) {
            crc_file = esp_rom_crc32_le(crc_file, buf, got);
            read_total += got;
            vTaskDelay(1);
        }
        if (f) fclose(f);
        ok = ok && read_total == total && crc_file == crc_flash;
    }
    free(buf);

    if (!ok) {
        unlink(kTemp);
        snprintf(msg, n, "Copying to the SD card failed; the card was left as it was");
        return false;
    }
    unlink(kTarget);
    if (rename(kTemp, kTarget) != 0) {
        unlink(kTemp);
        snprintf(msg, n, "Couldn't name the file on the SD card");
        return false;
    }
    const esp_app_desc_t *app = esp_app_get_description();
    snprintf(msg, n, "Version %s copied to the SD card as signal_finder.bin", app->version);
    ESP_LOGI(TAG, "%s (%u bytes, checksum %08lx)", msg, (unsigned)total, (unsigned long)crc_file);
    return true;
}

size_t app_old_copies(char *names, size_t n) {
    if (names && n) names[0] = 0;
    if (!sdlog_mount_card()) return 0;
    size_t found = 0, used = 0;
    for (const char *old : kOldNames) {
        struct stat st;
        if (stat(old, &st) != 0) continue;
        if (names && used < n) used += snprintf(names + used, n - used, "%s%s", found ? ", " : "", old + 4);
        ++found;
    }
    return found;
}

size_t app_remove_old_copies() {
    if (!sdlog_mount_card()) return 0;
    size_t removed = 0;
    for (const char *old : kOldNames) {
        struct stat st;
        if (stat(old, &st) == 0 && unlink(old) == 0) {
            ++removed;
            ESP_LOGI(TAG, "deleted %s from the SD card", old + 4);
        }
    }
    return removed;
}
