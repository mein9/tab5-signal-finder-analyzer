#include "sdlog.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#include "clock.h"
#include "devices.h"

namespace {

const char *TAG = "finder_log";

// The Tab5's SD slot, used over SPI. (Its SDMMC controller is already busy
// talking to the ESP32-C6 radio chip.)
constexpr gpio_num_t kPinCs = GPIO_NUM_42, kPinSck = GPIO_NUM_43, kPinMiso = GPIO_NUM_39,
                     kPinMosi = GPIO_NUM_44;
constexpr uint32_t kEveryMs = 10000;

bool     s_mounted = false;
FILE    *s_file = nullptr;
uint32_t s_rows = 0;
uint32_t s_last = 0;
char     s_status[96] = "Not logging";
char     s_path[40];
std::vector<DeviceView> s_list;

bool mount() {
    if (s_mounted) return true;
    spi_bus_config_t bus = {};
    bus.mosi_io_num = kPinMosi;
    bus.miso_io_num = kPinMiso;
    bus.sclk_io_num = kPinSck;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 4096;
    esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        snprintf(s_status, sizeof s_status, "SD card bus failed (%s)", esp_err_to_name(err));
        return false;
    }
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    sdspi_device_config_t dev = SDSPI_DEVICE_CONFIG_DEFAULT();
    dev.gpio_cs = kPinCs;
    dev.host_id = SPI2_HOST;
    esp_vfs_fat_sdmmc_mount_config_t cfg = {};
    cfg.format_if_mount_failed = false;     // never format someone's card
    cfg.max_files = 2;
    sdmmc_card_t *card = nullptr;
    err = esp_vfs_fat_sdspi_mount("/sd", &host, &dev, &cfg, &card);
    if (err != ESP_OK) {
        snprintf(s_status, sizeof s_status, "No SD card found (%s)", esp_err_to_name(err));
        ESP_LOGW(TAG, "%s", s_status);
        return false;
    }
    s_mounted = true;
    return true;
}

// CSV field: quoted, with quotes doubled.
void put(FILE *f, const char *s) {
    fputc('"', f);
    for (; *s; ++s) {
        if (*s == '"') fputc('"', f);
        fputc(*s, f);
    }
    fputc('"', f);
}

const char *kind_text(Kind k) {
    switch (k) {
    case Kind::Named:     return "named";
    case Kind::AirPods:   return "airpods";
    case Kind::FindMy:    return "findmy";
    case Kind::Apple:     return "apple";
    case Kind::Tile:      return "tile";
    case Kind::SmartTag:  return "smarttag";
    case Kind::Samsung:   return "samsung";
    case Kind::FastPair:  return "fastpair";
    case Kind::Microsoft: return "microsoft";
    case Kind::WifiAP:    return "wifi";
    default:              return "unknown";
    }
}

} // namespace

bool sdlog_active() { return s_file != nullptr; }
uint32_t sdlog_rows() { return s_rows; }
const char *sdlog_status() { return s_status; }

bool sdlog_start() {
    if (s_file) return true;
    if (!mount()) return false;
    mkdir("/sd/FINDLOG", 0775);
    // 8.3 names: LOG00001.CSV, LOG00002.CSV, ...
    for (int i = 1; i < 100000; ++i) {
        snprintf(s_path, sizeof s_path, "/sd/FINDLOG/LOG%05d.CSV", i);
        struct stat st;
        if (stat(s_path, &st) != 0) break;
    }
    s_file = fopen(s_path, "w");
    if (!s_file) {
        snprintf(s_status, sizeof s_status, "Couldn't create a file on the SD card");
        return false;
    }
    fputs("uptime_s,clock,type,name,detail,address,signal_dbm,peak_dbm,wifi_channel,packets\n", s_file);
    s_rows = 0;
    s_last = 0;
    snprintf(s_status, sizeof s_status, "Logging to %s", s_path + 4);
    ESP_LOGI(TAG, "%s", s_status);
    return true;
}

void sdlog_stop() {
    if (!s_file) return;
    fclose(s_file);
    s_file = nullptr;
    snprintf(s_status, sizeof s_status, "Saved %lu rows to %s", (unsigned long)s_rows, s_path + 4);
}

void sdlog_tick() {
    if (!s_file) return;
    uint32_t t = now_ms();
    if (s_last && t - s_last < kEveryMs) return;
    s_last = t | 1;

    char clock[24];
    clock_format(clock, sizeof clock, true);

    devices_snapshot(s_list, Filter::Any, kEveryMs);
    for (const auto &d : s_list) {
        fprintf(s_file, "%lu,%s,%s,", (unsigned long)(t / 1000), clock, kind_text(d.kind));
        put(s_file, d.label);
        fputc(',', s_file);
        put(s_file, d.detail);
        fprintf(s_file, ",%02X:%02X:%02X:%02X:%02X:%02X,%.0f,%d,", d.addr[5], d.addr[4], d.addr[3],
                d.addr[2], d.addr[1], d.addr[0], d.rssi, (int)d.peak);
        if (d.kind == Kind::WifiAP) fprintf(s_file, "%u", (unsigned)d.channel);
        fprintf(s_file, ",%lu\n", (unsigned long)d.count);
        ++s_rows;
    }
    bool ok = fflush(s_file) == 0 && fsync(fileno(s_file)) == 0;
    if (!ok) {
        fclose(s_file);
        s_file = nullptr;
        snprintf(s_status, sizeof s_status, "SD card write failed - logging stopped");
        return;
    }
    snprintf(s_status, sizeof s_status, "Logging to %s  |  %lu rows", s_path + 4, (unsigned long)s_rows);
}
