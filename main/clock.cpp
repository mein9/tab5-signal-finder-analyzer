#include "clock.h"

#include <M5Unified.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>

namespace {

// Anything earlier than this means "never set".
constexpr int kMinYear = 2024;

void set_system(const struct tm &t) {
    struct tm copy = t;
    copy.tm_isdst = 0;
    time_t secs = mktime(&copy);            // no time zone is configured, so this is wall-clock time
    struct timeval tv = {secs, 0};
    settimeofday(&tv, nullptr);
}

// The date this firmware was built: a starting point for the set-clock screen.
void build_time(struct tm *out) {
    static const char kMonths[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char mon[4] = {0};
    int d = 1, y = 2026, hh = 12, mm = 0, ss = 0;
    sscanf(__DATE__, "%3s %d %d", mon, &d, &y);
    sscanf(__TIME__, "%d:%d:%d", &hh, &mm, &ss);
    const char *m = strstr(kMonths, mon);
    memset(out, 0, sizeof *out);
    out->tm_year = y - 1900;
    out->tm_mon = m ? (int)(m - kMonths) / 3 : 0;
    out->tm_mday = d;
    out->tm_hour = hh;
    out->tm_min = mm;
}

} // namespace

void clock_init() {
    if (!M5.Rtc.isEnabled()) return;
    auto dt = M5.Rtc.getDateTime();
    if (dt.date.year < kMinYear) return;
    struct tm t = {};
    t.tm_year = dt.date.year - 1900;
    t.tm_mon = dt.date.month - 1;
    t.tm_mday = dt.date.date;
    t.tm_hour = dt.time.hours;
    t.tm_min = dt.time.minutes;
    t.tm_sec = dt.time.seconds;
    set_system(t);
}

bool clock_valid() {
    time_t now = time(nullptr);
    struct tm t;
    localtime_r(&now, &t);
    return t.tm_year + 1900 >= kMinYear;
}

void clock_now(struct tm *out) {
    if (!clock_valid()) { build_time(out); return; }
    time_t now = time(nullptr);
    localtime_r(&now, out);
}

void clock_set(const struct tm &t) {
    set_system(t);
    if (!M5.Rtc.isEnabled()) return;
    m5::rtc_datetime_t dt;
    dt.date.year = (int16_t)(t.tm_year + 1900);
    dt.date.month = (int8_t)(t.tm_mon + 1);
    dt.date.date = (int8_t)t.tm_mday;
    dt.date.weekDay = -1;
    dt.time.hours = (int8_t)t.tm_hour;
    dt.time.minutes = (int8_t)t.tm_min;
    dt.time.seconds = (int8_t)t.tm_sec;
    M5.Rtc.setDateTime(dt);
}

void clock_format(char *buf, size_t n, bool seconds) {
    buf[0] = 0;
    if (!clock_valid()) return;
    struct tm t;
    clock_now(&t);
    if (seconds)
        snprintf(buf, n, "%04d-%02d-%02d %02d:%02d:%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
                 t.tm_hour, t.tm_min, t.tm_sec);
    else
        snprintf(buf, n, "%04d-%02d-%02d %02d:%02d", t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour,
                 t.tm_min);
}
