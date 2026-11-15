// Date and time. The Tab5 has a battery-backed clock chip; once set here it
// keeps time across restarts and is used to timestamp the SD log and alerts.
//
// Only call these from the UI task.
#pragma once

#include <stddef.h>
#include <time.h>

void clock_init();                              // loads the time from the clock chip
bool clock_valid();                             // false until the time has been set
void clock_now(struct tm *out);                 // current time, or a sensible starting point
void clock_set(const struct tm &t);             // stores to the clock chip
// "2026-11-15 14:02:09" (or without seconds). Empty string when not set.
void clock_format(char *buf, size_t n, bool seconds);
