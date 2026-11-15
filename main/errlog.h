// Error log.
//
// Keeps the app's most recent log lines in memory that survives a crash and
// restart, notices when the previous run ended in a crash, and can write an
// error report to the SD card for sending to the developer.
//
// The report holds app logs, crash details and settings. It deliberately has
// no names or addresses of nearby devices or networks.
#pragma once

#include <stddef.h>

// Call first thing in app_main, before anything logs.
void errlog_init();

// True if the previous run ended in a crash that hasn't been reported or dismissed.
bool errlog_crashed();
const char *errlog_crash_text();        // one line, e.g. "The app crashed (panic)"

// Writes a report to the SD card. `status` is extra text from the caller
// (current radio states and settings). Returns false if it couldn't; either
// way errlog_message() says what happened.
bool errlog_save(const char *status);
const char *errlog_message();

// Forget the crash (after saving, or when the user says "not now").
void errlog_dismiss();

// Demo mode only: pretend the last run crashed, so the pop-up can be shown.
void errlog_demo_crash();
