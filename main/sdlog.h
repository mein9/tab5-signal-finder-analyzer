// Logs what the scanner hears to a CSV file on the SD card.
//
// Only call these from the UI task.
#pragma once

#include <stdint.h>

bool        sdlog_active();
bool        sdlog_start();        // mounts the card and opens a new file
void        sdlog_stop();
void        sdlog_tick();         // call often; writes a batch every 10 s
uint32_t    sdlog_rows();
const char *sdlog_status();       // one line for the screen
