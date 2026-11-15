// Copies the installed app from the Tab5's flash onto the SD card as
// signal_finder.bin, so the card always holds the version that is running
// (ready to reinstall from the M5Launcher).
//
// Only call app_copy_to_sd() from the UI task: it uses the SD card.
#pragma once

#include <stddef.h>

// Writes /sd/signal_finder.bin and checks it. Nothing else on the card is
// touched. Returns true on success; `msg` gets one line either way.
bool app_copy_to_sd(char *msg, size_t n);

// Older copies of this app on the card, under names it used to have (for
// example ble-finder.bin). `names` gets them as "a, b". Returns how many.
size_t app_old_copies(char *names, size_t n);

// Deletes those older copies. Returns how many were removed.
size_t app_remove_old_copies();

// A tiny command reader on the USB serial port, so a PC can drive the app
// without anyone touching the screen (for updating the SD card, and for
// testing). Send a line of text:
//   sdcopy     copy the app to the SD card
//   radiofw    send the bundled Wi-Fi + Bluetooth firmware to the radio chip again
//   sdclean    delete older copies of the app from the SD card
//   tap X Y    act as if the screen was tapped at X, Y
//   key N      act as if a key was pressed on the Tab5 Keyboard (N: character code,
//              or 128-131 for up, down, left, right)
//   flick N    set the device list gliding, N pixels a second (negative: back up)
//   shot       send a half-size screenshot (rows of RGB565 hex between
//              "SHOT name w h" and "ENDSHOT")
// Requests are picked up by the UI task through usb_command_take().
void usb_commands_start();
enum class UsbCommand { None, CopyAppToSd, RemoveOldCopies, Tap, Flick, Key, Screenshot, RadioFirmware };
UsbCommand usb_command_take(int *x, int *y);
