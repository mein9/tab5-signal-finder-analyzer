// M5Stack "Tab5 Keyboard" (the 70-key add-on the Tab5 docks into).
// It is an I2C device at address 0x6D on the Tab5's side connector
// (SDA = G0, SCL = G1). This reads it in its HID mode, where each event is a
// modifier byte plus a USB HID key code, and turns that into characters.
#pragma once

// Keys that are not plain characters. Printable keys are their ASCII code.
enum : int {
    K_BS = 8,
    K_TAB = 9,
    K_ENTER = 13,
    K_ESC = 27,
    K_UP = 0x80,
    K_DOWN,
    K_LEFT,
    K_RIGHT,
    K_DEL,
};

void kbd_start();       // sets up the bus; the keyboard may be attached later
void kbd_poll();        // call often, from one task only (it does the I2C)
bool kbd_present();
int  kbd_take();        // next key press (held keys repeat), or 0
