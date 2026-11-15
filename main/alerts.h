// Background checks that raise an on-screen alert:
//  - a tracker that has stayed with you while you moved,
//  - a device that wasn't around when you marked the surroundings as normal,
//  - a device with a left-behind alarm that has gone quiet.
//
// Only call these from the UI task.
#pragma once

#include <stdint.h>
#include <stddef.h>

#include "devices.h"

enum class AlertType : uint8_t { Tracker, NewDevice, LeftBehind };

struct Alert {
    bool       active = false;
    AlertType  type = AlertType::Tracker;
    char       title[64] = {0};
    char       line[112] = {0};
    DeviceView dev{};           // what "Find it" should track
    uint32_t   since_ms = 0;
};

void alerts_init();                 // loads settings
void alerts_tick();                 // call every frame; does its work about once a second

const Alert &alerts_current();
void alerts_dismiss();
void alerts_ignore_device();        // dismiss, and never alert for this tracker again

bool alerts_tracker_enabled();
void alerts_set_tracker_enabled(bool on);

// New-device watch. Starting it marks everything heard right now as normal.
bool   alerts_watch_active();
void   alerts_watch_start();
void   alerts_watch_stop();
size_t alerts_watch_known();        // how many devices count as normal
