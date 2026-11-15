// Phone-band signal meter using an RTL-SDR dongle on the Tab5's USB-A port.
//
// Sweeps the frequencies that mobile phones TRANSMIT on (the "uplink" side of
// each cellular band) and reports how much radio energy is there. It measures
// strength only: nothing is demodulated or decoded, so it cannot tell whose
// phone it is or what is being sent.
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <vector>

#include "devices.h"    // Sample

enum class SdrState : uint8_t {
    Off,            // not asked for yet
    NoDongle,       // USB is up, waiting for a supported dongle
    Starting,       // dongle found, starting the stream
    Running,
    Failed,         // USB or driver error; see sdr_status_text()
};

struct SdrBand {
    const char *name;       // "850 MHz"
    const char *detail;     // "Band 5 / 26  |  824-849 MHz"
    uint32_t    lo_hz, hi_hz;
    bool        in_range;   // false if this dongle can't tune it
    float       level_db;   // strongest slice in the latest pass, dB (relative)
    float       floor_db;   // quietest this band has been (its noise floor)
    uint32_t    peak_hz;    // where the strongest slice was
    uint32_t    age_ms;     // since last measured
    bool        valid;      // measured at least once
};

// Powers the USB-A port. Call once from the main task before other tasks use I2C.
void sdr_power_on();

// Starts the background task. It stays idle until sdr_set_wanted(true).
void sdr_start();
void sdr_set_wanted(bool wanted);

// -1: sweep every band. Otherwise stay on that band for fast updates.
void sdr_set_focus(int band);
int  sdr_focus();

SdrState    sdr_state();
const char *sdr_status_text();      // one line for the screen
const char *sdr_dongle_name();      // "" until a dongle is recognised

size_t  sdr_band_count();
SdrBand sdr_band(size_t i);

// Signal history of the focused band since `since_ms` (oldest first).
void sdr_samples(std::vector<Sample> &out, uint32_t since_ms);
