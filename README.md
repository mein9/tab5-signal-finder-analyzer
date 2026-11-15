# Signal Finder & Analyzer for the M5Stack Tab5

A "hot / cold" finder for Bluetooth and Wi-Fi devices, for the M5Stack Tab5
(ESP32-P4 with an ESP32-C6 radio chip). Pick a device from the list, then walk
toward it: the signal number, meter, beeps and colour all get stronger as you
get closer. Made for finding things like a dropped earbud, a tracker tag, or a
hidden access point.

## What it does

**Finding**

- **Device list** â€“ Bluetooth LE devices nearby, strongest first. Recognises
  AirPods and Beats (with model), Find My items (AirTags etc.), Tile, Galaxy
  SmartTag, Google Fast Pair earbuds, and Apple, Samsung and Windows devices.
- **Wi-Fi section** â€“ 2.4 GHz networks, found the same way. While you track
  one, the scanner stays on its channel for faster updates.
- **Hot / cold screen** â€“ big signal reading, warmer / colder trend, meter,
  30-second graph, and Geiger-style beeps that speed up as you close in.
- **Sweep radar** â€“ turn in a circle holding the Tab5; the motion sensor maps
  signal strength to direction and points at the strongest.
- **Colour view** â€“ the whole screen goes from blue (far) to red (close), with
  no sound, for quiet places.

**Keeping track**

- **Pin and rename** â€“ pin a device to the top of the list and give it your own
  name.
- **Left-behind alarm** â€“ alarms if a chosen device hasn't been heard for a
  minute.
- **Tracker alert** â€“ warns if an AirTag (separated from its owner), Tile or
  SmartTag stays near you for 10+ minutes while the Tab5 is being carried.
- **New-device watch** â€“ mark the surroundings as normal, then get an alert
  when a new Bluetooth device shows up.

**Tools**

- **Wi-Fi channel chart** â€“ networks per channel, and the least crowded of
  channels 1, 6 and 11.
- **Log to SD card** â€“ writes what it hears to `FINDLOG/LOGnnnnn.CSV` every
  10 seconds.
- Volume slider, scan on / off, and four screen orientations (landscape and
  portrait, each either way up).

## Limits worth knowing

- Signal strength is a rough guide, not a distance. Walls, bodies and
  reflections move it around; trust the trend over a few seconds.
- Find My items never broadcast their name â€“ only an anonymous, changing key â€“
  so they show as "Find My item". Nothing nearby can read the name.
- Phones, AirPods and Find My items near their owner change address every few
  minutes. AirPods are followed by model instead; a nickname on other rotating
  devices only lasts until the address changes.
- The Tab5's radio only hears 2.4 GHz Wi-Fi.
- The tracker alert uses the motion sensor to tell whether you are moving, so
  smooth travel (a car on a motorway) may not count as movement.
- The app only listens. It never connects to, pairs with, or sends anything to
  the devices it lists. The SD log stays on the card.

## Hardware

- M5Stack Tab5. Built and tested on a unit with ESP32-P4 chip revision v1.3
  and the ST7121 display panel.
- The ESP32-C6 radio chip must run ESP-Hosted **3.0.6 with Bluetooth enabled**.
  Some firmware (for example a Wi-Fi-only build) leaves Bluetooth off. If the
  app finds that, it offers to install a Wi-Fi + Bluetooth build of the same
  version over the internal link. That build is included as
  `main/c6_radio_fw.bin`; it is Espressif's ESP-Hosted co-processor example
  (`bleprph_wifi_coex/cp`, Apache-2.0) built for the Tab5's C6.

## Install

Take `signal_finder.bin` from the releases page (or build it), then either:

- copy it to the SD card and install it with the
  [M5Launcher](https://github.com/bmorcelli/Launcher), or
- flash it as the only app. This overwrites the boot loader, partition table
  and whatever app is installed:

```
python -m esptool --chip esp32p4 write_flash 0x2000 build/bootloader/bootloader.bin 0x8000 build/partition_table/partition-table.bin 0x10000 build/signal_finder.bin
```

The app needs about 2.5 MB, so the Launcher app slot must be at least that big.

## Build

Needs ESP-IDF v5.5.x.

```
idf.py set-target esp32p4
idf.py build
```

On Windows, `build.ps1` does the same and sets a short component cache path
(ESP-Hosted's nested file names overflow the 260-character path limit
otherwise).

Dependencies are fetched by the IDF component manager: `esp_hosted` 3.0.6,
`esp_wifi_remote`, `M5Unified` and `M5GFX`.

## How the screen stays fast

The Tab5's panel is natively portrait and its frame buffer is in external RAM.
Letting the graphics library rotate each landscape frame in software took about
650 ms. Instead, frames are drawn the way you hold the Tab5 into one of two buffers, and a
background task uses the ESP32-P4's PPA hardware to rotate the finished buffer
into the display and clear it for reuse while the next frame is drawn. That
gives about 17 frames a second.

## Source layout

| File | What it holds |
| --- | --- |
| `main/main.cpp` | Start-up |
| `main/ble_scan.cpp` | ESP-Hosted link, Bluetooth scanning, radio-chip update |
| `main/wifi_scan.cpp` | Wi-Fi scanning |
| `main/devices.cpp` | Device table, advertisement parsing, signal history |
| `main/saved.cpp` | Pinned / renamed / alarmed devices, stored in flash |
| `main/alerts.cpp` | Tracker, new-device and left-behind alerts |
| `main/sdlog.cpp` | SD card log |
| `main/ui.cpp` | All screens |
