# Zenith V rocket telemetry + onboard video flight computer (C / Arduino)

Two Arduino sketches:

- `flight_computer/` — runs on the Freenove ESP32-S3-CAM. Reads the
  BMP388 and NEO-M8N, tracks flight state, logs CSV + records video to
  the included SD card, sends LoRa telemetry. Camera capture runs on
  CPU core 0 as a separate FreeRTOS task so SD writes never stall the
  telemetry loop on core 1.
- `ground_station/` — runs on a second Arduino-capable board wired to
  your other SX1276 module. Receives telemetry, prints CSV over serial.

## Arduino IDE setup

1. Board: **ESP32S3 Dev Module** (per Freenove's own tutorial for this
   board).
2. Libraries to install via Library Manager:
   - `Adafruit BMP3XX Library` (+ its dependency `Adafruit BusIO`)
   - `TinyGPSPlus`
   - `LoRa` by Sandeep Mistry
   - `SD_MMC` and `esp_camera` ship with the ESP32 Arduino core — nothing
     to install.

## Before you flash — two things sourced from Freenove, not guessed here

**Camera pins.** `camera_capture.cpp` uses `#define CAMERA_MODEL_FREENOVE_ESP32S3_CAM`
and `#include "camera_pins.h"` rather than hardcoded GPIO numbers —
getting the camera pin map wrong is the most common way to hang one of
these boards on boot. Freenove's own C tutorial (downloadable from their
product page, fnk0085) has the tested pin definitions for this exact
board under their Chapter 7 "Camera Web Server" example — pull
`camera_pins.h` and confirm the `CAMERA_MODEL_*` define from there if
your installed core version doesn't already recognize
`CAMERA_MODEL_FREENOVE_ESP32S3_CAM`.

**SD_MMC pins.** This board uses SDMMC 1-bit mode via the built-in
`SD_MMC.h` library (confirmed in Freenove's docs). `flight_computer.ino`
calls `SD_MMC.begin("/sdcard", true)` and relies on default pin
detection; if that fails on your board, copy the exact `SD_MMC.setPins(...)`
call from Freenove's Chapter 4 "SDMMC Test" example, which is tested
against this specific board revision.

## Other things to set before flying

- `config.h` pin numbers for the barometer, GPS, LoRa, and battery ADC
  are placeholders — set them to your actual wiring.
- The barometer calibrates to ground level at boot (`baro.calibrate()`
  in `setup()`) — power on while the rocket is upright on the pad, not
  before you walk out to the field.
- No pyro/deployment channels in this build — the state machine drives
  telemetry rate and video start/stop only, not recovery.

## Video format

Frames save as `frame_00001.jpg`, `frame_00002.jpg`, ... inside a new
`flight_NNN` folder each time recording starts, rather than one big
video container. That way a mid-flight SD write error or power loss
only costs the last frame or two, not the whole clip. After recovery,
stitch into a video with ffmpeg:

```
ffmpeg -framerate 10 -i frame_%05d.jpg -pix_fmt yuv420p flight.mp4
```

## Why no live video over LoRa

LoRa's data rate (roughly hundreds of bits/sec to a few kbps depending
on spreading factor) can't carry video — that's why telemetry (altitude,
GPS, battery) goes over LoRa while video is recorded onboard and
recovered after the flight instead. That's reliable regardless of
altitude or line-of-sight, which a live downlink wouldn't be.

## Wire format

`telemetry.h` is the single source of truth for the packet layout and
is copied identically into both sketches. If you change it, copy the
updated file to both sides so encode/decode don't drift out of sync.
