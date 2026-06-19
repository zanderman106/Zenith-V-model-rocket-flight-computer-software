# 🚀 Model Rocket Flight Computer

**Hardware:** Freenove ESP32-CAM Dev Board + SHILLEHTEK Pre-Soldered BMP180

---

## What It Does

| Feature | Detail |
|---|---|
| 📹 Video | Continuous MJPEG recording to SD card |
| 📡 Altitude | BMP180 barometric altitude (AGL + ASL) |
| 🌡️ Temperature | °C from BMP180 |
| 💨 Pressure | Pascals from BMP180 |
| 💾 Storage | Everything saved to SD in a numbered `/flight_XXX/` folder |
| 🎯 State machine | IDLE → LAUNCH → COAST → DESCENT → LANDED |

---

## Wiring

```
BMP180 (SHILLEHTEK)    →    ESP32-CAM
────────────────────────────────────────
VCC                    →    3.3V
GND                    →    GND
SDA                    →    GPIO 13  ← IMPORTANT (see note)
SCL                    →    GPIO 12  ← IMPORTANT (see note)
```

> ⚠️ **Pin conflict note:** The Freenove ESP32-CAM uses GPIO 14 & 15 for the SD card
> (SDMMC 1-bit mode: CLK=14, CMD=15). Do **not** wire the BMP180 to those pins.
> Use **GPIO 12 (SCL) and GPIO 13 (SDA)** instead, and update the defines at the
> top of the sketch:
> ```cpp
> #define BMP_SDA  13
> #define BMP_SCL  12
> ```

---

## SD Card Output

Each flight creates a new folder automatically:

```
/sdcard/
  flight_001/
    telemetry.csv   ← altitude, temp, pressure at 5 Hz
    video.mjpeg     ← raw MJPEG stream
  flight_002/
    ...
```

### telemetry.csv columns

```
time_ms, state, altitude_m, agl_m, temperature_c, pressure_pa
```

### Converting video with FFmpeg

```bash
ffmpeg -f mjpeg -framerate 10 -i video.mjpeg -c copy output.avi
```

---

## Arduino IDE Setup

### 1. Install ESP32 Board Package
- File → Preferences → Additional Boards Manager URLs:
  ```
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
  ```
- Tools → Board → Boards Manager → search **esp32** → Install

### 2. Board Settings
| Setting | Value |
|---|---|
| Board | `AI Thinker ESP32-CAM` or `ESP32 Wrover Module` |
| Partition Scheme | `Huge APP (3MB No OTA)` |
| Upload Speed | `115200` |
| Flash Frequency | `80MHz` |

### 3. Install Libraries (Library Manager)
- `Adafruit BMP085 Library` (works for BMP180 too)
- `Adafruit Unified Sensor`

> The camera and SD_MMC libraries are included with the esp32 board package.

---

## Configuration (top of sketch)

```cpp
#define SEA_LEVEL_PRESSURE_PA  101325.0f  // Update from weather service before flight!
#define LOG_INTERVAL_MS        200         // Sensor log rate (200ms = 5 Hz)
#define LAUNCH_ALT_DELTA_M     2.0f        // Altitude rise to trigger launch detect
#define LAND_STABLE_SEC        10          // Seconds stable before "landed" declared
#define VIDEO_FRAME_SIZE       FRAMESIZE_VGA
#define VIDEO_JPEG_QUALITY     12          // 0=best, 63=worst
```

---

## Pre-Flight Checklist

- [ ] Format SD card as FAT32
- [ ] Update `SEA_LEVEL_PRESSURE_PA` from local weather (use QNH value × 100 for Pascals)
- [ ] Confirm serial monitor shows `All systems ready. Waiting for launch...`
- [ ] Verify BMP180 reads sensible temperature and pressure on serial monitor
- [ ] Secure wiring — vibration will disconnect loose jumpers
- [ ] Power from a 5V LiPo or BEC (ESP32-CAM draws ~200mA during recording)

---

## Flight State Machine

```
IDLE ──(altitude rises > 2m/sample)──► LAUNCH
         ↓
LAUNCH ──(altitude starts dropping)──► COAST
         ↓
COAST ──(rapid descent detected)──────► DESCENT
         ↓
DESCENT ──(stable for 10s)────────────► LANDED → files closed → IDLE
```

---

## Notes & Tips

- **Power**: The ESP32-CAM flash LED pin (GPIO 4) is connected to the SD card on some boards. Don't use the LED during flight — it may interfere with SD writes.
- **SD Speed**: Use a Class 10 / U1 or better card. Slow cards cause frame drops.
- **Altitude accuracy**: BMP180 has ±1 m resolution — fine for model rockets up to a few hundred metres.
- **Video quality**: `FRAMESIZE_VGA` (640×480) at quality 12 gives ~30–50 KB/frame. At 10 fps that's ~300–500 KB/s — well within SD write speed.
- **Post-flight analysis**: Import `telemetry.csv` into Google Sheets or Python/pandas to plot the altitude curve, calculate max velocity, etc.
