/*
 * ============================================================
 *  MODEL ROCKET FLIGHT COMPUTER
 *  Hardware: Freenove ESP32-CAM Dev Board + SHILLEHTEK BMP180
 * ============================================================
 *
 *  Features:
 *    - Continuous video recording to SD card (MJPEG)
 *    - Altitude, temperature & pressure logging via BMP180
 *    - CSV telemetry log saved to SD card
 *    - Flight state detection (IDLE → LAUNCH → COAST → DESCENT → LANDED)
 *    - Each flight gets a unique numbered folder on the SD card
 *
 *  Wiring:
 *    BMP180  →  ESP32-CAM
 *    VCC     →  3.3V
 *    GND     →  GND
 *    SDA     →  GPIO 14  (soft I2C — the CAM board's I2C pins)
 *    SCL     →  GPIO 15
 *
 *  SD card is built into the Freenove ESP32-CAM board (uses SDMMC).
 *
 *  Libraries required (install via Arduino Library Manager):
 *    - Adafruit BMP085 Unified  (works for BMP180)
 *    - Adafruit Unified Sensor
 *    - esp32 board package (includes camera, SD_MMC, Wire)
 *
 *  Board settings in Arduino IDE:
 *    Board    : "ESP32 Wrover Module"  or  "AI Thinker ESP32-CAM"
 *    Partition: "Huge APP (3MB No OTA)"
 * ============================================================
 */

#include "esp_camera.h"
#include "SD_MMC.h"
#include "FS.h"
#include <Wire.h>
#include <Adafruit_BMP085.h>   // BMP085/BMP180 unified driver

// ── Pin Definitions ────────────────────────────────────────

// Camera pins for AI-Thinker / Freenove ESP32-CAM
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26   // Camera I2C — do NOT use for BMP180
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      38
#define CAM_PIN_D3      37
#define CAM_PIN_D2      36
#define CAM_PIN_D1      21
#define CAM_PIN_D0      19
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// Soft I2C pins for BMP180 (avoid camera & SD pins)
#define BMP_SDA  14
#define BMP_SCL  15

// ── Configuration ──────────────────────────────────────────

// Sea-level pressure for altitude reference (update before flight!)
// Standard atmosphere = 101325 Pa  |  Check weather service for accuracy
#define SEA_LEVEL_PRESSURE_PA  101325.0f

// Telemetry logging interval in milliseconds
#define LOG_INTERVAL_MS  200   // 5 Hz

// Launch detection threshold (m/s² equivalent in altitude rise over 2 samples)
// If altitude increases by > LAUNCH_ALT_DELTA_M within LOG_INTERVAL_MS → launch
#define LAUNCH_ALT_DELTA_M   2.0f   // 2 m rise per sample = ~10 m/s ascent

// Landing detection: altitude stable within LAND_ALT_BAND_M for LAND_STABLE_SEC
#define LAND_ALT_BAND_M     3.0f
#define LAND_STABLE_SEC    10

// Video frame size  (FRAMESIZE_VGA = 640×480 recommended for SD write speed)
#define VIDEO_FRAME_SIZE  FRAMESIZE_VGA

// JPEG quality 0–63, lower = better quality, larger file
#define VIDEO_JPEG_QUALITY  12

// ── Globals ────────────────────────────────────────────────

Adafruit_BMP085 bmp;       // BMP180 object (library supports both 085 & 180)

// Flight state machine
enum FlightState { IDLE, LAUNCH, COAST, DESCENT, LANDED };
FlightState flightState = IDLE;

// File handles
File videoFile;
File csvFile;

// Telemetry
float baseAltitude   = 0.0f;   // Ground-level altitude (calibrated on boot)
float lastAltitude   = 0.0f;
float peakAltitude   = 0.0f;

unsigned long lastLogTime    = 0;
unsigned long landedTimer    = 0;
unsigned long flightNumber   = 0;
char flightFolder[32];         // e.g. "/flight_001"

// MJPEG video bookkeeping
uint32_t frameCount  = 0;
uint32_t videoStart  = 0;

// ── Helper: MJPEG container ────────────────────────────────
// We write a simple Motion JPEG stream: each frame = 4-byte length + JPEG bytes.
// This can be remuxed to .avi offline with FFmpeg:
//   ffmpeg -f mjpeg -framerate 10 -i video.mjpeg output.avi

void writeMJPEGFrame(File &f, camera_fb_t *fb) {
  // Write frame length as 4-byte little-endian header, then JPEG data
  uint32_t len = fb->len;
  f.write((uint8_t*)&len, 4);
  f.write(fb->buf, fb->len);
}

// ── Helper: next flight number from SD ────────────────────
uint32_t getNextFlightNumber() {
  uint32_t n = 1;
  while (true) {
    char path[32];
    snprintf(path, sizeof(path), "/flight_%03lu", (unsigned long)n);
    if (!SD_MMC.exists(path)) return n;
    n++;
  }
}

// ── Camera Initialisation ──────────────────────────────────
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = CAM_PIN_D0;
  config.pin_d1       = CAM_PIN_D1;
  config.pin_d2       = CAM_PIN_D2;
  config.pin_d3       = CAM_PIN_D3;
  config.pin_d4       = CAM_PIN_D4;
  config.pin_d5       = CAM_PIN_D5;
  config.pin_d6       = CAM_PIN_D6;
  config.pin_d7       = CAM_PIN_D7;
  config.pin_xclk     = CAM_PIN_XCLK;
  config.pin_pclk     = CAM_PIN_PCLK;
  config.pin_vsync    = CAM_PIN_VSYNC;
  config.pin_href     = CAM_PIN_HREF;
  config.pin_sscb_sda = CAM_PIN_SIOD;
  config.pin_sscb_scl = CAM_PIN_SIOC;
  config.pin_pwdn     = CAM_PIN_PWDN;
  config.pin_reset    = CAM_PIN_RESET;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = VIDEO_FRAME_SIZE;
  config.jpeg_quality = VIDEO_JPEG_QUALITY;
  config.fb_count     = 2;   // double buffer for smoother capture
  config.grab_mode    = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init failed: 0x%x\n", err);
    return false;
  }

  // Tune sensor for rocket (high-speed outdoor)
  sensor_t *s = esp_camera_sensor_get();
  s->set_brightness(s, 0);
  s->set_saturation(s, 0);
  s->set_gainceiling(s, GAINCEILING_4X);
  s->set_exposure_ctrl(s, 1);   // auto exposure
  s->set_awb_gain(s, 1);        // auto white balance

  Serial.println("[CAM] Initialised OK");
  return true;
}

// ── SD Initialisation ──────────────────────────────────────
bool initSD() {
  // SD_MMC uses 1-bit mode on Freenove board (GPIO 2 = DATA0, 14 = CLK, 15 = CMD)
  // NOTE: GPIO 14 & 15 are shared with BMP180 soft-I2C here.
  // IMPORTANT: SD_MMC must be mounted BEFORE starting I2C on those pins —
  // or use 1-bit mode which only needs GPIO 2 (DATA0), 14 (CLK), 15 (CMD).
  // The Freenove board routes SD to: CLK=14, CMD=15, DATA0=2
  // Those overlap with our BMP180 SDA/SCL choice — see note below!
  //
  // *** WIRING ADJUSTMENT FOR SHARED PINS ***
  // If SD_MMC CLK=14, CMD=15 conflicts with BMP SDA=14, SCL=15:
  //   Option A (recommended): wire BMP SDA→GPIO 13, SCL→GPIO 12
  //                           and change BMP_SDA/BMP_SCL defines above.
  //   Option B: Use hardware I2C on GPIO 21 (SDA) & 22 (SCL) — but those
  //             are used by the camera VSYNC/PCLK, so Option A is safer.
  //
  // The code below uses 1-bit SD_MMC mode (default on this board).

  if (!SD_MMC.begin("/sdcard", true)) {  // true = 1-bit mode
    Serial.println("[SD]  Mount failed");
    return false;
  }
  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("[SD]  No card detected");
    return false;
  }
  Serial.printf("[SD]  Card size: %llu MB\n", SD_MMC.cardSize() / (1024 * 1024));
  return true;
}

// ── BMP180 Initialisation ──────────────────────────────────
bool initBMP() {
  Wire.begin(BMP_SDA, BMP_SCL);
  if (!bmp.begin()) {
    Serial.println("[BMP] Not found — check wiring");
    return false;
  }
  Serial.println("[BMP] BMP180 found OK");
  return true;
}

// ── Calibrate ground-level altitude ───────────────────────
void calibrateBaseAltitude() {
  Serial.println("[CAL] Calibrating ground altitude (10 samples)...");
  float sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += bmp.readAltitude(SEA_LEVEL_PRESSURE_PA);
    delay(100);
  }
  baseAltitude = sum / 10.0f;
  lastAltitude = baseAltitude;
  Serial.printf("[CAL] Base altitude: %.2f m\n", baseAltitude);
}

// ── Open new flight folder & files ────────────────────────
void openFlightFiles() {
  flightNumber = getNextFlightNumber();
  snprintf(flightFolder, sizeof(flightFolder), "/flight_%03lu", (unsigned long)flightNumber);
  SD_MMC.mkdir(flightFolder);

  // Open CSV telemetry log
  char csvPath[48];
  snprintf(csvPath, sizeof(csvPath), "%s/telemetry.csv", flightFolder);
  csvFile = SD_MMC.open(csvPath, FILE_WRITE);
  if (csvFile) {
    csvFile.println("time_ms,state,altitude_m,agl_m,temperature_c,pressure_pa");
    Serial.printf("[SD]  Telemetry log: %s\n", csvPath);
  }

  // Open MJPEG video file
  char vidPath[48];
  snprintf(vidPath, sizeof(vidPath), "%s/video.mjpeg", flightFolder);
  videoFile = SD_MMC.open(vidPath, FILE_WRITE);
  if (videoFile) {
    Serial.printf("[SD]  Video file: %s\n", vidPath);
  }

  frameCount = 0;
  videoStart = millis();
}

// ── State name helper ──────────────────────────────────────
const char* stateName(FlightState s) {
  switch(s) {
    case IDLE:     return "IDLE";
    case LAUNCH:   return "LAUNCH";
    case COAST:    return "COAST";
    case DESCENT:  return "DESCENT";
    case LANDED:   return "LANDED";
    default:       return "UNKNOWN";
  }
}

// ── State Machine Update ───────────────────────────────────
void updateFlightState(float agl) {
  float delta = agl - lastAltitude;

  switch (flightState) {
    case IDLE:
      if (delta >= LAUNCH_ALT_DELTA_M) {
        flightState = LAUNCH;
        peakAltitude = agl;
        Serial.println("[FSM] LAUNCH detected!");
        openFlightFiles();
      }
      break;

    case LAUNCH:
      if (agl > peakAltitude) peakAltitude = agl;
      if (delta < 0) {
        flightState = COAST;
        Serial.printf("[FSM] COAST — apogee ~%.1f m AGL\n", peakAltitude);
      }
      break;

    case COAST:
      if (agl > peakAltitude) peakAltitude = agl;
      if (delta < -LAUNCH_ALT_DELTA_M) {
        flightState = DESCENT;
        Serial.println("[FSM] DESCENT detected");
      }
      break;

    case DESCENT:
      if (fabsf(agl - lastAltitude) < LAND_ALT_BAND_M) {
        if (landedTimer == 0) landedTimer = millis();
        if ((millis() - landedTimer) > (LAND_STABLE_SEC * 1000UL)) {
          flightState = LANDED;
          Serial.printf("[FSM] LANDED — peak %.1f m AGL\n", peakAltitude);
          // Flush & close files
          if (csvFile)   { csvFile.flush();   csvFile.close();  }
          if (videoFile) { videoFile.flush(); videoFile.close();}
          Serial.println("[SD]  Files saved. Ready for next flight.");
          flightState = IDLE;
          baseAltitude = 0; // re-calibrate on next boot / reset
        }
      } else {
        landedTimer = 0;
      }
      break;

    default: break;
  }
}

// ══════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  Serial.println("\n====  ROCKET FLIGHT COMPUTER  ====");

  // Mount SD first (before I2C, so 1-bit SDMMC can claim its pins)
  if (!initSD()) {
    Serial.println("[FATAL] SD card required — halting");
    while (true) delay(1000);
  }

  // Initialise camera
  if (!initCamera()) {
    Serial.println("[FATAL] Camera required — halting");
    while (true) delay(1000);
  }

  // Initialise BMP180
  // *** If you changed BMP_SDA/BMP_SCL to 13/12, update the defines at top ***
  if (!initBMP()) {
    Serial.println("[FATAL] BMP180 required — halting");
    while (true) delay(1000);
  }

  // Calibrate ground altitude
  calibrateBaseAltitude();

  Serial.println("[SYS] All systems ready. Waiting for launch...");
}

// ══════════════════════════════════════════════════════════
//  MAIN LOOP
// ══════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── 1. Capture & write video frame (always, once flight starts) ──
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) {
    if (flightState != IDLE && flightState != LANDED && videoFile) {
      writeMJPEGFrame(videoFile, fb);
      frameCount++;

      // Flush every 50 frames to protect against power loss
      if (frameCount % 50 == 0) videoFile.flush();
    }
    esp_camera_fb_return(fb);
  }

  // ── 2. Read sensors & log at LOG_INTERVAL_MS ──────────────
  if (now - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = now;

    float pressurePa  = bmp.readPressure();          // Pascals
    float tempC       = bmp.readTemperature();        // °C
    float altitudeM   = bmp.readAltitude(SEA_LEVEL_PRESSURE_PA); // metres ASL
    float agl         = altitudeM - baseAltitude;     // metres AGL

    // Update flight state machine
    updateFlightState(agl);

    // Write telemetry CSV (only during active flight)
    if (flightState != IDLE && csvFile) {
      csvFile.printf("%lu,%s,%.2f,%.2f,%.2f,%.2f\n",
        now,
        stateName(flightState),
        altitudeM,
        agl,
        tempC,
        pressurePa
      );
    }

    // Serial monitor heartbeat
    Serial.printf("[%7lu] %s | Alt: %6.1f m ASL | AGL: %5.1f m | Temp: %4.1f°C | Press: %.0f Pa\n",
      now,
      stateName(flightState),
      altitudeM,
      agl,
      tempC,
      pressurePa
    );

    lastAltitude = agl;
  }
}
