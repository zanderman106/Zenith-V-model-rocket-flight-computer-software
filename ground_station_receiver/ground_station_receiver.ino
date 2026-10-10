/*
 * GROUND STATION RECEIVER
 * Arduino Nano + CC1101
 *
 * Listens for telemetry packets broadcast by the rocket's flight computer
 * and prints them to the Serial Monitor. Packet format (plain text, CSV):
 *   STATE,alt_m,vel_mps,lat,lon,sats,maxalt_m
 *
 * REQUIRED LIBRARY (install via Library Manager):
 *   - SmartRC-CC1101-Driver-Lib  (ELECHOUSE_CC1101)
 *
 * IMPORTANT - VOLTAGE WARNING:
 * The CC1101 module is a 3.3V part. The Nano is a 5V board - its SPI pins
 * output 5V logic, which is above the CC1101's rated maximum. Many hobby
 * builds connect them directly and get away with it, but it is out of
 * spec and can shorten the module's life or damage it outright. The safe
 * options, in order of preference:
 *   1. Use a bidirectional logic-level shifter between Nano and CC1101
 *      for SCK/MOSI/CS/GDO0 (MISO going from 3.3V CC1101 to 5V Nano is
 *      the least risky direction, but shifting all four is safest).
 *   2. Use a 3.3V Arduino Nano variant (Nano 33 IoT, etc.) instead.
 *   3. At minimum, always power the CC1101 from a proper 3.3V supply
 *      (never 5V) - only the logic lines are the concern above.
 * This code assumes you've addressed the logic-level issue in hardware;
 * it does not compensate for it in software (it can't).
 */

#include <SPI.h>
#include <ELECHOUSE_CC1101_SRC_DRV.h>

// ---------- Pin assignments (Arduino Nano) ----------
// SPI pins are fixed on the Nano: SCK=D13, MISO=D12, MOSI=D11.
// CS and GDO0 can be any free digital pins - these are just a sensible choice.
#define CC1101_CS_PIN     10
#define CC1101_GDO0_PIN   2   // D2 supports interrupts if you want to extend this later

// ---------- Radio settings - MUST MATCH the flight computer exactly ----------
#define RADIO_FREQUENCY_MHZ   433.0f   // match config.h on the flight computer
#define RADIO_CHANNEL         0

// ---------- Status LED (built-in Nano LED, blinks on packet received) ----------
#define STATUS_LED_PIN    LED_BUILTIN

byte recvBuffer[128];  // sized with margin above the sender's packet[96] to avoid overflow

void setup() {
  Serial.begin(115200);
  pinMode(STATUS_LED_PIN, OUTPUT);

  Serial.println(F("=== Ground Station Receiver Boot ==="));

  ELECHOUSE_cc1101.setSpiPin(13, 12, 11, CC1101_CS_PIN); // SCK, MISO, MOSI, CS
  ELECHOUSE_cc1101.Init();
  ELECHOUSE_cc1101.setMHZ(RADIO_FREQUENCY_MHZ);
  ELECHOUSE_cc1101.setChannel(RADIO_CHANNEL);
  ELECHOUSE_cc1101.setCCMode(1);
  ELECHOUSE_cc1101.setModulation(0);   // 2-FSK, must match transmitter
  ELECHOUSE_cc1101.SetRx();

  Serial.println(F("CC1101 receiver ready. Listening for telemetry..."));
  Serial.println(F("lat,lon,sats"));
}

void loop() {
  if (ELECHOUSE_cc1101.CheckRxFifo(100)) {
    int len = ELECHOUSE_cc1101.ReceiveData(recvBuffer);
    if (len > 0 && len < (int)sizeof(recvBuffer)) {
      recvBuffer[len] = '\0';  // null-terminate so we can treat it as a string
      handlePacket((char*)recvBuffer);
      blinkOnce();
    }
  }
}

void handlePacket(char* packet) {
  // Just echo the raw packet - it's already a clean CSV row.
  // Add a local timestamp so you can see receive gaps/timing on the ground.
  Serial.print(millis());
  Serial.print(F(",rx,"));
  Serial.println(packet);
}

void blinkOnce() {
  digitalWrite(STATUS_LED_PIN, HIGH);
  delay(20);
  digitalWrite(STATUS_LED_PIN, LOW);
}
