// XM125_Distance_Multi.cpp
// Testskript fuer 4 XM125 Distance-Detector-Sensoren an einem gemeinsamen
// I2C-Bus (Wire), unterschieden ueber 4 verschiedene I2C-Adressen.
//
// Verkabelung: alle 4 Sensoren an Wire (SDA/SCL), Adressen 0x51/0x52/0x53/0x54.
//
// Ausfuehren mit:
//   pio test -e teensy40 -f XM125_Distance_Multi
//
// Hinweis: Peak-Distanz ist aktuell nur ein unkalibrierter Rohwert (0-255
// "Counts"), siehe getPeakDistanceRaw()-Warnung in XM125Distance.h -- die
// Umrechnung in echte mm ist noch offen (siehe XM125_Distance.cpp Test).

#include <Arduino.h>
#include <Wire.h>
#include "XM125Distance.h"

static constexpr uint8_t NUM_SENSORS = 4;
static constexpr uint8_t SENSOR_ADDRS[NUM_SENSORS] = { 0x51, 0x52, 0x53, 0x54 };
static constexpr int8_t  WAKE_PIN = -1;
static constexpr int8_t  INT_PIN  = -1;

XM125Distance sensors[NUM_SENSORS] = {
  XM125Distance(SENSOR_ADDRS[0], WAKE_PIN, INT_PIN),
  XM125Distance(SENSOR_ADDRS[1], WAKE_PIN, INT_PIN),
  XM125Distance(SENSOR_ADDRS[2], WAKE_PIN, INT_PIN),
  XM125Distance(SENSOR_ADDRS[3], WAKE_PIN, INT_PIN),
};

bool    sensorReady[NUM_SENSORS]      = {};
uint8_t consecutiveFailures[NUM_SENSORS] = {};
static constexpr uint8_t MAX_CONSECUTIVE_FAILURES = 5;

bool setupSensor(uint8_t idx) {
  XM125Distance& radar = sensors[idx];

  if (!radar.begin(Wire)) {
    Serial.print("Sensor 0x"); Serial.print(SENSOR_ADDRS[idx], HEX);
    Serial.println(": FEHLER, nicht erreichbar!");
    return false;
  }

  // Gleiche Konfiguration wie im Einzelsensor-Test (XM125_Distance.cpp)
  radar.config.startMm = 200;
  radar.config.endMm   = 2500;
  radar.config.maxProfile = XM125D_PROFILE5;
  radar.config.thresholdMethod = XM125D_THRESHOLD_CFAR;
  radar.config.peakSorting = XM125D_SORT_STRONGEST;

  if (!radar.applyConfigAndCalibrate()) {
    Serial.print("Sensor 0x"); Serial.print(SENSOR_ADDRS[idx], HEX);
    Serial.println(": FEHLER bei Konfiguration/Kalibrierung!");
    uint16_t failedReg = radar.getLastFailedConfigRegister();
    if (failedReg != 0xFFFF) {
      Serial.print("  -> I2C-Schreibfehler bei Register 0x");
      Serial.println(failedReg, HEX);
    }
    return false;
  }

  Serial.print("Sensor 0x"); Serial.print(SENSOR_ADDRS[idx], HEX);
  Serial.println(": bereit.");
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Wire.begin();
  Wire.setClock(100000); // niedriger Takt, mehr Toleranz fuer Clock-Stretching des XM125

  Serial.println("XM125 Distance Detector Test - 4 Sensoren");
  Serial.println("==========================================");

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sensorReady[i] = setupSensor(i);
  }
}

void loop() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (!sensorReady[i]) {
      // Regelmaessig erneut versuchen, falls ein Sensor beim Start fehlte
      sensorReady[i] = setupSensor(i);
      continue;
    }

    XM125Distance& radar = sensors[i];
    bool ok = radar.measure(3000);

    Serial.print("[0x"); Serial.print(SENSOR_ADDRS[i], HEX); Serial.print("] ");

    if (!ok) {
      Serial.println("Messfehler oder Timeout.");
      consecutiveFailures[i]++;
      if (consecutiveFailures[i] >= MAX_CONSECUTIVE_FAILURES) {
        Serial.print("[0x"); Serial.print(SENSOR_ADDRS[i], HEX);
        Serial.println("] Zu viele Fehlschlaege, versuche neu zu verbinden...");
        consecutiveFailures[i] = 0;
        sensorReady[i] = setupSensor(i);
      }
      continue;
    }
    consecutiveFailures[i] = 0;

    uint8_t n = radar.getNumDistances();
    Serial.print("Peaks="); Serial.print(n);
    Serial.print(" Temp="); Serial.print(radar.getTemperature());
    for (uint8_t p = 0; p < n; p++) {
      Serial.print("  Peak"); Serial.print(p);
      Serial.print(": raw="); Serial.print(radar.getPeakDistanceRaw(p));
      Serial.print(" Staerke="); Serial.print(radar.getPeakStrength(p));
    }
    Serial.println();
  }
}
