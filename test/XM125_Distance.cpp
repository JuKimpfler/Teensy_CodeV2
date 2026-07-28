// XM125_Distance.cpp
// Beispiel-/Testskript für den Distance-Detector des Acconeer XM125.
// Misst zyklisch Distanzen im Bereich 200mm-3000mm und gibt alle
// erkannten Peaks (Distanz + Signalstärke) über die serielle Schnittstelle aus.
//
// Ausführen mit:
//   pio test -e teensy40 -f XM125_Distance

#include <Arduino.h>
#include <Wire.h>
#include "XM125Distance.h"

static constexpr int8_t WAKE_PIN = -1;
static constexpr int8_t INT_PIN  = -1;

XM125Distance radar(XM125_I2C_ADDR_DEFAULT, WAKE_PIN, INT_PIN);

// Nach so vielen aufeinanderfolgenden Fehlschlaegen wird neu verbunden und
// konfiguriert. Sinnvoll z.B. bei floatendem WAKE_UP-Pin, wenn der Sensor
// zwischenzeitlich in einen unerreichbaren Zustand faellt.
static constexpr uint8_t MAX_CONSECUTIVE_FAILURES = 5;
uint8_t consecutiveFailures = 0;

bool setupSensor() {
  if (!radar.begin(Wire1)) {
    Serial.println("FEHLER: Sensor nicht erreichbar!");
    return false;
  }

  // Messbereich und Parameter anpassen
  radar.config.startMm = 200;
  radar.config.endMm   = 3000;
  radar.config.maxProfile = XM125D_PROFILE5;
  radar.config.thresholdMethod = XM125D_THRESHOLD_CFAR;
  radar.config.peakSorting = XM125D_SORT_STRONGEST;

  Serial.println("Konfiguriere & kalibriere (kann bis zu einigen Sekunden dauern)...");
  if (!radar.applyConfigAndCalibrate()) {
    Serial.println("FEHLER: Konfiguration/Kalibrierung fehlgeschlagen!");
    uint16_t failedReg = radar.getLastFailedConfigRegister();
    if (failedReg != 0xFFFF) {
      Serial.print("-> I2C-Schreibfehler bei Register 0x");
      Serial.println(failedReg, HEX);
    } else {
      Serial.println("-> Alle Konfigurationsregister wurden geschrieben, das APPLY-Kommando");
      Serial.println("   selbst ist fehlgeschlagen (Timeout oder DETECTOR_ERROR-Bit gesetzt).");
    }
    uint32_t status = radar.getDetectorStatus();
    Serial.print("Detector Status: 0x"); Serial.println(status, HEX);
    return false;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Wire1.begin();
  Wire1.setClock(400000);

  Serial.println("XM125 Distance Detector Test");

  if (!setupSensor()) {
    Serial.println("-> Pruefe SDA/SCL/GND/3V3. WAKE_UP-Pin ist hier absichtlich unverkabelt");
    Serial.println("   (kein Pullup) -- solange nie ein Sleep-Kommando gesendet wird, ist das");
    Serial.println("   nicht die erwartete Fehlerursache, siehe Kommentar in XM125.h.");
    Serial.println("Versuche es trotzdem weiter in der Hauptschleife...");
  } else {
    Serial.println("Bereit. Starte Messschleife.");
  }
}

void loop() {
  if (!radar.measure()) {
    Serial.println("Messfehler oder Timeout.");
    if (radar.getCalibrationNeeded()) {
      Serial.println("-> Kalibrierung wird benoetigt, rufe recalibrate() auf.");
      radar.recalibrate();
    }
    consecutiveFailures++;
    if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES) {
      Serial.println("-> Zu viele Fehlschlaege in Folge, versuche komplett neu zu verbinden...");
      consecutiveFailures = 0;
      setupSensor();
    }
    delay(200);
    return;
  }
  consecutiveFailures = 0;

  uint8_t n = radar.getNumDistances();
  Serial.print("Peaks: "); Serial.print(n);
  Serial.print("  Temp: "); Serial.print(radar.getTemperature());
  Serial.print(" C  NearStartEdge: "); Serial.println(radar.getNearStartEdge() ? "ja" : "nein");

  for (uint8_t i = 0; i < n; i++) {
    Serial.print("  Peak "); Serial.print(i);
    Serial.print(": "); Serial.print(radar.getPeakDistanceMm(i));
    Serial.print(" mm, Staerke="); Serial.println(radar.getPeakStrength(i));
  }

  delay(100);
}
