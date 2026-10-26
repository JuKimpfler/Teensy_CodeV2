// XM125_Presence.cpp
// Beispiel-/Testskript für den Presence-Detector des Acconeer XM125.
// Erkennt Anwesenheit (schnelle & langsame Bewegung) im Bereich 300mm-2000mm.
//
// Ausführen: Datei nach src/main.cpp des Hauptprojekts kopieren (in src/ darf
// immer nur eine Datei mit setup()/loop() liegen) und normal bauen/hochladen:
//   pio run -e teensy40 -t upload
// Siehe lib/XM125/examples/README.md fuer alle Beispiele im Ueberblick.

#include <Arduino.h>
#include <Wire.h>
#include "XM125Presence.h"

static constexpr int8_t WAKE_PIN = -1;
static constexpr int8_t INT_PIN  = -1;

XM125Presence radar(XM125_I2C_ADDR_DEFAULT, WAKE_PIN, INT_PIN);

// Nach so vielen aufeinanderfolgenden Fehlschlaegen wird neu verbunden und
// konfiguriert. Sinnvoll z.B. bei floatendem WAKE_UP-Pin, wenn der Sensor
// zwischenzeitlich in einen unerreichbaren Zustand faellt.
static constexpr uint8_t MAX_CONSECUTIVE_FAILURES = 5;
uint8_t consecutiveFailures = 0;

bool setupSensor() {
  if (!radar.begin(Wire)) {
    Serial.println("FEHLER: Sensor nicht erreichbar!");
    return false;
  }

  radar.config.startMm = 300;
  radar.config.endMm   = 2000;

  Serial.println("Konfiguriere...");
  if (!radar.applyConfiguration()) {
    Serial.println("FEHLER: Konfiguration fehlgeschlagen!");
    uint32_t status = radar.getDetectorStatus();
    Serial.print("Detector Status: 0x"); Serial.println(status, HEX);
    return false;
  }

  Serial.println("Starte Detektor...");
  if (!radar.start()) {
    Serial.println("FEHLER: Start fehlgeschlagen!");
    return false;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Wire.begin();
  Wire.setClock(100000); // niedriger Takt, mehr Toleranz fuer Clock-Stretching des XM125

  Serial.println("XM125 Presence Detector Test");

  if (!setupSensor()) {
    Serial.println("-> Pruefe SDA/SCL/GND/3V3. WAKE_UP-Pin ist hier absichtlich unverkabelt");
    Serial.println("   (kein Pullup) -- solange nie ein Sleep-Kommando gesendet wird, ist das");
    Serial.println("   nicht die erwartete Fehlerursache, siehe Kommentar in XM125.h.");
    Serial.println("Versuche es trotzdem weiter in der Hauptschleife...");
  } else {
    Serial.println("Bereit. Starte Ueberwachungsschleife.");
  }
}

void loop() {
  if (!radar.update()) {
    Serial.println("Lesefehler.");
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

  if (radar.hasDetectorError()) {
    Serial.println("Detektor meldet Fehler — Neustart (stop/start) noetig.");
    radar.stop();
    radar.start();
    delay(200);
    return;
  }

  Serial.print("Anwesenheit: ");
  Serial.print(radar.isPresenceDetected() ? "JA " : "nein");
  Serial.print(" (sticky: "); Serial.print(radar.isPresenceDetectedSticky() ? "ja" : "nein"); Serial.print(")");

  if (radar.isPresenceDetected()) {
    Serial.print("  Distanz: "); Serial.print(radar.getPresenceDistanceMm()); Serial.print(" mm");
  }

  Serial.print("  IntraScore: "); Serial.print(radar.getIntraPresenceScore());
  Serial.print("  InterScore: "); Serial.println(radar.getInterPresenceScore());

  delay(100);
}
