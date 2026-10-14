// XM125_Distance.cpp
// Beispiel-/Testskript für den Distance-Detector des Acconeer XM125.
// Misst zyklisch Distanzen im Bereich 200mm-2500mm und gibt alle
// erkannten Peaks (Distanz + Signalstärke) über die serielle Schnittstelle aus.
//
// Zeigt gleichzeitig das nicht blockierende Messschema der Library:
//   startMeasurement()  Messung starten, kehrt sofort zurueck
//   isMeasuring()       laeuft die Messung noch?
//   getLastResult()     letztes Messergebnis (Cache, kein I2C)
// loop() wartet damit nie auf den Sensor.
//
// Ausführen: Datei nach src/main.cpp des Hauptprojekts kopieren (in src/ darf
// immer nur eine Datei mit setup()/loop() liegen) und normal bauen/hochladen:
//   pio run -e teensy40 -t upload
// Siehe lib/XM125/examples/README.md fuer alle Beispiele im Ueberblick.

#include <Arduino.h>
#include <Wire.h>
#include "XM125Distance.h"

static constexpr int8_t WAKE_PIN = -1;
static constexpr int8_t INT_PIN = -1;

XM125Distance radar(0x55, WAKE_PIN, INT_PIN);

// Nach so vielen aufeinanderfolgenden Fehlschlaegen wird neu verbunden und
// konfiguriert. Sinnvoll z.B. bei floatendem WAKE_UP-Pin, wenn der Sensor
// zwischenzeitlich in einen unerreichbaren Zustand faellt.
static constexpr uint8_t MAX_CONSECUTIVE_FAILURES = 5;
uint8_t consecutiveFailures = 0;

/// true zwischen startMeasurement() und dem Abholen des Ergebnisses
bool measurementPending = false;

void printDiagnostics()
{
    uint32_t detStatus = radar.getDetectorStatus();
    uint32_t protoStatus = radar.getProtocolStatus();
    Serial.print("  Detector Status: 0x");
    Serial.print(detStatus, HEX);
    Serial.print("  (DETECTOR_CREATE_OK=");
    Serial.print((detStatus & (1UL << 3)) ? "1" : "0");
    Serial.print(" BUSY=");
    Serial.print(XM125Base::statusBusy(detStatus) ? "1" : "0");
    Serial.print(" DETECTOR_ERROR=");
    Serial.print(XM125Base::statusError(detStatus) ? "1" : "0");
    Serial.println(")");
    Serial.print("  Protocol Status: 0x");
    Serial.println(protoStatus, HEX);
    if (protoStatus != 0)
    {
        Serial.println("  -> Protocol Status != 0: Das Modul meldet einen Protokollfehler auf");
        Serial.println("     Register-/Kommandoebene (z.B. unbekanntes Register/Kommando fuer diese");
        Serial.println("     Firmware) -- starkes Indiz, dass die geflashte Applikation nicht (nur)");
        Serial.println("     dem Distance-Detector-Registerprotokoll folgt, das diese Library annimmt.");
    }
}

bool setupSensor()
{
    if (!radar.begin(Wire))
    {
        Serial.println("FEHLER: Sensor nicht erreichbar!");
        return false;
    }

    // Profile5/CFAR kann laenger brauchen als der Default-Timeout (1000 ms)
    radar.setMeasureTimeout(3000);

    // Messbereich und Parameter anpassen
    radar.config.startMm = 200;
    radar.config.endMm = 2500;
    radar.config.maxProfile = XM125D_PROFILE3;
    radar.config.thresholdMethod = XM125D_THRESHOLD_FIXED_STRENGTH;
    radar.config.fixedStrengthThreshold = -25;
    radar.config.peakSorting = XM125D_SORT_STRONGEST;
    radar.config.signalQuality = 35;
    radar.config.reflectorShape = XM125D_REFLECTOR_PLANAR;
    //radar.config.closeRangeLeakageCancellation = false;

    Serial.println("Konfiguriere & kalibriere (kann bis zu einigen Sekunden dauern)...");
    if (!radar.applyConfigAndCalibrate())
    {
        Serial.println("FEHLER: Konfiguration/Kalibrierung fehlgeschlagen!");
        uint16_t failedReg = radar.getLastFailedConfigRegister();
        if (failedReg != 0xFFFF)
        {
            Serial.print("-> I2C-Schreibfehler bei Register 0x");
            Serial.println(failedReg, HEX);
        }
        else
        {
            Serial.println("-> Alle Konfigurationsregister wurden geschrieben, das APPLY-Kommando");
            Serial.println("   selbst ist fehlgeschlagen (Timeout oder DETECTOR_ERROR-Bit gesetzt).");
        }
        printDiagnostics();
        return false;
    }

    Serial.println("Konfiguration/Kalibrierung erfolgreich laut Kommando-Ergebnis:");
    printDiagnostics();
    return true;
}

void setup()
{
    delay(2000);
    Serial.begin(115200);
    while (!Serial && millis() < 3000)
    {
    }

    Wire.begin();
    Wire.setClock(100000); // niedriger Takt, mehr Toleranz fuer Clock-Stretching des XM125

    Serial.println("XM125 Distance Detector Test");

    if (!setupSensor())
    {
        Serial.println("-> Pruefe SDA/SCL/GND/3V3. WAKE_UP-Pin ist hier absichtlich unverkabelt");
        Serial.println("   (kein Pullup) -- solange nie ein Sleep-Kommando gesendet wird, ist das");
        Serial.println("   nicht die erwartete Fehlerursache, siehe Kommentar in XM125.h.");
        Serial.println("Versuche es trotzdem weiter in der Hauptschleife...");
    }
    else
    {
        Serial.println("Bereit. Starte Messschleife.");
    }
}

/** Ergebnis der gerade abgeschlossenen Messung ausgeben bzw. Fehler behandeln. */
void handleResult()
{
    const XM125DistanceResult &res = radar.getLastResult();

    if (!res.valid)
    {
        Serial.print("Messfehler oder Timeout nach ");
        Serial.print(res.durationMs);
        Serial.println(" ms.");
        printDiagnostics();
        if (res.calibrationNeeded)
        {
            Serial.println("-> Kalibrierung wird benoetigt, rufe recalibrate() auf.");
            radar.recalibrate();
        }
        consecutiveFailures++;
        if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES)
        {
            Serial.println("-> Zu viele Fehlschlaege in Folge, versuche komplett neu zu verbinden...");
            consecutiveFailures = 0;
            setupSensor();
        }
        return;
    }
    consecutiveFailures = 0;

    Serial.print("Messung dauerte ");
    Serial.print(res.durationMs);
    Serial.println(" ms.");
    Serial.print("Peaks: ");
    Serial.print(res.numDistances);
    Serial.print("  Temp: ");
    Serial.print(res.temperature);
    Serial.print(" C  NearStartEdge: ");
    Serial.println(res.nearStartEdge ? "ja" : "nein");
    Serial.print("  (config.startMm=");
    Serial.print(radar.config.startMm);
    Serial.print(" endMm=");
    Serial.print(radar.config.endMm);
    Serial.println(")");

    // KALIBRIERUNG NOCH OFFEN: Peak-Distanz ist aktuell nur der rohe Byte-Wert
    // (0-255 "Counts"), noch NICHT in echte mm/cm umgerechnet. Bitte ein
    // einzelnes Objekt auf exakt bekannter Distanz platzieren (sonst nichts
    // in Reichweite) und den Raw-Wert hier gegen die bekannte Distanz notieren.
    for (uint8_t i = 0; i < res.numDistances; i++)
    {
        Serial.print("  Peak ");
        Serial.print(i);
        Serial.print(": raw=");
        Serial.print(res.peakDistanceRaw[i]);
        Serial.print(" counts, Staerke=");
        Serial.println(res.peakStrength[i]);
    }
}

void loop()
{
    // Es wird nie auf den Sensor gewartet: laeuft die Messung noch, ist loop()
    // sofort wieder draussen -- in einem echten Programm bleibt genau diese Zeit
    // fuer Regler, Motoren und die uebrigen Sensoren.
    if (digitalRead(10))
    {
        radar.applyConfigAndCalibrate();
        Serial.println("Config Applyed and Calibrated");
        delay(4000);
        Serial.println("Starting .........");
        delay(1000);
    }
    else
    {
        if (measurementPending)
        {
            if (radar.isMeasuring())
                return;

            measurementPending = false;
            handleResult();
        }

        measurementPending = radar.startMeasurement();
        if (!measurementPending)
        {
            Serial.println("Messkommando kam nicht an (I2C).");
            printDiagnostics();
            consecutiveFailures++;
            if (consecutiveFailures >= MAX_CONSECUTIVE_FAILURES)
            {
                consecutiveFailures = 0;
                setupSensor();
            }
        }
    }
}
