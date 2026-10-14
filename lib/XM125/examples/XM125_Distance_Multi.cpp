// XM125_Distance_Multi.cpp
// Testskript fuer 4 XM125 Distance-Detector-Sensoren an einem gemeinsamen
// I2C-Bus (Wire), unterschieden ueber 4 verschiedene I2C-Adressen.
//
// Verkabelung: alle 4 Sensoren an Wire (SDA/SCL), Adressen 0x51/0x52/0x53/0x54.
//
// Alle Sensoren messen GLEICHZEITIG: erst wird bei jedem startMeasurement()
// abgesetzt (je ein Registerschreibzugriff), danach reihum mit isMeasuring()
// nachgesehen. Die Messdauer faellt so einmal fuer alle vier Sensoren an statt
// viermal hintereinander, und gewartet wird an keiner Stelle. Genau dieses
// Schema fahrt systemlib::readRadar() im Roboter.
//
// Ausführen: Datei nach src/main.cpp des Hauptprojekts kopieren (in src/ darf
// immer nur eine Datei mit setup()/loop() liegen) und normal bauen/hochladen:
//   pio run -e teensy40 -t upload
// Siehe lib/XM125/examples/README.md fuer alle Beispiele im Ueberblick.
//
// Hinweis: Peak-Distanz ist aktuell nur ein unkalibrierter Rohwert (0-255
// "Counts"), siehe getPeakDistanceRaw()-Warnung in XM125Distance.h -- die
// Umrechnung in echte mm ist noch offen (siehe XM125_Distance.cpp Beispiel).

#include <Arduino.h>
#include <Wire.h>
#include "XM125Distance.h"
#include "PDS.h"

static constexpr uint8_t NUM_SENSORS = 4;
static constexpr uint8_t SENSOR_ADDRS[NUM_SENSORS] = {0x51, 0x52, 0x53, 0x54};
static constexpr int8_t WAKE_PIN = -1;
static constexpr int8_t INT_PIN = -1;

XM125Distance sensors[NUM_SENSORS] = {
    XM125Distance(SENSOR_ADDRS[0], WAKE_PIN, INT_PIN),
    XM125Distance(SENSOR_ADDRS[1], WAKE_PIN, INT_PIN),
    XM125Distance(SENSOR_ADDRS[2], WAKE_PIN, INT_PIN),
    XM125Distance(SENSOR_ADDRS[3], WAKE_PIN, INT_PIN),
};

bool sensorReady[NUM_SENSORS] = {};
bool measurePending[NUM_SENSORS] = {}; ///< Sensor nimmt an der laufenden Runde teil
uint8_t consecutiveFailures[NUM_SENSORS] = {};
static constexpr uint8_t MAX_CONSECUTIVE_FAILURES = 5;

PowerDebugger PDS;

bool setupSensor(uint8_t idx)
{
    XM125Distance &radar = sensors[idx];

    if (!radar.begin(Wire))
    {
        Serial.print("Sensor 0x");
        Serial.print(SENSOR_ADDRS[idx], HEX);
        Serial.println(": FEHLER, nicht erreichbar!");
        return false;
    }

    radar.setMeasureTimeout(3000); // Profile5/CFAR braucht laenger als der Default

    // Gleiche Konfiguration wie im Einzelsensor-Test (XM125_Distance.cpp)
    radar.config.startMm = 200;
    radar.config.endMm = 2500;
    radar.config.maxProfile = XM125D_PROFILE5;
    radar.config.thresholdMethod = XM125D_THRESHOLD_CFAR;
    radar.config.peakSorting = XM125D_SORT_CLOSEST;
    radar.config.reflectorShape = XM125D_REFLECTOR_PLANAR;

    if (!radar.applyConfigAndCalibrate())
    {
        Serial.print("Sensor 0x");
        Serial.print(SENSOR_ADDRS[idx], HEX);
        Serial.println(": FEHLER bei Konfiguration/Kalibrierung!");
        uint16_t failedReg = radar.getLastFailedConfigRegister();
        if (failedReg != 0xFFFF)
        {
            Serial.print("  -> I2C-Schreibfehler bei Register 0x");
            Serial.println(failedReg, HEX);
        }
        return false;
    }

    Serial.print("Sensor 0x");
    Serial.print(SENSOR_ADDRS[idx], HEX);
    Serial.println(": bereit.");
    return true;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000)
    {
    }

    Wire.begin();
    Wire.setClock(100000); // niedriger Takt, mehr Toleranz fuer Clock-Stretching des XM125

    PDS.init();

    Serial.println("XM125 Distance Detector Test - 4 Sensoren");
    Serial.println("==========================================");

    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        sensorReady[i] = setupSensor(i);
    }
}

/** Ergebnis eines fertig gemessenen Sensors ausgeben. */
void reportSensor(uint8_t i)
{
    Serial.print("[0x");
    Serial.print(SENSOR_ADDRS[i], HEX);
    Serial.print("] ");

    const XM125DistanceResult &res = sensors[i].getLastResult();
    if (!res.valid)
    {
        Serial.println("Messfehler oder Timeout.");
        consecutiveFailures[i]++;
        if (consecutiveFailures[i] >= MAX_CONSECUTIVE_FAILURES)
        {
            Serial.print("[0x");
            Serial.print(SENSOR_ADDRS[i], HEX);
            Serial.println("] Zu viele Fehlschlaege, versuche neu zu verbinden...");
            consecutiveFailures[i] = 0;
            sensorReady[i] = setupSensor(i);
        }
        return;
    }
    consecutiveFailures[i] = 0;

    Serial.print("Peaks=");
    Serial.print(res.numDistances);
    Serial.print(" Temp=");
    Serial.print(res.temperature);
    Serial.print(" (");
    Serial.print(res.durationMs);
    Serial.print(" ms)");
    for (uint8_t p = 0; p < res.numDistances; p++)
    {
        Serial.print("  Peak");
        Serial.print(p);
        Serial.print(": raw=");
        Serial.print(res.peakDistanceRaw[p]);
        Serial.print(" Staerke=");
        Serial.print(res.peakStrength[p]);
    }
    Serial.println();

    PDS.Channel(150 + i, res.numDistances > 0 ? res.peakDistanceRaw[0] : 0);
}

void loop()
{
    // 1. Nachsehen, welche Sensoren der laufenden Runde fertig sind. isMeasuring()
    //    kostet hoechstens einen Registerlesezugriff und wartet nie.
    bool roundRunning = false;
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        if (!measurePending[i])
            continue;

        if (sensors[i].isMeasuring())
        {
            roundRunning = true;
            continue;
        }

        measurePending[i] = false;
        reportSensor(i);
    }

    PDS.update();
    if (roundRunning)
        return; // noch nicht alle durch -- Zeit gehoert dem Rest

    // 2. Runde komplett: die naechste bei allen Sensoren gleichzeitig anstossen.
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
        if (!sensorReady[i])
        {
            // Regelmaessig erneut versuchen, falls ein Sensor beim Start fehlte
            sensorReady[i] = setupSensor(i);
            continue;
        }
        measurePending[i] = sensors[i].startMeasurement();
    }
}
