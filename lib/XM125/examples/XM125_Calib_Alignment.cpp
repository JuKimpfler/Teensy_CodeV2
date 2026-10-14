// XM125_Calib_Alignment.cpp
// ── Kalibriermethode 2: MECHANISCHE AUSRICHTUNG auf die Wand ──────────────────
//
// Beantwortet: Sieht der Sensor die Wand ueberhaupt, und steht er so, dass die
// Reflexion maximal zurueckkommt?
//
// Warum eigenes Skript: Eine glatte Wand reflektiert spiegelnd. Schon 10-15 Grad
// Schraeglage lenken die Reflexion am Sensor vorbei -- dann hilft auch die beste
// Empfindlichkeitseinstellung nicht mehr. Deshalb erst ausrichten, dann die
// Empfindlichkeit abstimmen (XM125_Calib_Sensitivity.cpp).
//
// Reihenfolge der Kalibrierskripte:
//   1. XM125_Calib_Module.cpp       Modul kalibriert ueberhaupt?
//   2. XM125_Calib_Alignment.cpp    <- dieses hier
//   3. XM125_Calib_Sensitivity.cpp  signalQuality / thresholdSensitivity finden
//   4. XM125_Calib_Crosstalk.cpp    Uebersprechen beim gleichzeitigen Messen
//
// Ausführen: Datei nach src/main.cpp des Hauptprojekts kopieren (in src/ darf
// immer nur eine Datei mit setup()/loop() liegen) und normal bauen/hochladen:
//   pio run -e teensy40 -t upload
// Siehe lib/XM125/examples/README.md fuer alle Beispiele im Ueberblick.
//
// Bedienung beim Ausrichten:
//   'm' starten, Sensor bzw. Roboter langsam drehen/neigen und dabei die
//   Staerke beobachten. Der Balken ist auf den bisher groessten Wert skaliert,
//   "max" haelt das Maximum fest -- Ziel ist die Stellung, in der die aktuelle
//   Staerke wieder nahe an "max" liegt. Mit 'z' das Maximum zuruecksetzen,
//   wenn ein neuer Abstand oder ein neuer Sensor gemessen wird.
//
// Messaufbau:
//   - Glatte, ebene Holzflaeche (Sperrholz/OSB/Tuer, mind. ca. 60x60 cm),
//     Sensormitte auf Wandmitte, Abstand ab Sensor-Vorderkante gemessen.
//   - Ausrichten am besten auf dem GROESSTEN Nutzabstand (2,5 m): dort ist die
//     Reserve am kleinsten, naeher wird es nur besser.
//   - Sonst nichts im Strahlkegel (Tischkanten, Stuhlbeine, Personen erzeugen
//     eigene Peaks) und Sensor nicht in der Hand halten.

#include <Arduino.h>
#include <Wire.h>
#include <params.h>
#include "XM125Distance.h"

static constexpr uint32_t MEASURE_TIMEOUT_MS = RADAR_MEASURE_TIMEOUT_MS;
static constexpr uint8_t BURST_SIZE = 20; ///< Messungen fuer die Bewertung ('q')
static constexpr uint8_t BAR_WIDTH = 24;  ///< Zeichen des Staerkebalkens

XM125Distance sensors[RADAR_NUM_SENSORS] = {
    XM125Distance(RADAR_SENSOR_ADDRS[0], RADAR_WAKE_PIN, RADAR_INT_PIN),
    XM125Distance(RADAR_SENSOR_ADDRS[1], RADAR_WAKE_PIN, RADAR_INT_PIN),
    XM125Distance(RADAR_SENSOR_ADDRS[2], RADAR_WAKE_PIN, RADAR_INT_PIN),
    XM125Distance(RADAR_SENSOR_ADDRS[3], RADAR_WAKE_PIN, RADAR_INT_PIN),
};

/// Sensorbezeichnungen wie RadarPos in RadarArray.h (Index -> Einbauposition)
static const char *SENSOR_NAMES[RADAR_NUM_SENSORS] = {"Vo", "Re", "Hi", "Li"};

uint8_t activeSensor = 0;
bool sensorReady[RADAR_NUM_SENSORS] = {};

bool monitorRunning = false;
int32_t bestStrength = 0; ///< bestes Echo seit dem letzten 'z'
uint16_t seenCount = 0;
uint16_t totalCount = 0;

// ── Kleine Helfer ─────────────────────────────────────────────────────────────

static void sortI32(int32_t *a, uint8_t n)
{
    for (uint8_t i = 1; i < n; i++)
    {
        int32_t key = a[i];
        int16_t j = i - 1;
        while (j >= 0 && a[j] > key)
        {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = key;
    }
}

/** Balken, skaliert auf das bisher groesste Echo (bestStrength). */
static void printBar(int32_t strength)
{
    uint8_t filled = 0;
    if (bestStrength > 0 && strength > 0)
    {
        int32_t scaled = (int32_t)BAR_WIDTH * strength / bestStrength;
        if (scaled < 0)
            scaled = 0;
        if (scaled > BAR_WIDTH)
            scaled = BAR_WIDTH;
        filled = (uint8_t)scaled;
    }
    Serial.print("[");
    for (uint8_t i = 0; i < BAR_WIDTH; i++)
        Serial.print(i < filled ? '#' : '.');
    Serial.print("]");
}

// ── Sensorsteuerung ───────────────────────────────────────────────────────────

/** Projektkonfiguration aus params.h -- identisch zu systemlib::setupRadarSens(). */
static bool applyProjectConfig(uint8_t idx)
{
    XM125Distance &radar = sensors[idx];

    radar.config.startMm = RADAR_START_MM;
    radar.config.endMm = RADAR_END_MM;
    radar.config.maxProfile = XM125D_PROFILE5;
    radar.config.thresholdMethod = XM125D_THRESHOLD_CFAR;
    radar.config.thresholdSensitivity = RADAR_THRESHOLD_SENSITIVITY;
    radar.config.signalQuality = RADAR_SIGNAL_QUALITY;
    radar.config.peakSorting = XM125D_SORT_CLOSEST;
    radar.config.reflectorShape = XM125D_REFLECTOR_PLANAR;
    radar.config.closeRangeLeakageCancellation = true;

    if (!radar.applyConfigAndCalibrate())
    {
        Serial.print("  FEHLER: Kalibrierung fehlgeschlagen (Sensor 0x");
        Serial.print(RADAR_SENSOR_ADDRS[idx], HEX);
        Serial.println(").");
        uint16_t failedReg = radar.getLastFailedConfigRegister();
        if (failedReg != 0xFFFF)
        {
            Serial.print("  -> I2C-Schreibfehler bei Register 0x");
            Serial.println(failedReg, HEX);
        }
        Serial.println("  -> Details klaert XM125_Calib_Module.cpp (Kalibriermethode 1).");
        return false;
    }
    return true;
}

static bool connectSensor(uint8_t idx)
{
    sensorReady[idx] = false;
    if (!sensors[idx].begin(I2C_RADAR))
    {
        Serial.print("Sensor 0x");
        Serial.print(RADAR_SENSOR_ADDRS[idx], HEX);
        Serial.println(": nicht erreichbar (SDA/SCL/3V3/GND pruefen).");
        return false;
    }
    sensorReady[idx] = applyProjectConfig(idx);
    return sensorReady[idx];
}

// ── Modi ──────────────────────────────────────────────────────────────────────

/** Live-Anzeige zum Drehen/Neigen des Sensors. */
static void monitorTick()
{
    XM125Distance &radar = sensors[activeSensor];

    bool ok = radar.measureBlocking(MEASURE_TIMEOUT_MS);
    totalCount++;

    Serial.print(SENSOR_NAMES[activeSensor]);
    Serial.print(" 0x");
    Serial.print(RADAR_SENSOR_ADDRS[activeSensor], HEX);
    Serial.print("  ");

    if (!ok)
    {
        Serial.println("Messfehler/Timeout");
        if (radar.getCalibrationNeeded())
            radar.recalibrate();
        return;
    }

    uint8_t peaks = radar.getNumDistances();
    if (peaks == 0)
    {
        Serial.print("kein Echo   ");
        printBar(0);
        Serial.print("  Treffer ");
        Serial.print(seenCount);
        Serial.print("/");
        Serial.println(totalCount);
        return;
    }

    seenCount++;
    int32_t strength = radar.getPeakStrength(0);
    if (strength > bestStrength)
        bestStrength = strength;

    printBar(strength);
    Serial.print(" S=");
    Serial.print(strength);
    Serial.print(" max=");
    Serial.print(bestStrength);
    Serial.print("  raw=");
    Serial.print(radar.getPeakDistanceRaw(0));
    Serial.print(" Peaks=");
    Serial.print(peaks);
    if (radar.getNearStartEdge())
        Serial.print(" NAH-KANTE");
    Serial.print("  Treffer ");
    Serial.print(seenCount);
    Serial.print("/");
    Serial.print(totalCount);
    Serial.println();
}

/** Alle Sensoren einmal messen -- zeigt auf einen Blick, welcher blind ist. */
static void modeRoundRobin()
{
    Serial.println();
    Serial.println("=== Alle Sensoren im Vergleich ====================================");
    Serial.println(" Sensor      Peaks  Rohwert  Staerke");
    Serial.println(" ----------  -----  -------  -------");

    for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
    {
        Serial.print("  ");
        Serial.print(SENSOR_NAMES[i]);
        Serial.print(" 0x");
        Serial.print(RADAR_SENSOR_ADDRS[i], HEX);
        Serial.print("   ");

        if (!sensorReady[i])
        {
            sensorReady[i] = connectSensor(i);
            if (!sensorReady[i])
            {
                Serial.println("  -- nicht bereit --");
                continue;
            }
        }

        XM125Distance &radar = sensors[i];
        if (!radar.measureBlocking(MEASURE_TIMEOUT_MS))
        {
            Serial.println("  Messfehler/Timeout");
            continue;
        }
        uint8_t peaks = radar.getNumDistances();
        Serial.print("   ");
        Serial.print(peaks);
        Serial.print("      ");
        Serial.print(radar.getPeakDistanceRaw(0));
        Serial.print("      ");
        Serial.println(radar.getPeakStrength(0));
    }
    Serial.println();
    Serial.println("  Jeder Sensor blickt in eine andere Richtung -- ein Vergleich der");
    Serial.println("  Staerken ist nur sinnvoll, wenn vor jedem Sensor die gleiche Wand");
    Serial.println("  im gleichen Abstand steht (Roboter mittig zwischen den Banden).");
    Serial.println("===================================================================");
}

/** Ausrichtung bewerten: Trefferquote und Staerkestreuung ueber BURST_SIZE Messungen. */
static void modeQuality()
{
    Serial.println();
    Serial.println("=== Bewertung der aktuellen Ausrichtung ===========================");
    Serial.print("Sensor ");
    Serial.print(SENSOR_NAMES[activeSensor]);
    Serial.print(" 0x");
    Serial.print(RADAR_SENSOR_ADDRS[activeSensor], HEX);
    Serial.print(", ");
    Serial.print(BURST_SIZE);
    Serial.println(" Messungen. Jetzt nichts bewegen.");

    XM125Distance &radar = sensors[activeSensor];
    int32_t strengths[BURST_SIZE];
    uint8_t detections = 0, errors = 0, multiPeak = 0, nearEdge = 0;
    uint32_t totalMs = 0;

    for (uint8_t i = 0; i < BURST_SIZE; i++)
    {
        uint32_t t0 = millis();
        bool ok = radar.measureBlocking(MEASURE_TIMEOUT_MS);
        totalMs += millis() - t0;

        if (!ok)
        {
            errors++;
            if (radar.getCalibrationNeeded())
                radar.recalibrate();
            continue;
        }
        if (radar.getNumDistances() == 0)
            continue;
        if (radar.getNumDistances() > 1)
            multiPeak++;
        if (radar.getNearStartEdge())
            nearEdge++;

        strengths[detections++] = radar.getPeakStrength(0);
    }

    Serial.println();
    Serial.print("  erkannt:        ");
    Serial.print(detections);
    Serial.print("/");
    Serial.println(BURST_SIZE);
    Serial.print("  Messfehler:     ");
    Serial.println(errors);
    Serial.print("  Messdauer:      ");
    Serial.print(totalMs / BURST_SIZE);
    Serial.println(" ms im Mittel");

    if (detections == 0)
    {
        Serial.println();
        Serial.println("  Kein einziges Echo. In dieser Reihenfolge pruefen:");
        Serial.println("   - Wand senkrecht getroffen? (mit 'm' langsam drehen und Staerke beobachten)");
        Serial.println("   - Wandflaeche gross genug und mittig vor dem Sensor?");
        Serial.print("   - Abstand innerhalb ");
        Serial.print(RADAR_START_MM);
        Serial.print("-");
        Serial.print(RADAR_END_MM);
        Serial.println(" mm?");
        Serial.println("   - Gehaeuse/Kabelbaum direkt vor der Antenne?");
        Serial.println("  Erst danach lohnt der Empfindlichkeits-Sweep (XM125_Calib_Sensitivity).");
        Serial.println("===================================================================");
        return;
    }

    sortI32(strengths, detections);
    int32_t sMin = strengths[0];
    int32_t sMed = strengths[detections / 2];
    int32_t sMax = strengths[detections - 1];

    Serial.print("  Staerke min/median/max: ");
    Serial.print(sMin);
    Serial.print(" / ");
    Serial.print(sMed);
    Serial.print(" / ");
    Serial.println(sMax);
    Serial.print("  mehr als ein Peak:      ");
    Serial.print(multiPeak);
    Serial.print("/");
    Serial.println(BURST_SIZE);
    Serial.print("  NEAR_START_EDGE:        ");
    Serial.print(nearEdge);
    Serial.print("/");
    Serial.println(BURST_SIZE);

    // Streuung als Mass fuer die Stabilitaet der Ausrichtung: bei sauber
    // getroffener Wand liegen die Echos dicht beieinander.
    if (sMed > 0)
    {
        int32_t spreadPct = 100 * (sMax - sMin) / sMed;
        Serial.print("  Streuung:               ");
        Serial.print(spreadPct);
        Serial.println(" % vom Median");
    }
    if (bestStrength > 0)
    {
        Serial.print("  bestes Echo seit 'z':   ");
        Serial.println(bestStrength);
        if (sMed < bestStrength / 2)
        {
            Serial.println("  -> Aktuell weniger als die Haelfte des besten Echos: der Sensor stand");
            Serial.println("     schon einmal deutlich guenstiger. Mit 'm' zurueckdrehen.");
        }
    }

    Serial.println();
    if (detections == BURST_SIZE && multiPeak == 0)
    {
        Serial.println("  -> Ausrichtung ist gut: jede Messung sieht genau die Wand.");
        Serial.println("     Weiter mit XM125_Calib_Sensitivity.cpp.");
    }
    else if (detections >= BURST_SIZE * 3 / 4)
    {
        Serial.println("  -> Brauchbar, aber nicht jede Messung trifft. Erst mechanisch");
        Serial.println("     nachjustieren ('m'), erst danach die Empfindlichkeit hochdrehen --");
        Serial.println("     eine hoehere Empfindlichkeit kaschiert eine schlechte Ausrichtung nur.");
    }
    else
    {
        Serial.println("  -> Zu unzuverlaessig. Sensor mit 'm' auf maximale Staerke ausrichten.");
    }
    if (multiPeak > 0)
    {
        Serial.println("  Hinweis: es wurden mehrere Peaks gesehen -- es steht noch etwas anderes");
        Serial.println("  im Strahlkegel. Fuer die weitere Kalibrierung freiraeumen.");
    }
    Serial.println("===================================================================");
}

// ── Bedienung ─────────────────────────────────────────────────────────────────

static void printHelp()
{
    Serial.println();
    Serial.println("=== XM125 Kalibrierung 2/4: mechanische Ausrichtung ===============");
    Serial.println(" a <0..3>  Sensor waehlen (0=Vo 0x51, 1=Li 0x52, 2=Hi 0x53, 3=Re 0x54)");
    Serial.println(" m         Live-Anzeige an/aus -- beim Drehen die Staerke maximieren");
    Serial.println(" q         Ausrichtung bewerten (Trefferquote + Staerkestreuung)");
    Serial.println(" l         alle vier Sensoren einmal messen (welcher ist blind?)");
    Serial.println(" z         Maximum und Trefferzaehler zuruecksetzen");
    Serial.println(" k         neu kalibrieren (nach Umbau/Verschieben des Sensors noetig)");
    Serial.println(" ?         diese Hilfe");
    Serial.println();
    Serial.print(" aktiv: ");
    Serial.print(SENSOR_NAMES[activeSensor]);
    Serial.print(" 0x");
    Serial.print(RADAR_SENSOR_ADDRS[activeSensor], HEX);
    Serial.print("  Bereich ");
    Serial.print(RADAR_START_MM);
    Serial.print("-");
    Serial.print(RADAR_END_MM);
    Serial.println(" mm (params.h)");
    Serial.println("===================================================================");
}

static void handleCommand(char *line)
{
    while (*line == ' ')
        line++;
    char cmd = *line;
    const char *arg = line + 1;

    switch (cmd)
    {
    case 'a':
    {
        int idx = atoi(arg);
        if (idx < 0 || idx >= (int)RADAR_NUM_SENSORS)
        {
            Serial.println("Sensorindex 0..3.");
            break;
        }
        activeSensor = (uint8_t)idx;
        bestStrength = 0;
        seenCount = 0;
        totalCount = 0;
        Serial.print("Sensor ");
        Serial.print(SENSOR_NAMES[activeSensor]);
        Serial.print(" 0x");
        Serial.print(RADAR_SENSOR_ADDRS[activeSensor], HEX);
        Serial.println(" gewaehlt, Maximum zurueckgesetzt.");
        if (!sensorReady[activeSensor])
            connectSensor(activeSensor);
        break;
    }
    case 'm':
        monitorRunning = !monitorRunning;
        Serial.println(monitorRunning ? "Live-Anzeige an (erneut 'm' beendet)." : "Live-Anzeige aus.");
        break;
    case 'q':
        monitorRunning = false;
        modeQuality();
        break;
    case 'l':
        monitorRunning = false;
        modeRoundRobin();
        break;
    case 'z':
        bestStrength = 0;
        seenCount = 0;
        totalCount = 0;
        Serial.println("Maximum und Trefferzaehler zurueckgesetzt.");
        break;
    case 'k':
        monitorRunning = false;
        Serial.println("Neu kalibrieren -- Umfeld frei halten, nichts bewegen ...");
        sensorReady[activeSensor] = applyProjectConfig(activeSensor);
        Serial.println(sensorReady[activeSensor] ? "  OK" : "  FEHLGESCHLAGEN");
        break;
    case '?':
    case 'h':
        printHelp();
        break;
    case '\0':
        break;
    default:
        Serial.println("Unbekannter Befehl, '?' fuer Hilfe.");
        break;
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000)
    {
    }

    I2C_RADAR.begin();
    I2C_RADAR.setClock(100000); // niedriger Takt, mehr Toleranz fuer Clock-Stretching des XM125

    Serial.println();
    Serial.println("XM125 Kalibrierung 2/4 -- mechanische Ausrichtung auf die Wand");

    for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
    {
        Serial.print("Sensor ");
        Serial.print(SENSOR_NAMES[i]);
        Serial.print(" 0x");
        Serial.print(RADAR_SENSOR_ADDRS[i], HEX);
        Serial.print(": ");
        Serial.println(connectSensor(i) ? "bereit" : "FEHLER");
    }

    printHelp();
}

void loop()
{
    static char line[32];
    static uint8_t len = 0;

    while (Serial.available())
    {
        char c = Serial.read();
        if (c == '\r')
            continue;
        if (c == '\n')
        {
            line[len] = '\0';
            handleCommand(line);
            len = 0;
        }
        else if (len < sizeof(line) - 1)
        {
            line[len++] = c;
        }
    }

    if (monitorRunning)
        monitorTick();
}
