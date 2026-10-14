// XM125_Calib_Module.cpp
// ── Kalibriermethode 1: MODUL-INTERNE KALIBRIERUNG (Sensor + Detektor) ────────
//
// Beantwortet: Laeuft die Kalibrierung im Modul sauber durch, wie lange dauert
// sie, und haelt sie im Dauerbetrieb (Temperaturdrift)?
// Entspricht Teil A in lib/XM125/doc/07-Kalibrierung.md.
//
// Diese Methode ist Voraussetzung fuer alle anderen: ohne gueltige Kalibrierung
// misst das Modul gar nicht. Empfohlene Reihenfolge der Kalibrierskripte:
//   1. XM125_Calib_Module.cpp       <- dieses hier (Modul kalibriert ueberhaupt?)
//   2. XM125_Calib_Alignment.cpp    Sensor mechanisch auf die Wand ausrichten
//   3. XM125_Calib_Sensitivity.cpp  signalQuality / thresholdSensitivity finden
//   4. XM125_Calib_Crosstalk.cpp    Uebersprechen beim gleichzeitigen Messen
//
// Ausführen: Datei nach src/main.cpp des Hauptprojekts kopieren (in src/ darf
// immer nur eine Datei mit setup()/loop() liegen) und normal bauen/hochladen:
//   pio run -e teensy40 -t upload
// Siehe lib/XM125/examples/README.md fuer alle Beispiele im Ueberblick.
//
// Messaufbau:
//   - Sensoren FINAL montiert (im Ring, hinter dem endgueltigen Gehaeuse).
//     Eine Kalibrierung "in der Hand" verfaelscht die Leakage-Kompensation.
//   - Waehrend jeder Kalibrierung bewegt sich nichts vor dem Sensor und der
//     Nahbereich ist frei -- die Kalibrierung nimmt die Umgebung als Referenz
//     auf und rechnet sie danach weg.
//   - Stabile Versorgung, Motoren aus.
//
// Konfiguriert wird mit den Projektwerten aus lib/include/params/params_radar.h, damit hier
// genau das kalibriert wird, was spaeter auch systemlib::setupRadarSens() setzt.

#include <Arduino.h>
#include <Wire.h>
#include <params.h>
#include "XM125Distance.h"

static constexpr uint32_t APPLY_TIMEOUT_MS = XM125_DEFAULT_APPLY_TIMEOUT_MS;
static constexpr uint32_t MEASURE_TIMEOUT_MS = RADAR_MEASURE_TIMEOUT_MS;

static constexpr uint8_t MAX_REPEATS = 20;            ///< Obergrenze fuer 'n <N>'
static constexpr uint32_t DRIFT_INTERVAL_MS = 5000;   ///< Abstand der Driftmessungen ('w')
static constexpr uint32_t DRIFT_RECAL_LOCK_MS = 2000; ///< Zeitsperre fuer recalibrate(), siehe Doku A.6

XM125Distance sensors[RADAR_NUM_SENSORS] = {
    XM125Distance(RADAR_SENSOR_ADDRS[0], RADAR_WAKE_PIN, RADAR_INT_PIN),
    XM125Distance(RADAR_SENSOR_ADDRS[1], RADAR_WAKE_PIN, RADAR_INT_PIN),
    XM125Distance(RADAR_SENSOR_ADDRS[2], RADAR_WAKE_PIN, RADAR_INT_PIN),
    XM125Distance(RADAR_SENSOR_ADDRS[3], RADAR_WAKE_PIN, RADAR_INT_PIN),
};

uint8_t activeSensor = 0;
bool sensorReady[RADAR_NUM_SENSORS] = {};

// Driftbeobachtung ('w')
bool driftRunning = false;
uint32_t driftLastMs = 0;
uint32_t driftStartMs = 0;
uint32_t driftLastRecal = 0;
int16_t driftFirstTemp = 0;
uint8_t driftFirstRaw = 0;
bool driftHaveRef = false;
uint16_t driftRecalCount = 0;

// ── Kleine Helfer ─────────────────────────────────────────────────────────────

static void printHex32(uint32_t v)
{
    Serial.print("0x");
    for (int8_t shift = 28; shift >= 0; shift -= 4)
    {
        Serial.print((int)((v >> shift) & 0xF), HEX);
    }
}

/** Alle kalibrierungsrelevanten Statusbits einzeln ausgeben (Doku A.8). */
static void printCalibStatus(uint32_t s)
{
    Serial.print("  DETECTOR_STATUS = ");
    printHex32(s);
    Serial.println();
    Serial.print("    RSS_REGISTER_OK      ");
    Serial.println((s & XM125D_STATUS_RSS_REGISTER_OK) ? "1" : "0");
    Serial.print("    CONFIG_CREATE_OK     ");
    Serial.println((s & XM125D_STATUS_CONFIG_CREATE_OK) ? "1" : "0");
    Serial.print("    SENSOR_CREATE_OK     ");
    Serial.println((s & XM125D_STATUS_SENSOR_CREATE_OK) ? "1" : "0");
    Serial.print("    DETECTOR_CREATE_OK   ");
    Serial.println((s & XM125D_STATUS_DETECTOR_CREATE_OK) ? "1" : "0");
    Serial.print("    CONFIG_APPLY_ERROR   ");
    Serial.println((s & XM125D_STATUS_CONFIG_APPLY_ERROR) ? "1" : "0");
    Serial.print("    SENSOR_CAL_ERROR     ");
    Serial.println((s & XM125D_STATUS_SENSOR_CALIBRATE_ERROR) ? "1" : "0");
    Serial.print("    DETECTOR_CAL_ERROR   ");
    Serial.println((s & XM125D_STATUS_DETECTOR_CALIBRATE_ERROR) ? "1" : "0");
    Serial.print("    DETECTOR_ERROR       ");
    Serial.println(XM125Base::statusError(s) ? "1" : "0");
    Serial.print("    BUSY                 ");
    Serial.println(XM125Base::statusBusy(s) ? "1" : "0");
}

/** Erklaert, welcher Teilschritt gescheitert ist, und was als erstes zu tun ist. */
static void explainFailure(XM125Distance &radar)
{
    uint16_t failedReg = radar.getLastFailedConfigRegister();
    if (failedReg != 0xFFFF)
    {
        Serial.print("  -> I2C-Schreibfehler beim Konfigurieren, Register 0x");
        Serial.println(failedReg, HEX);
        Serial.println("     Ursache liegt auf dem Bus (Verkabelung/Pullups/Adresse), nicht in der");
        Serial.println("     Kalibrierung selbst. Konfigurationswerte wurden nur teilweise gesetzt.");
        return;
    }

    uint32_t s = radar.getDetectorStatus();
    printCalibStatus(s);

    if (s & XM125D_STATUS_CONFIG_APPLY_ERROR)
    {
        Serial.println("  -> Konfiguration abgelehnt: Wertebereiche pruefen (endMm > startMm,");
        Serial.println("     signalQuality/thresholdSensitivity, siehe doc/04-Konfiguration.md).");
    }
    if (s & XM125D_STATUS_SENSOR_CALIBRATE_ERROR)
    {
        Serial.println("  -> Sensor-Kalibrierung fehlgeschlagen: Versorgung/Verkabelung pruefen,");
        Serial.println("     Temperatur ausserhalb der Spezifikation, sonst Power-Cycle.");
    }
    if (s & XM125D_STATUS_DETECTOR_CALIBRATE_ERROR)
    {
        Serial.println("  -> Detektor-Kalibrierung fehlgeschlagen: Umgebung waehrend der Kalibrierung");
        Serial.println("     (Bewegung, Objekt im Nahbereich). Testweise startMm erhoehen oder");
        Serial.println("     closeRangeLeakageCancellation abschalten.");
    }
    if (XM125Base::statusBusy(s))
    {
        Serial.println("  -> Modul ist noch BUSY: das Kommando lief in einen Timeout, laeuft im Modul");
        Serial.println("     aber weiter. Kurz warten, NICHT sofort das naechste Kommando senden.");
    }
    if (radar.getProtocolStatus() != 0)
    {
        Serial.print("  Protocol Status: ");
        printHex32(radar.getProtocolStatus());
        Serial.println();
        Serial.println("  -> != 0 heisst: Register-/Kommandoebene abgelehnt (evtl. andere Firmware).");
    }
}

// ── Kalibrierung ──────────────────────────────────────────────────────────────

/** Projektkonfiguration aus params.h setzen -- identisch zu systemlib::setupRadarSens(). */
static void writeProjectConfig(uint8_t idx)
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
}

/**
 * Ein Kalibrierkommando ausfuehren und Dauer + Ergebnis protokollieren.
 * @param what  0 = applyConfigAndCalibrate, 1 = calibrate, 2 = recalibrate
 */
static bool runCalibration(uint8_t idx, uint8_t what, bool verbose = true)
{
    XM125Distance &radar = sensors[idx];
    const char *name = (what == 0)   ? "applyConfigAndCalibrate()"
                       : (what == 1) ? "calibrate()"
                                     : "recalibrate()";

    if (verbose)
    {
        Serial.print("[0x");
        Serial.print(RADAR_SENSOR_ADDRS[idx], HEX);
        Serial.print("] ");
        Serial.print(name);
        Serial.println(" ...");
    }

    if (what == 0)
        writeProjectConfig(idx);

    uint32_t t0 = millis();
    bool ok = (what == 0)   ? radar.applyConfigAndCalibrate(APPLY_TIMEOUT_MS)
              : (what == 1) ? radar.calibrate(APPLY_TIMEOUT_MS)
                            : radar.recalibrate(APPLY_TIMEOUT_MS);
    uint32_t dt = millis() - t0;

    if (verbose)
    {
        Serial.print("  Dauer: ");
        Serial.print(dt);
        Serial.print(" ms  -> ");
        Serial.println(ok ? "OK" : "FEHLGESCHLAGEN");
        if (!ok)
        {
            explainFailure(radar);
        }
        else
        {
            // Erfolgsbits gegenpruefen: der Rueckgabewert prueft nur DETECTOR_ERROR
            uint32_t s = radar.getDetectorStatus();
            const uint32_t wanted = XM125D_STATUS_RSS_REGISTER_OK | XM125D_STATUS_CONFIG_CREATE_OK |
                                    XM125D_STATUS_SENSOR_CREATE_OK | XM125D_STATUS_DETECTOR_CREATE_OK;
            if ((s & wanted) != wanted)
            {
                Serial.println("  WARNUNG: nicht alle Create-Bits gesetzt, obwohl das Kommando OK meldet:");
                printCalibStatus(s);
            }
        }
    }
    return ok;
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

    // Vorzustand pruefen: laeuft im Modul noch ein Kommando, wuerde das naechste
    // Kommando in einen Timeout laufen (Doku A.10, Schritt 2).
    uint32_t st = sensors[idx].getDetectorStatus();
    if (XM125Base::statusBusy(st))
    {
        Serial.print("Sensor 0x");
        Serial.print(RADAR_SENSOR_ADDRS[idx], HEX);
        Serial.println(": noch BUSY, warte...");
        uint32_t t0 = millis();
        while (XM125Base::statusBusy(sensors[idx].getDetectorStatus()) && millis() - t0 < APPLY_TIMEOUT_MS)
        {
        }
    }

    sensorReady[idx] = runCalibration(idx, 0);
    return sensorReady[idx];
}

// ── Modi ──────────────────────────────────────────────────────────────────────

/** N Kalibrierungen hintereinander: Dauerstreuung und Fehlerquote (Doku A.7). */
static void modeRepeat(uint8_t repeats)
{
    if (repeats < 1)
        repeats = 1;
    if (repeats > MAX_REPEATS)
        repeats = MAX_REPEATS;

    Serial.println();
    Serial.println("=== Wiederholte Kalibrierung ======================================");
    Serial.print("Sensor 0x");
    Serial.print(RADAR_SENSOR_ADDRS[activeSensor], HEX);
    Serial.print(", ");
    Serial.print(repeats);
    Serial.println("x applyConfigAndCalibrate().");
    Serial.println("Waehrend des Laufs darf sich vor dem Sensor nichts bewegen.");
    Serial.println();

    uint32_t minMs = 0xFFFFFFFF, maxMs = 0, sumMs = 0;
    uint8_t fails = 0;

    for (uint8_t i = 0; i < repeats; i++)
    {
        writeProjectConfig(activeSensor);
        uint32_t t0 = millis();
        bool ok = sensors[activeSensor].applyConfigAndCalibrate(APPLY_TIMEOUT_MS);
        uint32_t dt = millis() - t0;

        sumMs += dt;
        if (dt < minMs)
            minMs = dt;
        if (dt > maxMs)
            maxMs = dt;
        if (!ok)
            fails++;

        Serial.print("  ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(dt);
        Serial.print(" ms  ");
        Serial.println(ok ? "OK" : "FEHLER");
        if (!ok)
            explainFailure(sensors[activeSensor]);

        delay(200); // dem Modul zwischen zwei Kommandos Luft lassen
    }

    Serial.println();
    Serial.print("  Dauer min/mittel/max: ");
    Serial.print(minMs);
    Serial.print(" / ");
    Serial.print(sumMs / repeats);
    Serial.print(" / ");
    Serial.print(maxMs);
    Serial.println(" ms");
    Serial.print("  Fehlschlaege: ");
    Serial.print(fails);
    Serial.print("/");
    Serial.println(repeats);
    Serial.println();
    Serial.print("  Timeout in params.h/XM125.h: ");
    Serial.print(APPLY_TIMEOUT_MS);
    Serial.println(" ms");
    if (maxMs > (APPLY_TIMEOUT_MS * 2) / 3)
    {
        Serial.println("  -> Die laengste Kalibrierung liegt nahe am Timeout. Timeout erhoehen oder");
        Serial.println("     signalQuality/endMm/maxProfile senken, sonst kippt das bei Erwaermung.");
    }
    else if (fails == 0)
    {
        Serial.println("  -> Reserve zum Timeout ist ausreichend, Kalibrierung reproduzierbar.");
    }
    Serial.println("===================================================================");
}

/** Startsequenz wie im Roboter: alle Sensoren NACHEINANDER kalibrieren (Doku A.11). */
static void modeBootSequence()
{
    Serial.println();
    Serial.println("=== Startsequenz (alle Sensoren nacheinander) =====================");
    Serial.println("Nacheinander, weil vier Module im selben Frequenzband senden: waehrend");
    Serial.println("einer die Rauschpegel-Schaetzung macht, darf kein Nachbar messen.");
    Serial.println();

    uint32_t total = 0;
    uint8_t okCount = 0;
    for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
    {
        uint32_t t0 = millis();
        bool ok = connectSensor(i);
        total += millis() - t0;
        if (ok)
            okCount++;
    }

    Serial.println();
    Serial.print("  bereit: ");
    Serial.print(okCount);
    Serial.print("/");
    Serial.println(RADAR_NUM_SENSORS);
    Serial.print("  Gesamtdauer der Startsequenz: ");
    Serial.print(total);
    Serial.println(" ms");
    Serial.println("  (so lange braucht systemlib::initialize() allein fuer das Radar)");
    Serial.println("===================================================================");
}

static void modeStatus()
{
    XM125Distance &radar = sensors[activeSensor];
    Serial.println();
    Serial.println("=== Status ========================================================");
    Serial.print("Sensor 0x");
    Serial.println(RADAR_SENSOR_ADDRS[activeSensor], HEX);
    Serial.print("  Version:        ");
    printHex32(radar.getVersionRaw());
    Serial.println();
    Serial.print("  MeasureCounter: ");
    Serial.println(radar.getMeasureCounter());
    Serial.print("  Protocol:       ");
    printHex32(radar.getProtocolStatus());
    Serial.println();
    printCalibStatus(radar.getDetectorStatus());
    Serial.println("===================================================================");
}

/**
 * Driftbeobachtung: zeigt, ob die einmal aufgenommene Kalibrierung im warmen
 * Zustand noch traegt. Ausgegeben werden Temperatur, CALIBRATION_NEEDED und der
 * Peak-Rohwert eines UNBEWEGTEN Objekts -- wandert der Rohwert bei konstantem
 * Abstand, ist das Drift (Doku A.6).
 */
static void driftTick()
{
    XM125Distance &radar = sensors[activeSensor];

    bool ok = radar.measureBlocking(MEASURE_TIMEOUT_MS);
    uint32_t tSec = (millis() - driftStartMs) / 1000;

    Serial.print("  t=");
    Serial.print(tSec);
    Serial.print(" s");

    if (!ok)
    {
        Serial.println("  Messfehler/Timeout");
    }
    else
    {
        uint8_t peaks = radar.getNumDistances();
        int16_t temp = radar.getTemperature();
        uint8_t raw = radar.getPeakDistanceRaw(0);

        if (!driftHaveRef && peaks > 0)
        {
            driftFirstTemp = temp;
            driftFirstRaw = raw;
            driftHaveRef = true;
        }

        Serial.print("  T=");
        Serial.print(temp);
        Serial.print(" C");
        Serial.print("  Peaks=");
        Serial.print(peaks);
        Serial.print("  raw=");
        Serial.print(raw);
        Serial.print("  S=");
        Serial.print(radar.getPeakStrength(0));
        if (driftHaveRef)
        {
            Serial.print("   dT=");
            Serial.print(temp - driftFirstTemp);
            Serial.print(" K  draw=");
            Serial.print((int)raw - (int)driftFirstRaw);
        }
        Serial.print("  CAL_NEEDED=");
        Serial.print(radar.getCalibrationNeeded() ? "1" : "0");
        Serial.println();
    }

    // Nachkalibrieren nur mit Zeitsperre -- recalibrate() blockiert und liefert
    // in dieser Zeit keine Messwerte (Doku A.6).
    if (radar.getCalibrationNeeded() && millis() - driftLastRecal > DRIFT_RECAL_LOCK_MS)
    {
        driftLastRecal = millis();
        driftRecalCount++;
        Serial.print("    -> recalibrate() (#");
        Serial.print(driftRecalCount);
        Serial.print(") ");
        uint32_t t0 = millis();
        bool rok = radar.recalibrate(APPLY_TIMEOUT_MS);
        Serial.print(millis() - t0);
        Serial.print(" ms ");
        Serial.println(rok ? "OK" : "FEHLER");
        if (!rok)
            explainFailure(radar);
    }
}

// ── Bedienung ─────────────────────────────────────────────────────────────────

static void printHelp()
{
    Serial.println();
    Serial.println("=== XM125 Kalibrierung 1/4: Modul-interne Kalibrierung ============");
    Serial.println(" a <0..3>  Sensor waehlen (0x51/0x52/0x53/0x54)");
    Serial.println(" k         applyConfigAndCalibrate() -- config schreiben + voll kalibrieren");
    Serial.println(" v         calibrate()   -- voll kalibrieren, config unveraendert");
    Serial.println(" r         recalibrate() -- kleine Nachkalibrierung (Temperaturdrift)");
    Serial.println(" n <N>     N Kalibrierungen: Dauer min/mittel/max + Fehlerquote");
    Serial.println(" b         Startsequenz: alle Sensoren nacheinander kalibrieren");
    Serial.println(" s         Status-/Fehlerbits des aktiven Sensors ausgeben");
    Serial.println(" w         Driftbeobachtung an/aus (Objekt unbewegt stehen lassen!)");
    Serial.println(" ?         diese Hilfe");
    Serial.println();
    Serial.print(" aktiv: Sensor 0x");
    Serial.print(RADAR_SENSOR_ADDRS[activeSensor], HEX);
    Serial.print("  Bereich ");
    Serial.print(RADAR_START_MM);
    Serial.print("-");
    Serial.print(RADAR_END_MM);
    Serial.print(" mm");
    Serial.print("  signalQuality=");
    Serial.print(RADAR_SIGNAL_QUALITY);
    Serial.print("  thresholdSensitivity=");
    Serial.println(RADAR_THRESHOLD_SENSITIVITY);
    Serial.println(" (Werte kommen aus lib/include/params/params_radar.h)");
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
        driftHaveRef = false;
        Serial.print("Sensor 0x");
        Serial.print(RADAR_SENSOR_ADDRS[activeSensor], HEX);
        Serial.println(" gewaehlt.");
        connectSensor(activeSensor);
        break;
    }
    case 'k':
        driftRunning = false;
        sensorReady[activeSensor] = runCalibration(activeSensor, 0);
        break;
    case 'v':
        driftRunning = false;
        runCalibration(activeSensor, 1);
        break;
    case 'r':
        driftRunning = false;
        runCalibration(activeSensor, 2);
        break;
    case 'n':
        driftRunning = false;
        modeRepeat((uint8_t)atoi(arg));
        break;
    case 'b':
        driftRunning = false;
        modeBootSequence();
        break;
    case 's':
        modeStatus();
        break;
    case 'w':
        driftRunning = !driftRunning;
        if (driftRunning)
        {
            driftStartMs = millis();
            driftLastMs = millis() - DRIFT_INTERVAL_MS; // sofort erste Messung
            driftHaveRef = false;
            driftRecalCount = 0;
            Serial.println();
            Serial.println("Driftbeobachtung an. Objekt in fester Distanz stehen lassen und nicht");
            Serial.print("bewegen. Messung alle ");
            Serial.print(DRIFT_INTERVAL_MS / 1000);
            Serial.println(" s. Erneut 'w' beendet.");
            Serial.println("Erwartung: dT steigt in den ersten Minuten, draw sollte klein bleiben.");
        }
        else
        {
            Serial.print("Driftbeobachtung aus. recalibrate()-Aufrufe: ");
            Serial.println(driftRecalCount);
        }
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
    Serial.println("XM125 Kalibrierung 1/4 -- Modul-interne Kalibrierung");
    Serial.println("Umfeld waehrend jeder Kalibrierung frei halten, nichts bewegen.");
    Serial.println();

    modeBootSequence();
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

    if (driftRunning && (millis() - driftLastMs) >= DRIFT_INTERVAL_MS)
    {
        driftLastMs = millis();
        driftTick();
    }
}
