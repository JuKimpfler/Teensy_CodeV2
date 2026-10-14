// XM125_Calib_Crosstalk.cpp
// ── Kalibriermethode 4: UEBERSPRECHEN ─────────────────────────────────────────
//
// Beantwortet: Stoeren sich die Sensoren gegenseitig, wenn sie GLEICHZEITIG
// messen -- und wenn ja, welches Paar?
//
// Der Roboter misst grundsaetzlich mit allen Sensoren gleichzeitig
// (systemlib::readRadar(): anstossen, nie warten, Ergebnisse abholen). Nur so
// faellt die Messdauer einmal statt einmal pro Sensor an. Dieses Skript prueft,
// was dieser Betrieb an Trefferquote kostet; Gegenmittel sind Montage
// (Blickrichtungen trennen) und Empfindlichkeit, kein Parameter in params.h.
//
// Reihenfolge der Kalibrierskripte:
//   1. XM125_Calib_Module.cpp       Modul kalibriert ueberhaupt?
//   2. XM125_Calib_Alignment.cpp    Sensor mechanisch auf die Wand ausrichten
//   3. XM125_Calib_Sensitivity.cpp  signalQuality / thresholdSensitivity finden
//   4. XM125_Calib_Crosstalk.cpp    <- dieses hier
//
// Ausführen: Datei nach src/main.cpp des Hauptprojekts kopieren (in src/ darf
// immer nur eine Datei mit setup()/loop() liegen) und normal bauen/hochladen:
//   pio run -e teensy40 -t upload
// Siehe lib/XM125/examples/README.md fuer alle Beispiele im Ueberblick.
//
// Messaufbau:
//   - Alle vier Sensoren montiert und angeschlossen, sonst misst der Test nichts.
//   - Vor jedem Sensor eine Wand in 2,5 m (Roboter mittig zwischen den Banden):
//     dort ist die Reserve am kleinsten, dort zeigt sich Uebersprechen zuerst.
//   - Empfindlichkeit vorher abgestimmt (Kalibrierung 3/4) -- sonst misst dieser
//     Test nur, dass die Wand ohnehin nicht sicher erkannt wird.

#include <Arduino.h>
#include <Wire.h>
#include <params.h>
#include "XM125Distance.h"

static constexpr uint32_t MEASURE_TIMEOUT_MS = RADAR_MEASURE_TIMEOUT_MS;
static constexpr uint8_t BURST_SIZE = 20;   ///< Messungen je Bewertung
static constexpr uint8_t CYCLE_ROUNDS = 15; ///< Runden je Gruppengroesse ('g')

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

struct BurstResult
{
    uint8_t detections;
    uint8_t errors;
    int32_t strengthMedian;
    uint16_t avgMeasureMs;
};

struct CycleResult
{
    uint16_t cycleMs; ///< mittlere Dauer einer kompletten Runde
    uint8_t detections[RADAR_NUM_SENSORS];
    int32_t strengthMedian[RADAR_NUM_SENSORS];
    uint8_t failures; ///< Sensor-Runden ohne gueltiges Ergebnis
};

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

/** Prozentualer Staerkeverlust gegenueber der ungestoerten Referenz. */
static float strengthDropPct(int32_t quiet, int32_t noisy)
{
    if (quiet == 0)
        return 0.0f;
    return 100.0f * (float)(quiet - noisy) / (float)quiet;
}

// ── Sensorsteuerung ───────────────────────────────────────────────────────────

/** Projektkonfiguration aus params.h -- identisch zu systemlib::setupRadarSens(). */
static bool applyProjectConfig(uint8_t idx)
{
    XM125Distance &radar = sensors[idx];

    radar.setMeasureTimeout(MEASURE_TIMEOUT_MS);

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
        Serial.print("  FEHLER: Kalibrierung Sensor 0x");
        Serial.print(RADAR_SENSOR_ADDRS[idx], HEX);
        Serial.println(" fehlgeschlagen.");
        Serial.println("  -> Details klaert XM125_Calib_Module.cpp (Kalibriermethode 1).");
        return false;
    }
    return true;
}

/**
 * Sensoren NACHEINANDER initialisieren: waehrend einer die Rauschpegel-
 * Schaetzung macht, darf kein Nachbar senden (Doku A.4).
 */
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

static uint8_t readyCount()
{
    uint8_t n = 0;
    for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
        if (sensorReady[i])
            n++;
    return n;
}

/**
 * Burst auf einem Sensor, waehrend eine Auswahl von Nachbarn mitsendet.
 * @param disturbMask  Bitmaske der Stoerer; deren Messungen werden unmittelbar
 *                     vor der eigenen angestossen, ihre Ergebnisse interessieren
 *                     nicht. Maske 0 = ungestoerte Referenz.
 */
static BurstResult runBurst(uint8_t idx, uint8_t n, uint8_t disturbMask)
{
    XM125Distance &radar = sensors[idx];
    BurstResult res = {};

    int32_t strengths[BURST_SIZE];
    uint32_t totalMs = 0;
    if (n > BURST_SIZE)
        n = BURST_SIZE;

    for (uint8_t i = 0; i < n; i++)
    {
        for (uint8_t j = 0; j < RADAR_NUM_SENSORS; j++)
        {
            if (j == idx || !sensorReady[j])
                continue;
            if (disturbMask & (1 << j))
                sensors[j].startMeasurement();
        }

        uint32_t t0 = millis();
        bool ok = radar.measureBlocking(MEASURE_TIMEOUT_MS);
        totalMs += (millis() - t0);

        if (!ok)
        {
            res.errors++;
            if (radar.getCalibrationNeeded())
                radar.recalibrate();
            continue;
        }
        if (radar.getNumDistances() == 0)
            continue;

        strengths[res.detections++] = radar.getPeakStrength(0);
    }

    res.avgMeasureMs = (n > 0) ? (uint16_t)(totalMs / n) : 0;
    if (res.detections > 0)
    {
        sortI32(strengths, res.detections);
        res.strengthMedian = strengths[res.detections / 2];
    }
    return res;
}

/**
 * Eine Sensorgruppe messen lassen: erst bei allen startMeasurement(), dann
 * reihum isMeasuring() bis alle durch sind, dann die Ergebnisse aus dem Cache
 * holen. Gruppenbildung ueber die Schrittweite `stride`; stride == 1 ab
 * `first` == 0 ist die Betriebsart des Roboters (alle gleichzeitig, siehe
 * systemlib::readRadar()), grosse Schrittweiten messen nacheinander.
 *
 * Anders als im Roboter wird hier bewusst gewartet, bis die Gruppe fertig ist
 * -- dieses Skript misst ja gerade, was eine Runde kostet.
 */
static void readGroup(uint8_t first, uint8_t stride, bool detected[RADAR_NUM_SENSORS],
                      int32_t strength[RADAR_NUM_SENSORS], uint8_t &failures)
{
    bool pending[RADAR_NUM_SENSORS] = {};

    for (uint8_t i = first; i < RADAR_NUM_SENSORS; i += stride)
    {
        if (!sensorReady[i])
            continue;
        if (sensors[i].startMeasurement())
            pending[i] = true;
        else
            failures++;
    }

    bool anyPending = true;
    while (anyPending)
    {
        anyPending = false;
        for (uint8_t i = first; i < RADAR_NUM_SENSORS; i += stride)
        {
            if (!pending[i])
                continue;
            if (sensors[i].isMeasuring())
            {
                anyPending = true;
                continue;
            }

            // Fertig (oder abgebrochen -- den Timeout fuehrt die Library selbst).
            pending[i] = false;
            if (!sensors[i].hasResult())
            {
                failures++;
                continue;
            }

            const XM125DistanceResult &res = sensors[i].getLastResult();
            if (res.numDistances == 0)
                continue;

            detected[i] = true;
            strength[i] = res.peakStrength[0];
        }
    }
}

/** rounds komplette Runden mit der gegebenen Gruppengroesse fahren. */
static CycleResult runCycles(uint8_t groupSize, uint8_t rounds)
{
    CycleResult res = {};
    if (rounds > BURST_SIZE)
        rounds = BURST_SIZE;

    const uint8_t numGroups = RADAR_NUM_SENSORS / groupSize;
    int32_t strengths[RADAR_NUM_SENSORS][BURST_SIZE];
    uint32_t totalMs = 0;

    for (uint8_t r = 0; r < rounds; r++)
    {
        bool detected[RADAR_NUM_SENSORS] = {};
        int32_t strength[RADAR_NUM_SENSORS] = {};

        uint32_t t0 = millis();
        for (uint8_t g = 0; g < numGroups; g++)
        {
            readGroup(g, numGroups, detected, strength, res.failures);
        }
        totalMs += millis() - t0;

        for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
        {
            if (!detected[i])
                continue;
            strengths[i][res.detections[i]++] = strength[i];
        }
    }

    res.cycleMs = (rounds > 0) ? (uint16_t)(totalMs / rounds) : 0;
    for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
    {
        if (res.detections[i] > 0)
        {
            sortI32(strengths[i], res.detections[i]);
            res.strengthMedian[i] = strengths[i][res.detections[i] / 2];
        }
    }
    return res;
}

// ── Modi ──────────────────────────────────────────────────────────────────────

/** Grundtest: ein Sensor allein gegen "alle anderen senden gleichzeitig". */
static void modeInterference()
{
    Serial.println();
    Serial.println("=== Uebersprech-Test (allein <-> alle Nachbarn) ===================");
    Serial.print("Referenzsensor ");
    Serial.print(SENSOR_NAMES[activeSensor]);
    Serial.print(" 0x");
    Serial.println(RADAR_SENSOR_ADDRS[activeSensor], HEX);
    Serial.println("Wand bei 2,5 m aufstellen -- dort ist die Reserve am kleinsten.");
    Serial.println();

    uint8_t allOthers = 0;
    for (uint8_t j = 0; j < RADAR_NUM_SENSORS; j++)
    {
        if (j != activeSensor && sensorReady[j])
            allOthers |= (1 << j);
    }
    if (allOthers == 0)
    {
        Serial.println("  Kein zweiter Sensor bereit -- ohne Nachbarn ist nichts zu messen.");
        Serial.println("===================================================================");
        return;
    }

    BurstResult quiet = runBurst(activeSensor, BURST_SIZE, 0);
    Serial.print("  allein:        ");
    Serial.print(quiet.detections);
    Serial.print("/");
    Serial.print(BURST_SIZE);
    Serial.print(" erkannt, Staerke ");
    Serial.print(quiet.strengthMedian);
    Serial.print(", ");
    Serial.print(quiet.avgMeasureMs);
    Serial.println(" ms");

    BurstResult noisy = runBurst(activeSensor, BURST_SIZE, allOthers);
    Serial.print("  mit Nachbarn:  ");
    Serial.print(noisy.detections);
    Serial.print("/");
    Serial.print(BURST_SIZE);
    Serial.print(" erkannt, Staerke ");
    Serial.print(noisy.strengthMedian);
    Serial.print(", ");
    Serial.print(noisy.avgMeasureMs);
    Serial.println(" ms");

    Serial.println();
    if (quiet.detections == 0)
    {
        Serial.println("  Der Sensor sieht die Wand schon allein nicht -- erst Kalibrierung 2/4");
        Serial.println("  (Ausrichtung) und 3/4 (Empfindlichkeit), dann diesen Test wiederholen.");
        Serial.println("===================================================================");
        return;
    }

    int16_t lost = (int16_t)quiet.detections - (int16_t)noisy.detections;
    float drop = strengthDropPct(quiet.strengthMedian, noisy.strengthMedian);
    Serial.print("  verlorene Treffer: ");
    Serial.println(lost);
    Serial.print("  Staerkeverlust:    ");
    Serial.print(drop, 1);
    Serial.println(" %");
    Serial.println();

    if (lost <= 0 && drop < 10.0f)
    {
        Serial.println("  -> Kein messbares Uebersprechen. Mit 'g' gegenpruefen, wie viel");
        Serial.println("     Rundendauer der Gleichzeitigbetrieb wirklich spart.");
    }
    else if (lost <= 1 && drop < 25.0f)
    {
        Serial.println("  -> Schwaches Uebersprechen. Mit 'g' pruefen, ob es ueber eine ganze");
        Serial.println("     Runde ins Gewicht faellt, und mit 'p' das stoerende Paar suchen.");
    }
    else
    {
        Serial.println("  -> Deutliches Uebersprechen. Mit 'p' klaeren, WELCHER Nachbar stoert,");
        Serial.println("     und ob die Montage schuld ist (zwei Sensoren mit ueberlappendem");
        Serial.println("     Blickfeld, oder eine Ecke, die von einem zum anderen spiegelt).");
    }
    Serial.println("===================================================================");
}

/** Paarweise: welcher einzelne Nachbar stoert den Referenzsensor wie stark? */
static void modePairMatrix()
{
    Serial.println();
    Serial.println("=== Paarweises Uebersprechen ======================================");
    Serial.print("Referenzsensor ");
    Serial.print(SENSOR_NAMES[activeSensor]);
    Serial.print(" 0x");
    Serial.println(RADAR_SENSOR_ADDRS[activeSensor], HEX);
    Serial.println("Jeweils genau EIN Nachbar sendet mit -- zeigt, welche Paare sich");
    Serial.println("vertragen und damit in eine gemeinsame Messgruppe duerfen.");
    Serial.println();

    BurstResult quiet = runBurst(activeSensor, BURST_SIZE, 0);
    Serial.print("  Referenz allein: ");
    Serial.print(quiet.detections);
    Serial.print("/");
    Serial.print(BURST_SIZE);
    Serial.print(" erkannt, Staerke ");
    Serial.println(quiet.strengthMedian);

    if (quiet.detections == 0)
    {
        Serial.println("  Ohne Echo im Ruhefall ist der Vergleich sinnlos -- erst Kalibrierung 2/4 und 3/4.");
        Serial.println("===================================================================");
        return;
    }

    Serial.println();
    Serial.println(" Stoerer     Erkannt  Verlust  Staerke  Staerkeverlust");
    Serial.println(" ---------   -------  -------  -------  --------------");

    for (uint8_t j = 0; j < RADAR_NUM_SENSORS; j++)
    {
        if (j == activeSensor)
            continue;
        if (!sensorReady[j])
        {
            Serial.print("  ");
            Serial.print(SENSOR_NAMES[j]);
            Serial.println(" 0x--     nicht bereit");
            continue;
        }

        BurstResult r = runBurst(activeSensor, BURST_SIZE, (uint8_t)(1 << j));
        Serial.print("  ");
        Serial.print(SENSOR_NAMES[j]);
        Serial.print(" 0x");
        Serial.print(RADAR_SENSOR_ADDRS[j], HEX);
        Serial.print("    ");
        Serial.print(r.detections);
        Serial.print("/");
        Serial.print(BURST_SIZE);
        Serial.print("     ");
        Serial.print((int16_t)quiet.detections - (int16_t)r.detections);
        Serial.print("       ");
        Serial.print(r.strengthMedian);
        Serial.print("     ");
        Serial.print(strengthDropPct(quiet.strengthMedian, r.strengthMedian), 1);
        Serial.println(" %");
    }

    Serial.println();
    Serial.println("  Auffaellig ist ein Nachbar, der mehr als einen Treffer kostet oder die");
    Serial.println("  Staerke um mehr als ein Viertel druckt. Solche Paare gehoeren nicht in");
    Serial.println("  dieselbe Messgruppe -- die Standardgruppierung stellt deshalb die");
    Serial.println("  gegenueberliegenden Sensoren zusammen (Vo+Hi, Li+Re).");
    Serial.println("===================================================================");
}

/**
 * Der eigentliche Entscheidungstest: komplette Radar-Runden nacheinander gegen
 * alle gleichzeitig -- gemessen wird beides, was zaehlt: Trefferquote je Sensor
 * und Dauer einer kompletten Runde.
 *
 * Der Roboter misst immer gleichzeitig (systemlib::readRadar()). Dieser Test
 * beantwortet also nicht mehr "welche Gruppengroesse einstellen", sondern
 * "kostet der Gleichzeitigbetrieb Trefferquote?" -- wenn ja, ist die Montage
 * bzw. die Empfindlichkeit dran, nicht mehr ein Parameter in params.h.
 */
static void modeGroupCompare()
{
    Serial.println();
    Serial.println("=== Nacheinander gegen gleichzeitig ===============================");
    Serial.print("Je ");
    Serial.print(CYCLE_ROUNDS);
    Serial.println(" komplette Runden ueber alle Sensoren, wie systemlib::readRadar().");
    Serial.println("Vor jedem Sensor sollte eine Wand stehen, sonst fehlen Trefferquoten.");
    Serial.println();

    if (readyCount() < RADAR_NUM_SENSORS)
    {
        Serial.print("  WARNUNG: nur ");
        Serial.print(readyCount());
        Serial.print(" von ");
        Serial.print(RADAR_NUM_SENSORS);
        Serial.println(" Sensoren bereit -- die Zeiten sind dadurch zu guenstig.");
        Serial.println();
    }

    const uint8_t groupSizes[2] = {1, RADAR_NUM_SENSORS}; // 1 = nacheinander
    CycleResult results[2];

    Serial.println(" Gruppe  Zyklus     Vo      Li      Hi      Re     Fehler");
    Serial.println(" ------  --------   -----   -----   -----   -----  ------");

    for (uint8_t k = 0; k < 2; k++)
    {
        results[k] = runCycles(groupSizes[k], CYCLE_ROUNDS);

        Serial.print("   ");
        Serial.print(groupSizes[k]);
        Serial.print("    ");
        Serial.print(results[k].cycleMs);
        Serial.print(" ms   ");
        for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
        {
            Serial.print(results[k].detections[i]);
            Serial.print("/");
            Serial.print(CYCLE_ROUNDS);
            Serial.print("   ");
        }
        Serial.print(" ");
        Serial.println(results[k].failures);
    }

    Serial.println();
    Serial.println(" Staerke im Median je Sensor:");
    for (uint8_t k = 0; k < 2; k++)
    {
        Serial.print("   Gruppe ");
        Serial.print(groupSizes[k]);
        Serial.print(":  ");
        for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
        {
            Serial.print(SENSOR_NAMES[i]);
            Serial.print("=");
            Serial.print(results[k].strengthMedian[i]);
            Serial.print("  ");
        }
        Serial.println();
    }

    // Bewertung gegen "nacheinander" (per Definition stoerungsfrei)
    const CycleResult &ref = results[0];
    const CycleResult &para = results[1];
    bool acceptable = true;
    for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
    {
        if (!sensorReady[i])
            continue;
        if (para.detections[i] + 1 < ref.detections[i])
            acceptable = false; // mehr als 1 Treffer verloren
        if (strengthDropPct(ref.strengthMedian[i], para.strengthMedian[i]) > 25.0f)
            acceptable = false;
    }

    Serial.println();
    Serial.print("    Rundendauer ");
    Serial.print(ref.cycleMs);
    Serial.print(" ms -> ");
    Serial.print(para.cycleMs);
    Serial.print(" ms");
    if (ref.cycleMs > 0 && para.cycleMs < ref.cycleMs)
    {
        Serial.print("  (-");
        Serial.print(100 - (100UL * para.cycleMs) / ref.cycleMs);
        Serial.print(" %)");
    }
    Serial.println();

    if (acceptable)
    {
        Serial.println("    -> Gleichzeitig messen kostet keine nennenswerte Trefferquote.");
        Serial.println("       Der Betrieb in systemlib::readRadar() ist damit bestaetigt.");
    }
    else
    {
        Serial.println("    -> Gleichzeitig messen kostet Treffer. Der Roboter misst trotzdem");
        Serial.println("       immer gleichzeitig, also hier ansetzen: Montage pruefen ('p'");
        Serial.println("       zeigt das stoerende Paar) oder Empfindlichkeit nachziehen");
        Serial.println("       (XM125_Calib_Sensitivity.cpp).");
    }
    Serial.println("===================================================================");
}

// ── Bedienung ─────────────────────────────────────────────────────────────────

static void printHelp()
{
    Serial.println();
    Serial.println("=== XM125 Kalibrierung 4/4: Uebersprechen / Gruppengroesse ========");
    Serial.println(" a <0..3>  Referenzsensor waehlen (0=Vo, 1=Li, 2=Hi, 3=Re)");
    Serial.println(" i         allein gegen 'alle Nachbarn senden mit'");
    Serial.println(" p         paarweise: welcher einzelne Nachbar stoert?");
    Serial.println(" g         komplette Runden: nacheinander gegen gleichzeitig");
    Serial.println(" k         alle Sensoren neu kalibrieren (nacheinander)");
    Serial.println(" ?         diese Hilfe");
    Serial.println();
    Serial.print(" aktiv: ");
    Serial.print(SENSOR_NAMES[activeSensor]);
    Serial.print(" 0x");
    Serial.print(RADAR_SENSOR_ADDRS[activeSensor], HEX);
    Serial.print("  bereit: ");
    Serial.print(readyCount());
    Serial.print("/");
    Serial.println(RADAR_NUM_SENSORS);
    Serial.println(" Betrieb im Roboter: alle Sensoren messen gleichzeitig.");
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
        Serial.print("Referenzsensor ");
        Serial.print(SENSOR_NAMES[activeSensor]);
        Serial.print(" 0x");
        Serial.print(RADAR_SENSOR_ADDRS[activeSensor], HEX);
        Serial.println(" gewaehlt.");
        if (!sensorReady[activeSensor])
            connectSensor(activeSensor);
        break;
    }
    case 'i':
        modeInterference();
        break;
    case 'p':
        modePairMatrix();
        break;
    case 'g':
        modeGroupCompare();
        break;
    case 'k':
        Serial.println("Alle Sensoren nacheinander neu kalibrieren, Umfeld frei halten ...");
        for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
        {
            Serial.print("  ");
            Serial.print(SENSOR_NAMES[i]);
            Serial.print(" 0x");
            Serial.print(RADAR_SENSOR_ADDRS[i], HEX);
            Serial.print(": ");
            Serial.println(connectSensor(i) ? "bereit" : "FEHLER");
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
    Serial.println("XM125 Kalibrierung 4/4 -- Uebersprechen zwischen den vier Sensoren");
    Serial.println("Sensoren nacheinander initialisieren (nur einer kalibriert zur Zeit):");

    for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
    {
        Serial.print("  ");
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
}
