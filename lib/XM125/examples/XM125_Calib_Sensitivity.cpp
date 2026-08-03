// XM125_Calib_Sensitivity.cpp
// ── Kalibriermethode 3: EMPFINDLICHKEIT auf eine Holzwand abstimmen ───────────
//
// Beantwortet: Mit welchen Werten fuer signalQuality und thresholdSensitivity
// (und welchem Profil/Reflektormodell) taucht eine Holzwand am aeussersten
// Nutzabstand bei JEDER Messung als Peak auf -- ohne unnoetig langsam oder
// unnoetig fehleranfaellig zu werden?
//
// Ergebnis wandert nach lib/include/params.h:
//   RADAR_SIGNAL_QUALITY, RADAR_THRESHOLD_SENSITIVITY
// von dort holt systemlib::setupRadarSens() die Werte fuer den ganzen Roboter.
//
// Reihenfolge der Kalibrierskripte:
//   1. XM125_Calib_Module.cpp       Modul kalibriert ueberhaupt?
//   2. XM125_Calib_Alignment.cpp    Sensor mechanisch auf die Wand ausrichten
//   3. XM125_Calib_Sensitivity.cpp  <- dieses hier
//   4. XM125_Calib_Crosstalk.cpp    RADAR_PARALLEL_GROUP festlegen
//
// Ausführen: Datei nach src/main.cpp des Hauptprojekts kopieren (in src/ darf
// immer nur eine Datei mit setup()/loop() liegen) und normal bauen/hochladen:
//   pio run -e teensy40 -t upload
// Siehe lib/XM125/examples/README.md fuer alle Beispiele im Ueberblick.
//
// Messaufbau:
//   - Wand auf den GROESSTEN gewuenschten Abstand stellen (2,5 m): dort ist die
//     Erkennung am schwierigsten, naeher wird es nur besser.
//   - Sensor vorher mit XM125_Calib_Alignment.cpp ausgerichtet -- ein Sweep auf
//     einen schlecht getroffenen Reflektor liefert nur zu hohe Empfindlichkeiten.
//   - Sonst nichts im Strahlkegel, Sensor fest montiert.
//
// Hinweis zur Laufzeit: jede Kombination wird komplett neu konfiguriert und
// kalibriert (mehrere Sekunden) und danach BURST_SIZE mal gemessen. Der volle
// Sweep 's' dauert dadurch einige Minuten.

#include <Arduino.h>
#include <Wire.h>
#include <params.h>
#include "XM125Distance.h"

static constexpr uint32_t MEASURE_TIMEOUT_MS = RADAR_MEASURE_TIMEOUT_MS;
static constexpr uint8_t  BURST_SIZE         = 20; ///< Messungen pro Bewertung

// Sweep-Raster: Signalqualitaet (1000x-Einheit, Acconeer-Default 15000) x
// CFAR-Empfindlichkeit (0..1000, Default 500). Hoehere Werte = empfindlicher,
// aber laengere Messzeit bzw. mehr Fehlalarme.
static const uint32_t SWEEP_SIGNAL_QUALITY[]        = { 15000, 20000, 25000, 30000 };
static const uint32_t SWEEP_THRESHOLD_SENSITIVITY[] = { 500, 650, 750, 900 };
static constexpr uint8_t SWEEP_SQ_COUNT   = sizeof(SWEEP_SIGNAL_QUALITY) / sizeof(SWEEP_SIGNAL_QUALITY[0]);
static constexpr uint8_t SWEEP_SENS_COUNT = sizeof(SWEEP_THRESHOLD_SENSITIVITY) / sizeof(SWEEP_THRESHOLD_SENSITIVITY[0]);

XM125Distance sensors[RADAR_NUM_SENSORS] = {
  XM125Distance(RADAR_SENSOR_ADDRS[0], RADAR_WAKE_PIN, RADAR_INT_PIN),
  XM125Distance(RADAR_SENSOR_ADDRS[1], RADAR_WAKE_PIN, RADAR_INT_PIN),
  XM125Distance(RADAR_SENSOR_ADDRS[2], RADAR_WAKE_PIN, RADAR_INT_PIN),
  XM125Distance(RADAR_SENSOR_ADDRS[3], RADAR_WAKE_PIN, RADAR_INT_PIN),
};

uint8_t activeSensor = 0;
bool    sensorReady[RADAR_NUM_SENSORS] = {};

// Aktuell eingestellte Werte -- Startpunkt ist immer das, was im Roboter laeuft.
uint32_t activeSignalQuality        = RADAR_SIGNAL_QUALITY;
uint32_t activeThresholdSensitivity = RADAR_THRESHOLD_SENSITIVITY;
uint32_t activeProfile              = XM125D_PROFILE5;
uint32_t activeReflectorShape       = XM125D_REFLECTOR_PLANAR;

struct BurstResult {
  uint8_t  detections;     ///< Messungen mit mindestens einem Peak
  uint8_t  errors;         ///< measure() fehlgeschlagen / Timeout
  uint8_t  multiPeak;      ///< Messungen mit mehr als einem Peak (Fehlalarm-Indiz)
  int32_t  strengthMedian;
  uint16_t avgMeasureMs;
};

// ── Kleine Helfer ─────────────────────────────────────────────────────────────

static void sortI32(int32_t* a, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {
    int32_t key = a[i];
    int16_t j = i - 1;
    while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
    a[j + 1] = key;
  }
}

static const char* profileName(uint32_t p) {
  switch (p) {
    case XM125D_PROFILE1: return "PROFILE1";
    case XM125D_PROFILE2: return "PROFILE2";
    case XM125D_PROFILE3: return "PROFILE3";
    case XM125D_PROFILE4: return "PROFILE4";
    default:              return "PROFILE5";
  }
}

// ── Sensorsteuerung ───────────────────────────────────────────────────────────

/**
 * Eine Parameterkombination schreiben und kalibrieren.
 * Alles ausser den vier abgestimmten Groessen entspricht params.h.
 */
static bool applyTuning(uint8_t idx, uint32_t signalQuality, uint32_t thresholdSensitivity,
                        uint32_t profile, uint32_t reflectorShape) {
  XM125Distance& radar = sensors[idx];

  radar.config.startMm                       = RADAR_START_MM;
  radar.config.endMm                         = RADAR_END_MM;
  radar.config.maxProfile                    = profile;
  radar.config.thresholdMethod               = XM125D_THRESHOLD_CFAR;   // adaptiv
  radar.config.peakSorting                   = XM125D_SORT_CLOSEST;     // Wand = naechstes Objekt
  radar.config.reflectorShape                = reflectorShape;
  radar.config.closeRangeLeakageCancellation = true;
  radar.config.signalQuality                 = signalQuality;
  radar.config.thresholdSensitivity          = thresholdSensitivity;

  if (!radar.applyConfigAndCalibrate()) {
    Serial.println("  FEHLER: applyConfigAndCalibrate() fehlgeschlagen.");
    uint16_t failedReg = radar.getLastFailedConfigRegister();
    if (failedReg != 0xFFFF) {
      Serial.print("  -> I2C-Schreibfehler bei Register 0x"); Serial.println(failedReg, HEX);
    } else {
      Serial.println("  -> Kombination evtl. ausserhalb des zulaessigen Bereichs, oder");
      Serial.println("     Kalibrierung gestoert. Details: XM125_Calib_Module.cpp.");
    }
    return false;
  }
  return true;
}

/** Aktuell gewaehlte Einstellungen anwenden. */
static bool applyActive(uint8_t idx) {
  return applyTuning(idx, activeSignalQuality, activeThresholdSensitivity,
                     activeProfile, activeReflectorShape);
}

static bool connectSensor(uint8_t idx) {
  sensorReady[idx] = false;
  if (!sensors[idx].begin(I2C_RADAR)) {
    Serial.print("Sensor 0x"); Serial.print(RADAR_SENSOR_ADDRS[idx], HEX);
    Serial.println(": nicht erreichbar (SDA/SCL/3V3/GND pruefen).");
    return false;
  }
  sensorReady[idx] = applyActive(idx);
  return sensorReady[idx];
}

static BurstResult runBurst(uint8_t idx, uint8_t n) {
  XM125Distance& radar = sensors[idx];
  BurstResult res = {};

  int32_t  strengths[BURST_SIZE];
  uint32_t totalMs = 0;
  if (n > BURST_SIZE) n = BURST_SIZE;

  for (uint8_t i = 0; i < n; i++) {
    uint32_t t0 = millis();
    bool ok = radar.measure(MEASURE_TIMEOUT_MS);
    totalMs += (millis() - t0);

    if (!ok) {
      res.errors++;
      if (radar.getCalibrationNeeded()) radar.recalibrate();
      continue;
    }
    if (radar.getNumDistances() == 0) continue;
    if (radar.getNumDistances() > 1)  res.multiPeak++;

    strengths[res.detections++] = radar.getPeakStrength(0);
  }

  res.avgMeasureMs = (n > 0) ? (uint16_t)(totalMs / n) : 0;
  if (res.detections > 0) {
    sortI32(strengths, res.detections);
    res.strengthMedian = strengths[res.detections / 2];
  }
  return res;
}

/** Kopfzeile + eine Ergebniszeile im gleichen Raster. */
static void printResultHeader(const char* firstCol) {
  Serial.println();
  Serial.print(" "); Serial.print(firstCol);
  Serial.println("  Erkannt  Mehrfach  Fehler  Staerke   Zeit");
}

static void printResultRow(const BurstResult& r) {
  Serial.print("   "); Serial.print(r.detections); Serial.print("/"); Serial.print(BURST_SIZE);
  Serial.print("     "); Serial.print(r.multiPeak);
  Serial.print("        "); Serial.print(r.errors);
  Serial.print("     "); Serial.print(r.strengthMedian);
  Serial.print("   "); Serial.print(r.avgMeasureMs); Serial.println(" ms");
}

/** Den Block ausgeben, der so in params.h uebernommen werden kann. */
static void printParamsBlock() {
  Serial.println();
  Serial.println("  Fuer lib/include/params.h:");
  Serial.print("    static constexpr uint32_t RADAR_SIGNAL_QUALITY        = ");
  Serial.print(activeSignalQuality); Serial.println(";");
  Serial.print("    static constexpr uint32_t RADAR_THRESHOLD_SENSITIVITY = ");
  Serial.print(activeThresholdSensitivity); Serial.println(";");
  if (activeProfile != XM125D_PROFILE5 || activeReflectorShape != XM125D_REFLECTOR_PLANAR) {
    Serial.println("  Abweichend vom Projektstand -- diese beiden stehen fest in");
    Serial.println("  systemlib::setupRadarSens() und muessten dort geaendert werden:");
    Serial.print("    maxProfile      = "); Serial.println(profileName(activeProfile));
    Serial.print("    reflectorShape  = ");
    Serial.println(activeReflectorShape == XM125D_REFLECTOR_PLANAR ? "PLANAR" : "GENERIC");
  }
}

// ── Modi ──────────────────────────────────────────────────────────────────────

/** Schnelltest mit den aktuell eingestellten Werten. */
static void modeTest() {
  Serial.println();
  Serial.println("=== Test der aktuellen Einstellung ================================");
  Serial.print("Sensor 0x"); Serial.print(RADAR_SENSOR_ADDRS[activeSensor], HEX);
  Serial.print("  sq="); Serial.print(activeSignalQuality);
  Serial.print("  sens="); Serial.print(activeThresholdSensitivity);
  Serial.print("  "); Serial.print(profileName(activeProfile));
  Serial.print("  "); Serial.println(activeReflectorShape == XM125D_REFLECTOR_PLANAR ? "PLANAR" : "GENERIC");

  BurstResult r = runBurst(activeSensor, BURST_SIZE);
  printResultHeader("           ");
  printResultRow(r);

  Serial.println();
  if (r.detections == BURST_SIZE) {
    Serial.println("  -> Wand wird zuverlaessig erkannt.");
  } else if (r.detections == 0) {
    Serial.println("  -> Gar kein Echo. Erst die Ausrichtung pruefen");
    Serial.println("     (XM125_Calib_Alignment.cpp), dann 's'.");
  } else {
    Serial.println("  -> Nicht jede Messung sieht die Wand: mit 's' den Sweep fahren.");
  }
  Serial.println("===================================================================");
}

/** Sweep signalQuality x thresholdSensitivity -- die eigentliche Abstimmung. */
static void modeSweep() {
  Serial.println();
  Serial.println("=== Sweep signalQuality x thresholdSensitivity ====================");
  Serial.print("Sensor 0x"); Serial.print(RADAR_SENSOR_ADDRS[activeSensor], HEX);
  Serial.print(", "); Serial.print(BURST_SIZE); Serial.print(" Messungen je Kombination, ");
  Serial.print(SWEEP_SQ_COUNT * SWEEP_SENS_COUNT); Serial.println(" Kombinationen.");
  Serial.println("Wand JETZT auf den GROESSTEN gewuenschten Abstand stellen (2,5 m).");
  Serial.println();
  Serial.println(" SigQual  Sens  Erkannt  Mehrfach  Staerke   Zeit");
  Serial.println(" -------  ----  -------  --------  -------   ----");

  uint32_t bestSq = 0, bestSens = 0;
  uint16_t bestCost = 0xFFFF;

  for (uint8_t s = 0; s < SWEEP_SQ_COUNT; s++) {
    for (uint8_t t = 0; t < SWEEP_SENS_COUNT; t++) {
      uint32_t sq   = SWEEP_SIGNAL_QUALITY[s];
      uint32_t sens = SWEEP_THRESHOLD_SENSITIVITY[t];

      if (!applyTuning(activeSensor, sq, sens, activeProfile, activeReflectorShape)) continue;
      BurstResult r = runBurst(activeSensor, BURST_SIZE);

      Serial.print("  "); Serial.print(sq);   Serial.print("   ");
      Serial.print(sens); Serial.print("    ");
      Serial.print(r.detections); Serial.print("/"); Serial.print(BURST_SIZE);
      Serial.print("      "); Serial.print(r.multiPeak);
      Serial.print("       "); Serial.print(r.strengthMedian);
      Serial.print("   "); Serial.print(r.avgMeasureMs); Serial.println(" ms");

      // Guenstigste Kombination, die alle Messungen erkennt: moeglichst wenig
      // Signalqualitaet (= kurze Messzeit) und moeglichst wenig Empfindlichkeit
      // (= wenig Fehlalarme).
      if (r.detections == BURST_SIZE) {
        uint16_t cost = (uint16_t)(sq / 1000) + (uint16_t)(sens / 100);
        if (cost < bestCost) { bestCost = cost; bestSq = sq; bestSens = sens; }
      }
    }
  }

  Serial.println();
  if (bestCost == 0xFFFF) {
    Serial.println("KEINE Kombination hat die Wand bei jeder Messung erkannt.");
    Serial.println("Moegliche Ursachen, in dieser Reihenfolge pruefen:");
    Serial.println("  - Wand nicht senkrecht getroffen (schon 10-15 Grad Schraeglage lenken");
    Serial.println("    die Reflexion an der glatten Flaeche weg vom Sensor)");
    Serial.println("    -> XM125_Calib_Alignment.cpp, Modus 'm'");
    Serial.println("  - Wandflaeche zu klein / zu weit ausserhalb der Sensorachse");
    Serial.print(  "  - Abstand groesser als RADAR_END_MM = "); Serial.println(RADAR_END_MM);
    Serial.println("  - Sensorabdeckung (Gehaeuse, Kabelbaum) direkt vor der Antenne");
    Serial.println("  - Profil zu niedrig -> mit 'p' gegenpruefen");
  } else {
    activeSignalQuality        = bestSq;
    activeThresholdSensitivity = bestSens;
    Serial.println("Guenstigste Kombination mit voller Trefferquote uebernommen:");
    Serial.print("  signalQuality        = "); Serial.println(bestSq);
    Serial.print("  thresholdSensitivity = "); Serial.println(bestSens);
    printParamsBlock();
    Serial.println();
    Serial.println("  Danach mit 't' gegenpruefen und den Sweep an einem zweiten Sensor");
    Serial.println("  wiederholen -- die Module streuen von Exemplar zu Exemplar.");
  }

  applyActive(activeSensor);
  Serial.println("===================================================================");
}

/** Profil-Sweep: bringt PROFILE5 wirklich mehr als ein kuerzeres Profil? */
static void modeProfileSweep() {
  Serial.println();
  Serial.println("=== Sweep maxProfile ==============================================");
  Serial.println("Laengere Profile = mehr Energie pro Puls = groessere Reichweite, dafuer");
  Serial.println("schlechtere Nahbereichsaufloesung. Wand auf 2,5 m stehen lassen.");
  Serial.print("sq="); Serial.print(activeSignalQuality);
  Serial.print(", sens="); Serial.println(activeThresholdSensitivity);
  Serial.println();
  Serial.println(" Profil     Erkannt  Mehrfach  Staerke   Zeit");
  Serial.println(" --------   -------  --------  -------   ----");

  uint32_t bestProfile = 0;
  int32_t  bestStrength = 0;

  for (uint32_t p = XM125D_PROFILE1; p <= XM125D_PROFILE5; p++) {
    if (!applyTuning(activeSensor, activeSignalQuality, activeThresholdSensitivity,
                     p, activeReflectorShape)) continue;
    BurstResult r = runBurst(activeSensor, BURST_SIZE);

    Serial.print("  "); Serial.print(profileName(p)); Serial.print("    ");
    Serial.print(r.detections); Serial.print("/"); Serial.print(BURST_SIZE);
    Serial.print("      "); Serial.print(r.multiPeak);
    Serial.print("       "); Serial.print(r.strengthMedian);
    Serial.print("   "); Serial.print(r.avgMeasureMs); Serial.println(" ms");

    if (r.detections == BURST_SIZE && r.strengthMedian > bestStrength) {
      bestStrength = r.strengthMedian;
      bestProfile  = p;
    }
  }

  Serial.println();
  if (bestProfile == 0) {
    Serial.println("  Kein Profil erreicht die volle Trefferquote -- erst 's' fahren.");
  } else {
    Serial.print("  Staerkstes Echo bei voller Trefferquote: "); Serial.println(profileName(bestProfile));
    Serial.println("  Das Projekt nutzt fest PROFILE5 (systemlib::setupRadarSens). Ein");
    Serial.println("  kuerzeres Profil lohnt nur, wenn es hier gleich gut abschneidet UND");
    Serial.println("  spuerbar schneller misst.");
    activeProfile = bestProfile;
  }

  applyActive(activeSensor);
  Serial.println("===================================================================");
}

/** reflectorShape: PLANAR gegen GENERIC, direkt am gleichen Aufbau. */
static void modeShapeCompare() {
  Serial.println();
  Serial.println("=== Vergleich reflectorShape ======================================");
  Serial.println("PLANAR rechnet damit, dass die Echo-Amplitude einer FLAECHE langsamer");
  Serial.println("mit der Distanz faellt als bei einem punktfoermigen Reflektor -- die");
  Serial.println("Schwelle am Bereichsende wird dadurch niedriger angesetzt.");
  Serial.println();
  Serial.println(" Shape      Erkannt  Mehrfach  Staerke   Zeit");
  Serial.println(" --------   -------  --------  -------   ----");

  BurstResult res[2] = {};
  const uint32_t shapes[2] = { XM125D_REFLECTOR_GENERIC, XM125D_REFLECTOR_PLANAR };
  const char*    names[2]  = { "GENERIC ", "PLANAR  " };

  for (uint8_t i = 0; i < 2; i++) {
    if (!applyTuning(activeSensor, activeSignalQuality, activeThresholdSensitivity,
                     activeProfile, shapes[i])) continue;
    res[i] = runBurst(activeSensor, BURST_SIZE);

    Serial.print("  "); Serial.print(names[i]); Serial.print("    ");
    Serial.print(res[i].detections); Serial.print("/"); Serial.print(BURST_SIZE);
    Serial.print("      "); Serial.print(res[i].multiPeak);
    Serial.print("       "); Serial.print(res[i].strengthMedian);
    Serial.print("   "); Serial.print(res[i].avgMeasureMs); Serial.println(" ms");
  }

  Serial.println();
  if (res[1].detections > res[0].detections) {
    Serial.println("  -> PLANAR erkennt die Wand zuverlaessiger. So laeuft es im Projekt.");
    activeReflectorShape = XM125D_REFLECTOR_PLANAR;
  } else if (res[0].detections > res[1].detections) {
    Serial.println("  -> GENERIC ist hier besser. Ungewoehnlich fuer eine flache Wand --");
    Serial.println("     pruefen, ob wirklich die Wand und nicht ein Kantenecho gemessen wird.");
  } else {
    Serial.println("  -> Kein Unterschied in der Trefferquote. Bei gleicher Quote ist PLANAR");
    Serial.println("     die richtige Wahl fuer eine Bandenwand (Reserve am Bereichsende).");
    activeReflectorShape = XM125D_REFLECTOR_PLANAR;
  }

  applyActive(activeSensor);
  Serial.println("===================================================================");
}

// ── Bedienung ─────────────────────────────────────────────────────────────────

static void printHelp() {
  Serial.println();
  Serial.println("=== XM125 Kalibrierung 3/4: Empfindlichkeit =======================");
  Serial.println(" a <0..3>  Sensor waehlen (0x51/0x52/0x53/0x54)");
  Serial.println(" t         aktuelle Einstellung testen (20 Messungen)");
  Serial.println(" s         Sweep signalQuality x thresholdSensitivity  -> Empfehlung");
  Serial.println(" p         Sweep maxProfile (PROFILE1..5)");
  Serial.println(" r         reflectorShape PLANAR gegen GENERIC vergleichen");
  Serial.println(" q         Werte-Block fuer params.h ausgeben");
  Serial.println(" d         zurueck auf die Werte aus params.h");
  Serial.println(" ?         diese Hilfe");
  Serial.println();
  Serial.print(" aktiv: Sensor 0x"); Serial.print(RADAR_SENSOR_ADDRS[activeSensor], HEX);
  Serial.print("  sq="); Serial.print(activeSignalQuality);
  Serial.print("  sens="); Serial.print(activeThresholdSensitivity);
  Serial.print("  "); Serial.print(profileName(activeProfile));
  Serial.print("  "); Serial.print(activeReflectorShape == XM125D_REFLECTOR_PLANAR ? "PLANAR" : "GENERIC");
  Serial.print("  Bereich "); Serial.print(RADAR_START_MM); Serial.print("-");
  Serial.print(RADAR_END_MM); Serial.println(" mm");
  Serial.println("===================================================================");
}

static void handleCommand(char* line) {
  while (*line == ' ') line++;
  char cmd = *line;
  const char* arg = line + 1;

  switch (cmd) {
    case 'a': {
      int idx = atoi(arg);
      if (idx < 0 || idx >= (int)RADAR_NUM_SENSORS) { Serial.println("Sensorindex 0..3."); break; }
      activeSensor = (uint8_t)idx;
      Serial.print("Sensor 0x"); Serial.print(RADAR_SENSOR_ADDRS[activeSensor], HEX); Serial.println(" gewaehlt.");
      connectSensor(activeSensor);
      break;
    }
    case 't': modeTest();         break;
    case 's': modeSweep();        break;
    case 'p': modeProfileSweep(); break;
    case 'r': modeShapeCompare(); break;
    case 'q': printParamsBlock(); break;
    case 'd':
      activeSignalQuality        = RADAR_SIGNAL_QUALITY;
      activeThresholdSensitivity = RADAR_THRESHOLD_SENSITIVITY;
      activeProfile              = XM125D_PROFILE5;
      activeReflectorShape       = XM125D_REFLECTOR_PLANAR;
      Serial.println("Zurueck auf die Projektwerte aus params.h.");
      applyActive(activeSensor);
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

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  I2C_RADAR.begin();
  I2C_RADAR.setClock(100000); // niedriger Takt, mehr Toleranz fuer Clock-Stretching des XM125

  Serial.println();
  Serial.println("XM125 Kalibrierung 3/4 -- Empfindlichkeit auf eine Holzwand");
  Serial.println("Wand auf 2,5 m stellen, Sensor vorher ausrichten (Kalibrierung 2/4).");

  for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++) {
    Serial.print("Sensor 0x"); Serial.print(RADAR_SENSOR_ADDRS[i], HEX); Serial.print(": ");
    Serial.println(connectSensor(i) ? "bereit" : "FEHLER");
  }

  printHelp();
}

void loop() {
  static char    line[32];
  static uint8_t len = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[len] = '\0';
      handleCommand(line);
      len = 0;
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}
