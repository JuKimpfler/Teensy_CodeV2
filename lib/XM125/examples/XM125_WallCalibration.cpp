// XM125_WallCalibration.cpp
// Kalibrierwerkzeug fuer die 4 XM125-Radarsensoren auf eine GERADE HOLZWAND
// in bis zu 2,5 m Abstand.
//
// Beantwortet zwei getrennte Fragen:
//   1) REAGIERT der Sensor ueberhaupt auf die Wand?   -> Mode 'm' und 's'
//   2) Welche echte Distanz gehoert zu welchem Rohwert? -> Mode 'd' und 'f'
//
// Ausführen: Datei nach src/main.cpp des Hauptprojekts kopieren (in src/ darf
// immer nur eine Datei mit setup()/loop() liegen) und normal bauen/hochladen:
//   pio run -e teensy40 -t upload
// Siehe lib/XM125/examples/README.md fuer alle Beispiele im Ueberblick.
//
// Hinweis: die Modi 's' (Empfindlichkeits-Sweep) und 'i' (Uebersprech-Test)
// sind hier nur noch aus historischen Gruenden enthalten -- als eigenstaendige,
// weiter ausgebaute Beispiele siehe XM125_Calib_Sensitivity.cpp bzw.
// XM125_Calib_Crosstalk.cpp. Fuer dieses Skript zaehlen 'd', 'f', 'c' (Teil B:
// Rohwert -> mm, siehe lib/XM125/doc/07-Kalibrierung.md).
//
// Messaufbau:
//   - Sensor senkrecht auf eine glatte, ebene Holzflaeche (Sperrholz/OSB/Tuer,
//     mind. ca. 60x60 cm) ausrichten, Sensormitte auf Wandmitte.
//   - Abstand ab Sensor-Vorderkante messen, nicht ab Platinenrand.
//   - Im Bereich vor dem Sensor sonst NICHTS stehen lassen -- Tischkanten,
//     Stuhlbeine und Personen erzeugen eigene Peaks und verfaelschen die
//     Zuordnung Rohwert <-> Distanz. (Oeffnungswinkel der integrierten
//     Antenne siehe XM125-Datenblatt.)
//   - Sensor waehrend einer Messreihe nicht in der Hand halten.
//
// Ergebnis uebertragen: die im Sweep gefundenen Werte fuer signalQuality und
// thresholdSensitivity nach lib/include/params.h (RADAR_SIGNAL_QUALITY,
// RADAR_THRESHOLD_SENSITIVITY) eintragen, dann gilt die Abstimmung fuer den
// gesamten Roboter (systemlib::setupRadarSens).

#include <Arduino.h>
#include <Wire.h>
#include "XM125Distance.h"

// ── Aufbau ────────────────────────────────────────────────────────────────────
static constexpr uint8_t NUM_SENSORS = 4;
static constexpr uint8_t SENSOR_ADDRS[NUM_SENSORS] = { 0x51, 0x52, 0x53, 0x54 };
static constexpr int8_t  WAKE_PIN = -1;
static constexpr int8_t  INT_PIN  = -1;

// Messbereich. endMm bewusst ueber dem Ziel von 2500 mm, damit eine Wand bei
// exakt 2,5 m nicht auf der Bereichskante liegt (dort wird sie unzuverlaessig
// erkannt, siehe NEAR_START_EDGE/Bereichsende im Datenblatt).
static constexpr uint32_t CAL_START_MM = 200;
static constexpr uint32_t CAL_END_MM   = 2700;

// Sweep-Raster: Signalqualitaet (1000x, Default 15000) x CFAR-Empfindlichkeit
// (0..1000, Default 500). Hoehere Werte = empfindlicher, aber langsamer bzw.
// mehr Fehlalarme.
static const uint32_t SWEEP_SIGNAL_QUALITY[]        = { 15000, 20000, 25000, 30000 };
static const uint32_t SWEEP_THRESHOLD_SENSITIVITY[] = { 500, 650, 750, 900 };
static constexpr uint8_t SWEEP_SQ_COUNT   = sizeof(SWEEP_SIGNAL_QUALITY) / sizeof(SWEEP_SIGNAL_QUALITY[0]);
static constexpr uint8_t SWEEP_SENS_COUNT = sizeof(SWEEP_THRESHOLD_SENSITIVITY) / sizeof(SWEEP_THRESHOLD_SENSITIVITY[0]);

static constexpr uint8_t  BURST_SIZE      = 20;   ///< Messungen pro Bewertung
static constexpr uint32_t MEASURE_TIMEOUT = 3000; ///< PROFILE5 braucht laenger als der Default

static constexpr uint8_t MAX_CAL_POINTS = 12;

XM125Distance sensors[NUM_SENSORS] = {
  XM125Distance(SENSOR_ADDRS[0], WAKE_PIN, INT_PIN),
  XM125Distance(SENSOR_ADDRS[1], WAKE_PIN, INT_PIN),
  XM125Distance(SENSOR_ADDRS[2], WAKE_PIN, INT_PIN),
  XM125Distance(SENSOR_ADDRS[3], WAKE_PIN, INT_PIN),
};

uint8_t  activeSensor = 0;
uint32_t activeSignalQuality        = 25000;
uint32_t activeThresholdSensitivity = 750;
bool     sensorReady[NUM_SENSORS]   = {};

struct CalPoint {
  uint16_t trueMm;      ///< tatsaechlicher Abstand, vom Benutzer eingegeben
  uint8_t  rawMedian;   ///< Median von getPeakDistanceRaw()
  uint32_t regMedian;   ///< Median des ungekuerzten Registerworts
  int32_t  strength;    ///< Median der Signalstaerke
  uint8_t  detections;  ///< von BURST_SIZE Messungen erkannt
};
CalPoint calPoints[MAX_CAL_POINTS];
uint8_t  calPointCount = 0;

struct BurstResult {
  uint8_t  ok;            ///< Messung ohne Fehler
  uint8_t  detections;    ///< davon mit mindestens einem Peak
  uint8_t  rawMedian;
  uint32_t regMedian;
  int32_t  strengthMedian;
  uint16_t avgMeasureMs;
};

// ── Kleine Helfer ─────────────────────────────────────────────────────────────

static void printHex32(uint32_t v) {
  Serial.print("0x");
  for (int8_t shift = 28; shift >= 0; shift -= 4) {
    Serial.print((int)((v >> shift) & 0xF), HEX);
  }
}

/** Aufsteigend sortieren (Insertion Sort, n <= BURST_SIZE). */
static void sortU32(uint32_t* a, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {
    uint32_t key = a[i];
    int16_t j = i - 1;
    while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
    a[j + 1] = key;
  }
}

static void sortI32(int32_t* a, uint8_t n) {
  for (uint8_t i = 1; i < n; i++) {
    int32_t key = a[i];
    int16_t j = i - 1;
    while (j >= 0 && a[j] > key) { a[j + 1] = a[j]; j--; }
    a[j + 1] = key;
  }
}

/** Registerwort in allen plausiblen Deutungen ausgeben (siehe Mode 'f'). */
static void printRegInterpretations(uint32_t reg) {
  uint32_t swapped = __builtin_bswap32(reg);
  float asFloat = 0.0f, asFloatSwapped = 0.0f;
  memcpy(&asFloat, &reg, sizeof(asFloat));
  memcpy(&asFloatSwapped, &swapped, sizeof(asFloatSwapped));

  Serial.print("    reg=");        printHex32(reg);
  Serial.print("  Bytes=");
  Serial.print((int)((reg >> 24) & 0xFF)); Serial.print(".");
  Serial.print((int)((reg >> 16) & 0xFF)); Serial.print(".");
  Serial.print((int)((reg >> 8) & 0xFF));  Serial.print(".");
  Serial.print((int)(reg & 0xFF));
  Serial.println();
  Serial.print("    als uint32 = ");        Serial.print(reg);
  Serial.print("   byteswapped = ");        Serial.println(swapped);
  Serial.print("    als float  = ");        Serial.print(asFloat, 4);
  Serial.print("   float(swapped) = ");     Serial.println(asFloatSwapped, 4);
}

// ── Sensorsteuerung ───────────────────────────────────────────────────────────

static void printDiagnostics(XM125Distance& radar) {
  uint32_t detStatus   = radar.getDetectorStatus();
  uint32_t protoStatus = radar.getProtocolStatus();
  Serial.print("  Detector Status: "); printHex32(detStatus);
  Serial.print("  BUSY=");  Serial.print(XM125Base::statusBusy(detStatus) ? "1" : "0");
  Serial.print(" ERROR=");  Serial.println(XM125Base::statusError(detStatus) ? "1" : "0");
  Serial.print("  Protocol Status: "); printHex32(protoStatus); Serial.println();
}

/** Konfiguration fuer eine planare Holzwand schreiben und kalibrieren. */
static bool applyTuning(uint8_t idx, uint32_t signalQuality, uint32_t thresholdSensitivity) {
  XM125Distance& radar = sensors[idx];

  radar.config.startMm                       = CAL_START_MM;
  radar.config.endMm                         = CAL_END_MM;
  radar.config.maxProfile                    = XM125D_PROFILE5;          // groesste Reichweite
  radar.config.thresholdMethod               = XM125D_THRESHOLD_CFAR;    // adaptiv
  radar.config.peakSorting                   = XM125D_SORT_CLOSEST;      // Wand = naechstes Objekt
  radar.config.reflectorShape                = XM125D_REFLECTOR_PLANAR;  // flaechiger Reflektor
  radar.config.closeRangeLeakageCancellation = true;
  radar.config.signalQuality                 = signalQuality;
  radar.config.thresholdSensitivity          = thresholdSensitivity;

  if (!radar.applyConfigAndCalibrate()) {
    Serial.println("  FEHLER: applyConfigAndCalibrate() fehlgeschlagen.");
    uint16_t failedReg = radar.getLastFailedConfigRegister();
    if (failedReg != 0xFFFF) {
      Serial.print("  -> I2C-Schreibfehler bei Register 0x"); Serial.println(failedReg, HEX);
    }
    printDiagnostics(radar);
    return false;
  }
  return true;
}

static bool connectSensor(uint8_t idx) {
  sensorReady[idx] = false;
  if (!sensors[idx].begin(Wire)) {
    Serial.print("Sensor 0x"); Serial.print(SENSOR_ADDRS[idx], HEX);
    Serial.println(": nicht erreichbar (SDA/SCL/3V3/GND pruefen).");
    return false;
  }
  sensorReady[idx] = applyTuning(idx, activeSignalQuality, activeThresholdSensitivity);
  return sensorReady[idx];
}

/**
 * n Messungen ausfuehren und zusammenfassen.
 * @param disturb  wenn true, senden alle anderen Sensoren waehrend jeder
 *                 Messung gleichzeitig mit -- so laesst sich das Uebersprechen
 *                 direkt gegen den ungestoerten Fall vergleichen (Mode 'i').
 */
static BurstResult runBurst(uint8_t idx, uint8_t n, bool disturb = false) {
  XM125Distance& radar = sensors[idx];
  BurstResult res = {};

  uint32_t raws[BURST_SIZE];
  uint32_t regs[BURST_SIZE];
  int32_t  strengths[BURST_SIZE];
  uint32_t totalMs = 0;

  if (n > BURST_SIZE) n = BURST_SIZE;

  for (uint8_t i = 0; i < n; i++) {
    if (disturb) {
      // Nachbarn unmittelbar vorher anstossen, damit sie waehrend der
      // eigentlichen Messung senden. Ihre Ergebnisse interessieren nicht.
      for (uint8_t j = 0; j < NUM_SENSORS; j++) {
        if (j == idx || !sensorReady[j]) continue;
        sensors[j].startMeasurement();
      }
    }

    uint32_t t0 = millis();
    bool ok = radar.measureBlocking(MEASURE_TIMEOUT);
    totalMs += (millis() - t0);

    if (!ok) {
      if (radar.getCalibrationNeeded()) radar.recalibrate();
      continue;
    }
    res.ok++;

    if (radar.getNumDistances() == 0) continue;

    raws[res.detections]      = radar.getPeakDistanceRaw(0);
    regs[res.detections]      = radar.getPeakDistanceReg(0);
    strengths[res.detections] = radar.getPeakStrength(0);
    res.detections++;
  }

  res.avgMeasureMs = (n > 0) ? (uint16_t)(totalMs / n) : 0;

  if (res.detections > 0) {
    sortU32(raws, res.detections);
    sortU32(regs, res.detections);
    sortI32(strengths, res.detections);
    uint8_t mid = res.detections / 2;
    res.rawMedian      = (uint8_t)raws[mid];
    res.regMedian      = regs[mid];
    res.strengthMedian = strengths[mid];
  }
  return res;
}

// ── Modi ──────────────────────────────────────────────────────────────────────

static void modeSweep() {
  Serial.println();
  Serial.println("=== Parameter-Sweep ===============================================");
  Serial.print("Sensor 0x"); Serial.print(SENSOR_ADDRS[activeSensor], HEX);
  Serial.print(", "); Serial.print(BURST_SIZE); Serial.println(" Messungen je Kombination.");
  Serial.println("Wand JETZT auf den GROESSTEN gewuenschten Abstand stellen (2,5 m):");
  Serial.println("dort ist die Erkennung am schwierigsten, naeher wird es nur besser.");
  Serial.println();
  Serial.println(" SigQual  Sens  Erkannt  Rohwert  Staerke   Zeit");
  Serial.println(" -------  ----  -------  -------  -------   ----");

  uint32_t bestSq = 0, bestSens = 0;
  uint16_t bestCost = 0xFFFF;

  for (uint8_t s = 0; s < SWEEP_SQ_COUNT; s++) {
    for (uint8_t t = 0; t < SWEEP_SENS_COUNT; t++) {
      uint32_t sq   = SWEEP_SIGNAL_QUALITY[s];
      uint32_t sens = SWEEP_THRESHOLD_SENSITIVITY[t];

      if (!applyTuning(activeSensor, sq, sens)) continue;
      BurstResult r = runBurst(activeSensor, BURST_SIZE);

      Serial.print("  ");
      Serial.print(sq);    Serial.print("   ");
      Serial.print(sens);  Serial.print("    ");
      Serial.print(r.detections); Serial.print("/"); Serial.print(BURST_SIZE);
      Serial.print("    ");
      Serial.print(r.rawMedian);  Serial.print("     ");
      Serial.print(r.strengthMedian); Serial.print("   ");
      Serial.print(r.avgMeasureMs); Serial.println(" ms");

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
    Serial.println("  - Wand nicht senkrecht getroffen (schon 10-15 Grad Schraeglage");
    Serial.println("    lenken die Reflexion an der glatten Flaeche weg vom Sensor)");
    Serial.println("  - Wandflaeche zu klein / zu weit ausserhalb der Sensorachse");
    Serial.println("  - Abstand groesser als CAL_END_MM");
    Serial.println("  - Sensorabdeckung (Gehaeuse, Kabelbaum) direkt vor der Antenne");
  } else {
    Serial.println("Empfehlung fuer lib/include/params.h:");
    Serial.print("  RADAR_SIGNAL_QUALITY        = "); Serial.print(bestSq);   Serial.println(";");
    Serial.print("  RADAR_THRESHOLD_SENSITIVITY = "); Serial.print(bestSens); Serial.println(";");
    activeSignalQuality        = bestSq;
    activeThresholdSensitivity = bestSens;
  }

  applyTuning(activeSensor, activeSignalQuality, activeThresholdSensitivity);
  Serial.println("===================================================================");
}

static void modeInterference() {
  Serial.println();
  Serial.println("=== Uebersprech-Test ==============================================");
  Serial.print("Referenzsensor 0x"); Serial.print(SENSOR_ADDRS[activeSensor], HEX);
  Serial.println(", Wand bei 2,5 m aufstellen (dort ist die Reserve am kleinsten).");
  Serial.println("Verglichen wird: Sensor misst allein <-> alle anderen senden gleichzeitig.");
  Serial.println();

  BurstResult quiet = runBurst(activeSensor, BURST_SIZE, false);
  Serial.print("  allein:          "); Serial.print(quiet.detections); Serial.print("/");
  Serial.print(BURST_SIZE); Serial.print(" erkannt, Staerke ");
  Serial.print(quiet.strengthMedian); Serial.print(", "); Serial.print(quiet.avgMeasureMs);
  Serial.println(" ms");

  BurstResult noisy = runBurst(activeSensor, BURST_SIZE, true);
  Serial.print("  mit Nachbarn:    "); Serial.print(noisy.detections); Serial.print("/");
  Serial.print(BURST_SIZE); Serial.print(" erkannt, Staerke ");
  Serial.print(noisy.strengthMedian); Serial.print(", "); Serial.print(noisy.avgMeasureMs);
  Serial.println(" ms");

  Serial.println();
  if (quiet.detections == 0) {
    Serial.println("  Der Sensor sieht die Wand schon allein nicht -- erst mit 's' die");
    Serial.println("  Empfindlichkeit klaeren, dann diesen Test wiederholen.");
    Serial.println("===================================================================");
    return;
  }

  int16_t lostDetections = (int16_t)quiet.detections - (int16_t)noisy.detections;
  float strengthDrop = 0.0f;
  if (quiet.strengthMedian != 0) {
    strengthDrop = 100.0f * (float)(quiet.strengthMedian - noisy.strengthMedian)
                          / (float)quiet.strengthMedian;
  }
  Serial.print("  verlorene Treffer: "); Serial.println(lostDetections);
  Serial.print("  Staerkeverlust:    "); Serial.print(strengthDrop, 1); Serial.println(" %");
  Serial.println();

  if (lostDetections <= 0 && strengthDrop < 10.0f) {
    Serial.println("  -> Kein messbares Uebersprechen. Der Gleichzeitigbetrieb aller");
    Serial.println("     Sensoren (systemlib::readRadar()) ist damit unbedenklich.");
  } else if (lostDetections <= 1 && strengthDrop < 25.0f) {
    Serial.println("  -> Schwaches Uebersprechen. Im Blick behalten, aber der");
    Serial.println("     Gleichzeitigbetrieb bleibt vertretbar.");
  } else {
    Serial.println("  -> Deutliches Uebersprechen. Der Roboter misst trotzdem immer");
    Serial.println("     gleichzeitig, also an der Ursache ansetzen: zeigen zwei");
    Serial.println("     Sensoren teilweise in dieselbe Richtung, oder reflektiert die");
    Serial.println("     Wand von einem zum anderen (z.B. in einer Ecke)? Details liefert");
    Serial.println("     XM125_Calib_Crosstalk.cpp (Mode 'p').");
  }
  Serial.println("===================================================================");
}

static void modeRecordPoint(uint16_t trueMm) {
  if (calPointCount >= MAX_CAL_POINTS) {
    Serial.println("Speicher voll -- mit 'c' loeschen oder mit 'f' auswerten.");
    return;
  }

  Serial.print("Messe bei "); Serial.print(trueMm); Serial.println(" mm ...");
  BurstResult r = runBurst(activeSensor, BURST_SIZE);

  Serial.print("  erkannt "); Serial.print(r.detections); Serial.print("/"); Serial.print(BURST_SIZE);
  Serial.print(", Rohwert "); Serial.print(r.rawMedian);
  Serial.print(", Staerke "); Serial.println(r.strengthMedian);

  if (r.detections == 0) {
    Serial.println("  -> kein Peak, Punkt NICHT gespeichert.");
    return;
  }
  if (r.detections < BURST_SIZE / 2) {
    Serial.println("  WARNUNG: weniger als die Haelfte der Messungen hat die Wand gesehen,");
    Serial.println("  der Punkt ist entsprechend unsicher.");
  }
  printRegInterpretations(r.regMedian);

  CalPoint& p = calPoints[calPointCount++];
  p.trueMm     = trueMm;
  p.rawMedian  = r.rawMedian;
  p.regMedian  = r.regMedian;
  p.strength   = r.strengthMedian;
  p.detections = r.detections;
  Serial.print("  gespeichert als Punkt "); Serial.println(calPointCount);
}

static void modeFit() {
  Serial.println();
  Serial.println("=== Auswertung ====================================================");
  if (calPointCount == 0) {
    Serial.println("Keine Punkte aufgenommen ('d <cm>' benutzen).");
    return;
  }

  Serial.println(" Distanz  Rohwert  Staerke  erkannt");
  for (uint8_t i = 0; i < calPointCount; i++) {
    Serial.print("  "); Serial.print(calPoints[i].trueMm); Serial.print(" mm     ");
    Serial.print(calPoints[i].rawMedian); Serial.print("     ");
    Serial.print(calPoints[i].strength); Serial.print("    ");
    Serial.print(calPoints[i].detections); Serial.print("/"); Serial.println(BURST_SIZE);
    printRegInterpretations(calPoints[i].regMedian);
  }

  if (calPointCount < 2) {
    Serial.println("Fuer eine Geradenanpassung werden mindestens 2 Punkte gebraucht.");
    Serial.println("===================================================================");
    return;
  }

  // Ausgleichsgerade trueMm = a * rawMedian + b
  float n = (float)calPointCount;
  float sx = 0.0f, sy = 0.0f, sxx = 0.0f, sxy = 0.0f;
  for (uint8_t i = 0; i < calPointCount; i++) {
    float x = (float)calPoints[i].rawMedian;
    float y = (float)calPoints[i].trueMm;
    sx += x; sy += y; sxx += x * x; sxy += x * y;
  }
  float denom = n * sxx - sx * sx;
  if (fabsf(denom) < 1e-3f) {
    Serial.println("Rohwert ist ueber alle Punkte konstant -- er traegt keine Distanz-");
    Serial.println("information. Stattdessen die oben ausgegebenen Registerdeutungen");
    Serial.println("vergleichen: welche Spalte laeuft monoton mit der Distanz mit?");
    Serial.println("===================================================================");
    return;
  }

  float a = (n * sxy - sx * sy) / denom;
  float b = (sy - a * sx) / n;

  Serial.println();
  Serial.println("Ausgleichsgerade  Distanz_mm = a * Rohwert + b");
  Serial.print("  a = "); Serial.print(a, 3); Serial.println(" mm pro Count");
  Serial.print("  b = "); Serial.print(b, 3); Serial.println(" mm");

  float maxAbsResidual = 0.0f;
  Serial.println("  Restfehler je Punkt:");
  for (uint8_t i = 0; i < calPointCount; i++) {
    float fit = a * (float)calPoints[i].rawMedian + b;
    float res = fit - (float)calPoints[i].trueMm;
    if (fabsf(res) > maxAbsResidual) maxAbsResidual = fabsf(res);
    Serial.print("    "); Serial.print(calPoints[i].trueMm); Serial.print(" mm -> ");
    Serial.print(fit, 0); Serial.print(" mm  (Abweichung "); Serial.print(res, 0);
    Serial.println(" mm)");
  }
  Serial.print("  groesste Abweichung: "); Serial.print(maxAbsResidual, 0); Serial.println(" mm");

  if (maxAbsResidual > 100.0f) {
    Serial.println();
    Serial.println("  Die Gerade passt schlecht (>100 mm Abweichung). Der 8-bit-Rohwert");
    Serial.println("  ist dann vermutlich NICHT linear in der Distanz. Oben stehen zu");
    Serial.println("  jedem Punkt die vier Deutungen des vollstaendigen Registerworts --");
    Serial.println("  diejenige, die sauber mit der Distanz mitlaeuft, ist die richtige");
    Serial.println("  Kodierung und gehoert in XM125Distance::_readPeaks().");
  } else {
    Serial.println();
    Serial.println("  Passt. Werte in XM125Distance::getPeakDistanceMm() eintragen.");
  }
  Serial.println("===================================================================");
}

static void modeMonitor() {
  XM125Distance& radar = sensors[activeSensor];
  static uint16_t seen = 0, total = 0;

  uint32_t t0 = millis();
  bool ok = radar.measureBlocking(MEASURE_TIMEOUT);
  uint32_t dt = millis() - t0;
  total++;

  Serial.print("[0x"); Serial.print(SENSOR_ADDRS[activeSensor], HEX); Serial.print("] ");
  if (!ok) {
    Serial.print("Messfehler/Timeout nach "); Serial.print(dt); Serial.println(" ms");
    if (radar.getCalibrationNeeded()) { Serial.println("  -> recalibrate()"); radar.recalibrate(); }
  } else {
    uint8_t peaks = radar.getNumDistances();
    if (peaks > 0) seen++;
    Serial.print("Peaks="); Serial.print(peaks);
    Serial.print(" Temp="); Serial.print(radar.getTemperature()); Serial.print("C");
    Serial.print(" Zeit="); Serial.print(dt); Serial.print("ms");
    for (uint8_t p = 0; p < peaks; p++) {
      Serial.print("  P"); Serial.print(p);
      Serial.print(": raw="); Serial.print(radar.getPeakDistanceRaw(p));
      Serial.print(" reg=");  printHex32(radar.getPeakDistanceReg(p));
      Serial.print(" S=");    Serial.print(radar.getPeakStrength(p));
    }
    Serial.print("   Trefferquote "); Serial.print(seen); Serial.print("/"); Serial.print(total);
    Serial.println();
  }
  if (total >= 1000) { total = 0; seen = 0; }
}

// ── Bedienung ─────────────────────────────────────────────────────────────────

bool monitorRunning = false;

static void printHelp() {
  Serial.println();
  Serial.println("=== XM125 Wand-Kalibrierung =======================================");
  Serial.println(" a <0..3>  Sensor waehlen (0x51/0x52/0x53/0x54)");
  Serial.println(" m         Dauermessung an/aus (zeigt, ob die Wand gesehen wird)");
  Serial.println(" s         Parameter-Sweep bei 2,5 m -> empfohlene Einstellungen");
  Serial.println(" i         Uebersprech-Test: stoeren sich gleichzeitig messende Sensoren?");
  Serial.println(" d <cm>    Kalibrierpunkt bei bekanntem Abstand aufnehmen, z.B. 'd 150'");
  Serial.println(" f         Kalibrierpunkte auswerten (Rohwert -> mm)");
  Serial.println(" c         Kalibrierpunkte loeschen");
  Serial.println(" ?         diese Hilfe");
  Serial.println();
  Serial.print(" aktiv: Sensor 0x"); Serial.print(SENSOR_ADDRS[activeSensor], HEX);
  Serial.print(", signalQuality="); Serial.print(activeSignalQuality);
  Serial.print(", thresholdSensitivity="); Serial.print(activeThresholdSensitivity);
  Serial.print(", Bereich "); Serial.print(CAL_START_MM); Serial.print("-");
  Serial.print(CAL_END_MM); Serial.println(" mm");
  Serial.println("===================================================================");
}

static void handleCommand(char* line) {
  while (*line == ' ') line++;
  char cmd = *line;
  const char* arg = line + 1;

  switch (cmd) {
    case 'a': {
      int idx = atoi(arg);
      if (idx < 0 || idx >= (int)NUM_SENSORS) { Serial.println("Sensorindex 0..3."); break; }
      activeSensor = (uint8_t)idx;
      Serial.print("Sensor 0x"); Serial.print(SENSOR_ADDRS[activeSensor], HEX); Serial.println(" gewaehlt.");
      connectSensor(activeSensor);
      break;
    }
    case 'm':
      monitorRunning = !monitorRunning;
      Serial.println(monitorRunning ? "Dauermessung an." : "Dauermessung aus.");
      break;
    case 's':
      monitorRunning = false;
      modeSweep();
      break;
    case 'i':
      monitorRunning = false;
      modeInterference();
      break;
    case 'd': {
      int cm = atoi(arg);
      if (cm <= 0) { Serial.println("Abstand in cm angeben, z.B. 'd 150'."); break; }
      monitorRunning = false;
      modeRecordPoint((uint16_t)(cm * 10));
      break;
    }
    case 'f':
      monitorRunning = false;
      modeFit();
      break;
    case 'c':
      calPointCount = 0;
      Serial.println("Kalibrierpunkte geloescht.");
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

  Wire.begin();
  Wire.setClock(100000); // niedriger Takt, mehr Toleranz fuer Clock-Stretching des XM125

  Serial.println();
  Serial.println("XM125 Kalibrierung auf gerade Holzwand (bis 2,5 m)");

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print("Sensor 0x"); Serial.print(SENSOR_ADDRS[i], HEX); Serial.print(": ");
    Serial.println(connectSensor(i) ? "bereit" : "FEHLER");
  }

  printHelp();
}

void loop() {
  static char   line[32];
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

  if (monitorRunning) modeMonitor();
}
