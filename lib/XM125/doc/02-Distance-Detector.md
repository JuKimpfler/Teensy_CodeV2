# Distance-Detector (`XM125Distance`)

Header: [XM125Distance.h](../XM125Distance.h) · Implementierung: [XM125Distance.cpp](../XM125Distance.cpp)

Der Distance-Detector misst Entfernungen zu bis zu 10 Reflektoren
("Peaks") innerhalb eines konfigurierbaren Messfensters (`startMm` …
`endMm`) und liefert je Peak eine Distanz und eine Signalstärke.

## Die drei Kommandos des Messbetriebs

Die Library **wartet nie auf ein Messergebnis**. Eine Messung dauert je nach
Konfiguration 20–200 ms; diese Zeit gehört dem Rest des Programms. Der gesamte
Messbetrieb besteht deshalb aus drei Aufrufen:

| Kommando | Was es tut | Kosten |
|---|---|---|
| `startMeasurement()` | **Messung starten** — schreibt `MEASURE_DISTANCE`, kehrt sofort zurück | 1 Registerschreibzugriff, < 1 ms |
| `isMeasuring()` | **Läuft eine Messung?** — treibt dabei die Zustandsmaschine weiter (BUSY prüfen, Ergebnis abholen) | höchstens 1 Registerlesezugriff, oft gar keiner |
| `getLastResult()` | **Letztes Messergebnis abfragen** — liefert `XM125DistanceResult` aus dem Cache | 0 (reiner Speicherzugriff) |

Sobald `isMeasuring()` `false` liefert, hat die Library das Ergebnis bereits
eingelesen. Zwischen zwei Runden bleibt es abrufbar — auch während schon die
nächste Messung läuft.

## Typischer Ablauf

```cpp
XM125Distance radar; // Default-I2C-Adresse, kein WAKE_UP/INT verkabelt
radar.begin(Wire);

radar.config.startMm = 200;
radar.config.endMm   = 4000;
radar.applyConfigAndCalibrate();   // schreibt config + kalibriert (kann Sekunden dauern)
radar.startMeasurement();          // erste Runde anstoßen

void loop() {
  if (radar.isMeasuring()) return;               // läuft noch → nichts zu tun

  const XM125DistanceResult& res = radar.getLastResult();
  if (res.valid) {
    for (uint8_t i = 0; i < res.numDistances; i++) {
      Serial.println(res.peakDistanceRaw[i]);
      Serial.println(res.peakStrength[i]);
    }
  }

  radar.startMeasurement();                      // nächste Runde
}
```

Alle Konfigurationsfelder (`radar.config.*`) sind in
[04-Konfiguration.md](04-Konfiguration.md) im Detail beschrieben.

## Mehrere Sensoren messen gleichzeitig

Weil `startMeasurement()` nicht wartet, starten alle Sensoren innerhalb
weniger Millisekunden — die Messdauer fällt damit **einmal für alle** an statt
einmal pro Sensor:

```cpp
for (auto& r : sensoren) r.startMeasurement();   // alle zusammen los

// in jedem Zyklus: einmal nachsehen, nie warten
bool laeuft = false;
for (auto& r : sensoren) if (r.isMeasuring()) laeuft = true;

if (!laeuft) {                                   // Runde komplett
  for (auto& r : sensoren) verarbeite(r.getLastResult());
  for (auto& r : sensoren) r.startMeasurement(); // nächste Runde
}
```

Genau dieses Schema fährt `systemlib::readRadar()` — siehe
[11-Hardware-und-Integration.md](11-Hardware-und-Integration.md).

Damit senden alle Sensoren gleichzeitig — zum Übersprechen siehe
[05-Fehlerbehandlung-und-Troubleshooting.md](05-Fehlerbehandlung-und-Troubleshooting.md#8-übersprechen-zwischen-mehreren-sensoren).

## Das Ergebnis: `XM125DistanceResult`

`getLastResult()` liefert eine Referenz auf diese Struktur:

| Feld | Typ | Bedeutung |
|---|---|---|
| `valid` | `bool` | `true` = die Werte stammen aus einer fehlerfreien Messung. Nach I²C-Fehler, `DETECTOR_ERROR`, Timeout oder `MEASURE_ERROR` steht hier `false`, die Zahlenwerte bleiben aber auf dem letzten guten Stand |
| `timestampMs` | `uint32_t` | `millis()` beim Einlesen — zusammen mit `getResultAgeMs()` die Antwort auf "wie alt ist der Wert?" |
| `durationMs` | `uint16_t` | Dauer von `startMeasurement()` bis Ergebnis (Diagnose) |
| `numDistances` | `uint8_t` | Anzahl gefundener Peaks (0–10) |
| `nearStartEdge` | `bool` | Objekt dicht an `config.startMm` |
| `calibrationNeeded` | `bool` | Sensor fordert `recalibrate()` an |
| `measureError` | `bool` | Messung selbst meldet einen Fehler |
| `temperature` | `int16_t` | °C, geringe Absolutgenauigkeit lt. Datenblatt |
| `peakDistanceRaw[10]` | `uint8_t` | unkalibrierte "Counts" — siehe Warnung unten |
| `peakDistanceReg[10]` | `uint32_t` | ungekürztes Registerwort je Peak |
| `peakStrength[10]` | `int32_t` | Signalstärke je Peak (1000×-Einheit) |

## API-Überblick

| Methode | Beschreibung |
|---|---|
| `writeConfig()` | Schreibt alle Werte aus `config` in die Konfigurationsregister. Bricht beim ersten fehlgeschlagenen Register-Write ab (fail-fast) |
| `getLastFailedConfigRegister()` | Registeradresse des zuletzt fehlgeschlagenen `writeConfig()`-Schreibvorgangs (`0xFFFF` = keiner) |
| `applyConfigAndCalibrate(timeoutMs = 3000)` | `writeConfig()` + Kommando `APPLY_CONFIG_AND_CALIBRATE`, blockiert bis fertig |
| `applyConfiguration(timeoutMs = 3000)` | Nur anwenden, ohne Neukalibrierung (z.B. nach Neustart mit noch gültiger alter Kalibrierung) |
| `calibrate(timeoutMs = 3000)` | Sensor/Detektor neu kalibrieren |
| `recalibrate(timeoutMs = 3000)` | Wie `calibrate()`, für den Fall dass `calibrationNeeded` gesetzt ist |
| `hardCalibration(cfg, timeoutMs = 3000, resetTimeoutMs = 3000)` | **Harte Neukalibrierung**: Modul neu starten, danach alle 14 Konfigurationsregister schreiben und kalibrieren. Blockiert lang, gehört ins Hochfahren |
| `hardCalibration(startMm, endMm, …)` | Dasselbe mit Einzelparametern statt Struktur — jeder Konfigurationswert einzeln setzbar, alles Weggelassene bekommt die Werksvorgabe. `hardCalibration()` ohne Argumente = Neustart + komplette Werksvorgabe |
| `startMeasurement()` | **Messung starten**, wartet nicht |
| `isMeasuring()` | **Läuft eine Messung?** — treibt die Zustandsmaschine, blockiert nie |
| `getLastResult()` | **Letztes Messergebnis** als `XM125DistanceResult` (Cache, kein I²C) |
| `hasResult()` | Kurzform für `getLastResult().valid` |
| `getResultAgeMs()` | Alter des zwischengespeicherten Ergebnisses in ms |
| `getState()` | `XM125MeasureState`: `IDLE`, `RUNNING`, `SETTLING`, `READY`, `FAILED` — ohne die Maschine weiterzutreiben |
| `setMeasureTimeout(ms)` | Ab wann eine laufende Messung als verloren gilt (Default 1000 ms) |
| `setMaxPeaksToFetch(n)` | Wie viele Peaks nach der Messung wirklich geholt werden (Default 10). Jeder Peak kostet zwei Registerlesezugriffe — wer nur Peak 0 auswertet, setzt 1 |
| `measureBlocking(timeoutMs = 1000)` | Starten **und warten**, bis das Ergebnis da ist. Nur für Kalibrier-/Testskripte, nicht im Roboterzyklus |
| `getNumDistances()` | Anzahl gefundener Peaks (0–10) der letzten Messung |
| `getNearStartEdge()` | `true`, wenn ein Objekt sehr nah an `config.startMm` erkannt wurde |
| `getCalibrationNeeded()` | `true`, wenn der Sensor eine Neukalibrierung anfordert → `recalibrate()` aufrufen |
| `getMeasureError()` | `true`, wenn die letzte Messung einen Fehler hatte |
| `getTemperature()` | Sensortemperatur in °C (geringe Absolutgenauigkeit lt. Datenblatt) |
| `getPeakDistanceRaw(idx)` | Roh-Distanzwert des Peaks `idx` (0–255 "Counts") |
| `getPeakDistanceMm(idx)` | **Aktuell identisch zu `getPeakDistanceRaw()`** — siehe Warnung unten |
| `getPeakDistanceReg(idx)` | Ungekürztes 32-bit-Registerwort des Peaks `idx` — für die Kalibrierung, siehe Warnung unten |
| `getPeakStrength(idx)` | Signalstärke des Peaks `idx` (1000x-Einheit, RSS-intern) |

Die `get*(idx)`-Einzelzugriffe sind bequeme Sichten auf `getLastResult()` und
kosten ebenfalls kein I²C.

## ⚠️ Unkalibrierte Peak-Distanz

Die auf den Sensoren in diesem Projekt laufende Firmware weicht vom
offiziellen Acconeer-Registerprotokoll ab: Statt eines vollen 32-bit
Distanzwerts pro Peak-Register wird offenbar nur **1 Byte** (das oberste
Byte des gelesenen 32-bit-Worts) mit einem Wert 0–255 ("Counts") befüllt.

- Der genaue Umrechnungsfaktor (mm pro Count, mit/ohne Offset, evtl.
  relativ zu `config.startMm`/`config.endMm`) ist **nicht zuverlässig
  bekannt** — zwei bisherige Annahmen dazu haben sich als falsch
  herausgestellt (siehe Code-Kommentar in
  [XM125Distance.cpp](../XM125Distance.cpp)).
- `getPeakDistanceMm()` ist deshalb aktuell nur ein **1:1-Platzhalter** um
  `getPeakDistanceRaw()` und liefert **keine echten Millimeter**.
- Wichtig: `getPeakDistanceRaw()` wirft 24 der 32 gelesenen Bits weg. Ob die
  Distanz wirklich im obersten Byte steckt, ist genau das, was noch nicht
  feststeht — für die Kalibrierung deshalb **`getPeakDistanceReg()`**
  protokollieren, das das vollständige Registerwort liefert.

### Kalibrieren mit `lib/XM125/examples/XM125_WallCalibration.cpp`

Der Sketch [XM125_WallCalibration.cpp](../examples/XM125_WallCalibration.cpp)
nimmt die Messreihe ab:

1. Ein einzelnes, flaches Objekt (Holzwand) auf **exakt bekannter Distanz**
   platzieren, sonst nichts im Messkegel.
2. `d <cm>` eingeben, z.B. `d 150`. Der Sketch misst 20-mal, bildet den
   Median und legt den Punkt ab.
3. Schritt 1–2 für mehrere Distanzen wiederholen (z.B. 50, 100, 150, 200,
   250 cm).
4. `f` wertet aus: Ausgleichsgerade `mm = a · Rohwert + b` samt Restfehler.

Zu jedem Punkt gibt der Sketch das vollständige Registerwort in **vier
Deutungen** aus — als `uint32`, byte-getauscht als `uint32`, als `float` und
byte-getauscht als `float`. Passt die Gerade über dem 8-bit-Rohwert schlecht
(Restfehler > 100 mm), ist die Spalte, die sauber mit der Distanz mitläuft,
die richtige Kodierung; sie gehört dann nach `XM125Distance::_readPeaks()`.

Details und Reproduktionshinweise: siehe
[05-Fehlerbehandlung-und-Troubleshooting.md](05-Fehlerbehandlung-und-Troubleshooting.md).

## Ergebnisregister im Detail (Reg. `0x0010`, `DISTANCE_RESULT`)

| Bits | Bedeutung |
|---|---|
| 0–3 | Anzahl gefundener Distanzen (`NUM_DISTANCES`, max. 10) |
| 8 | `NEAR_START_EDGE` — Objekt nahe der unteren Messbereichsgrenze |
| 9 | `CALIBRATION_NEEDED` — Neukalibrierung erforderlich |
| 10 | `MEASURE_ERROR` — Messung fehlgeschlagen |
| 16–31 | Temperatur (signed, °C) |

Peak-Register: `PEAK0_DISTANCE` (`0x0011`) und `PEAK0_STRENGTH` (`0x001B`),
jeweils `+i` für Peak `i` (0-basiert, bis zu 10 Peaks).

## Distance-spezifische Status-Bits (Reg. `DETECTOR_STATUS`, `0x0003`)

| Bit | Konstante | Bedeutung |
|---|---|---|
| 0 | `RSS_REGISTER_OK` | RSS-Registerzugriff erfolgreich |
| 1 | `CONFIG_CREATE_OK` | Konfigurationsobjekt erfolgreich erstellt |
| 2 | `SENSOR_CREATE_OK` | Sensorobjekt erfolgreich erstellt |
| 3 | `DETECTOR_CREATE_OK` | Detektorobjekt erfolgreich erstellt |
| 23 | `CONFIG_APPLY_ERROR` | Fehler beim Anwenden der Konfiguration |
| 24 | `SENSOR_CALIBRATE_ERROR` | Fehler bei der Sensor-Kalibrierung |
| 25 | `DETECTOR_CALIBRATE_ERROR` | Fehler bei der Detektor-Kalibrierung |
| 28 | `XM125_STATUS_DETECTOR_ERROR` (gemeinsam) | Allgemeiner Detektorfehler |
| 31 | `XM125_STATUS_BUSY` (gemeinsam) | Kommando läuft noch |

## Kommandos (Register `COMMAND`, `0x0100`)

| Kommando | Wert | Ausgelöst durch |
|---|---|---|
| `APPLY_CONFIG_AND_CALIBRATE` | 1 | `applyConfigAndCalibrate()`, `hardCalibration()` |
| `MEASURE_DISTANCE` | 2 | `startMeasurement()` |
| `APPLY_CONFIGURATION` | 3 | `applyConfiguration()` |
| `CALIBRATE` | 4 | `calibrate()` |
| `RECALIBRATE` | 5 | `recalibrate()` |

Dazu das gemeinsame Kommando `RESET_MODULE` (1381192737 = `0x52535421`), das
`XM125Base::resetModule()` sendet und `hardCalibration()` seinem
`APPLY_CONFIG_AND_CALIBRATE` voranstellt.
