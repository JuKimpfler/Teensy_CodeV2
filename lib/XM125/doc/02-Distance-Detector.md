# Distance-Detector (`XM125Distance`)

Header: [XM125Distance.h](../XM125Distance.h) · Implementierung: [XM125Distance.cpp](../XM125Distance.cpp)

Der Distance-Detector misst Entfernungen zu bis zu 10 Reflektoren
("Peaks") innerhalb eines konfigurierbaren Messfensters (`startMm` …
`endMm`) und liefert je Peak eine Distanz und eine Signalstärke.

## Typischer Ablauf

```cpp
XM125Distance radar; // Default-I2C-Adresse, kein WAKE_UP/INT verkabelt
radar.begin(Wire);

radar.config.startMm = 200;
radar.config.endMm   = 4000;
radar.applyConfigAndCalibrate();   // schreibt config + kalibriert (kann Sekunden dauern)

while (true) {
  if (radar.measure()) {
    for (uint8_t i = 0; i < radar.getNumDistances(); i++) {
      Serial.println(radar.getPeakDistanceRaw(i));
      Serial.println(radar.getPeakStrength(i));
    }
  }
}
```

Alle Konfigurationsfelder (`radar.config.*`) sind in
[04-Konfiguration.md](04-Konfiguration.md) im Detail beschrieben.

## Mehrere Sensoren parallel messen lassen

`measure()` blockiert über die **gesamte** Messdauer. Bei vier Sensoren
nacheinander summiert sich das entsprechend. Da die Wartezeit fast vollständig
aus Warten besteht, lässt sie sich überlappen — `measure()` ist dafür in drei
Teile zerlegt:

```cpp
for (auto& r : sensoren) r.startMeasurement();   // alle anstoßen

bool anyPending = true;                          // gemeinsam warten
while (anyPending) {
  anyPending = false;
  for (auto& r : sensoren) {
    bool done = false;
    if (r.pollMeasurement(done) && !done) anyPending = true;
  }
}

for (auto& r : sensoren) r.fetchResult();        // Ergebnisse abholen
```

`measure()` selbst ist genau so implementiert und bleibt unverändert nutzbar.

Damit senden die Sensoren einer Gruppe allerdings gleichzeitig — zum
Übersprechen siehe
[05-Fehlerbehandlung-und-Troubleshooting.md](05-Fehlerbehandlung-und-Troubleshooting.md#8-übersprechen-zwischen-mehreren-sensoren).
Im Projekt steuert `RADAR_PARALLEL_GROUP` in [params.h](../../include/params.h),
wie viele Sensoren zusammen messen; umgesetzt in `systemlib::readRadarGroup()`.

## API-Überblick

| Methode | Beschreibung |
|---|---|
| `writeConfig()` | Schreibt alle Werte aus `config` in die Konfigurationsregister. Bricht beim ersten fehlgeschlagenen Register-Write ab (fail-fast) |
| `getLastFailedConfigRegister()` | Registeradresse des zuletzt fehlgeschlagenen `writeConfig()`-Schreibvorgangs (`0xFFFF` = keiner) |
| `applyConfigAndCalibrate(timeoutMs = 3000)` | `writeConfig()` + Kommando `APPLY_CONFIG_AND_CALIBRATE`, blockiert bis fertig |
| `applyConfiguration(timeoutMs = 3000)` | Nur anwenden, ohne Neukalibrierung (z.B. nach Neustart mit noch gültiger alter Kalibrierung) |
| `calibrate(timeoutMs = 3000)` | Sensor/Detektor neu kalibrieren |
| `recalibrate(timeoutMs = 3000)` | Wie `calibrate()`, für den Fall dass `CALIBRATION_NEEDED` gesetzt ist |
| `measure(timeoutMs = 1000)` | Startet eine Messung, wartet bis fertig, liest Ergebnis + alle Peaks ein |
| `startMeasurement()` | Sendet nur das Messkommando und kehrt sofort zurück (wartet **nicht**) |
| `pollMeasurement(done)` | Fragt einmal ab, ob die Messung fertig ist — blockiert nicht |
| `fetchResult()` | Liest Ergebnisregister + Peaks der abgeschlossenen Messung ein |
| `getNumDistances()` | Anzahl gefundener Peaks (0–10) nach `measure()` |
| `getNearStartEdge()` | `true`, wenn ein Objekt sehr nah an `config.startMm` erkannt wurde |
| `getCalibrationNeeded()` | `true`, wenn der Sensor eine Neukalibrierung anfordert → `recalibrate()` aufrufen |
| `getMeasureError()` | `true`, wenn die letzte Messung einen Fehler hatte |
| `getTemperature()` | Sensortemperatur in °C (geringe Absolutgenauigkeit lt. Datenblatt) |
| `getPeakDistanceRaw(idx)` | Roh-Distanzwert des Peaks `idx` (0–255 "Counts") |
| `getPeakDistanceMm(idx)` | **Aktuell identisch zu `getPeakDistanceRaw()`** — siehe Warnung unten |
| `getPeakDistanceReg(idx)` | Ungekürztes 32-bit-Registerwort des Peaks `idx` — für die Kalibrierung, siehe Warnung unten |
| `getPeakStrength(idx)` | Signalstärke des Peaks `idx` (1000x-Einheit, RSS-intern) |

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
die richtige Kodierung; sie gehört dann nach `XM125Distance::measure()`.

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
| `APPLY_CONFIG_AND_CALIBRATE` | 1 | `applyConfigAndCalibrate()` |
| `MEASURE_DISTANCE` | 2 | `measure()` |
| `APPLY_CONFIGURATION` | 3 | `applyConfiguration()` |
| `CALIBRATE` | 4 | `calibrate()` |
| `RECALIBRATE` | 5 | `recalibrate()` |
