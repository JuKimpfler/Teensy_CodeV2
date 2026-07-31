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
| `getNumDistances()` | Anzahl gefundener Peaks (0–10) nach `measure()` |
| `getNearStartEdge()` | `true`, wenn ein Objekt sehr nah an `config.startMm` erkannt wurde |
| `getCalibrationNeeded()` | `true`, wenn der Sensor eine Neukalibrierung anfordert → `recalibrate()` aufrufen |
| `getMeasureError()` | `true`, wenn die letzte Messung einen Fehler hatte |
| `getTemperature()` | Sensortemperatur in °C (geringe Absolutgenauigkeit lt. Datenblatt) |
| `getPeakDistanceRaw(idx)` | Roh-Distanzwert des Peaks `idx` (0–255 "Counts") |
| `getPeakDistanceMm(idx)` | **Aktuell identisch zu `getPeakDistanceRaw()`** — siehe Warnung unten |
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
- Zum Kalibrieren: Ein einzelnes Objekt auf **exakt bekannter Distanz**
  platzieren (sonst nichts in Reichweite lassen) und den von
  `getPeakDistanceRaw()` gelieferten Rohwert gegen die bekannte Distanz
  notieren, um den Skalierungsfaktor empirisch zu bestimmen.

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
