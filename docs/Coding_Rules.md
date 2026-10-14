# Coding Rules

## Specifications

### Angle
- angle = +-180 CW
- Sensor Positionen = 0-x CW

### Koordinaten
- Koordinaten = links unten beim eigenen Tor, in cm, bis zur Wand (0-182 / 0-243).
  Für x/y relevant ist 14-168 / 14-229: die Roboterposition gilt vom Mittelpunkt.
- Die Feldmaße stehen einmal in `lib/include/params/params_field.h`.

### Single Numbers
- Speed (-100) - 100 CW
- linedepth 0-1 max=1

### Abkürzungen
- Dir -> Direction
- Pos -> Position
- Calib -> Calibrate
- Var -> Variables
- IR -> Infrarot
- US -> Ultraschall
- us -> microsekunden
- ms -> millissekunden
- sek -> sekunden
- DBG -> Debugger

## Aufbau

- **Hauptlib `systemlib`:** Zugriff auf alle Sensorwerte und alle Aktuatoren.
- **Sublibs:** Ordner = Name der Klasse, genau zwei Dateien `X.h` + `X.cpp`
  (dazu `X.md` als Doku). Eine Sublib kennt `systemlib` nicht.
- **Parameter:** nur in `lib/include/params/` (Übersicht: [include.md](../lib/include/include.md)).
  Keine `#define`/`constexpr`-Werte im `.cpp` oder in Klassen. Ausnahme: Register-
  und Protokollkonstanten eines Chips bleiben in dessen Treiber.
- **Pins:** nur in `lib/include/enum.h`.

## Namen

| Was | Schreibweise | Beispiel |
|---|---|---|
| Klasse, Struct, Enum | PascalCase | `RadarArray`, `PositionData`, `ArrayState` |
| Methode | camelCase | `updateArray()`, `moveToPoint()` |
| Lesen / Schreiben | `giveXxx()` / `setXxx()` | `givePosition()`, `setHeading()` |
| Wahrheitswert abfragen | `isXxx()` oder `giveXxx()` | `isPressed()`, `giveCon()` |
| Lebenszyklus | `begin()` · `update()` · `calib()` | `Radar.begin()`, `AStar.begin()` |
| Hilfsfunktion | camelCase, Verb zuerst | `wrapAngle()`, `inRange()` |
| privates Member / privater Helfer | `_name` | `_sensors`, `_startRound()` |
| Struct-Feld | PascalCase | `Position.X`, `Radar[RadarVo].Dist` |
| Parameter | `PRÄFIX_NAME_EINHEIT` | `FUS_REACQ_MS`, `FIELD_X_CM` |
| Enum-Wert | `Typ_Wert` | `SPattern_Trapezoid` |

Einheiten-Suffix am Parameter: `_CM`, `_MM`, `_MS`, `_S`, `_DEG`, `_PCT`, `_MDB`.
Präfixe: `RADAR_` · `FIELD_` · `ODO_` · `POS_` · `FUS_` · `MOVE_` · `ARRAY_` ·
`PATH_` · `HDG_` · `LINE_`.

## Stil

- 4 Leerzeichen, keine Tabs, UTF-8, LF, Datei endet mit Zeilenumbruch.
- Klammern in eigener Zeile (Allman), `#pragma once` in jedem Header.
- Formatiert wird mit `clang-format` (Regeln in `.clang-format`); `lib/PDS` ist
  ausgenommen, weil es per `tools/sync_pds.py` aus Robotic_PDS kopiert wird.
  Vor dem Commit: `clang-format -i <Datei>`.
- Nichts Blockierendes im Roboterzyklus (kein `delay()`, kein Warten auf Sensoren).
- Bus-Zugriffe sparen: jeden Chip pro Zyklus **einmal** lesen und die Werte weiterreichen (`TCA9534DW::update()` + `givePinState()`), nie
  dieselbe Quelle mehrfach abfragen. Hintergrund und Zahlen: [Laufzeit_Optimierung.md](Laufzeit_Optimierung.md).

## Eigene Libs und Treiber

`give`/`set`/`begin` gilt für alle **eigenen** Libs (`systemlib`, `RadarArray`, `Pathfinder`, `Cam`, `ADC108S102`,
`TCA9534DW`, `fancyBTN`, `commlib`, `Utils`, `BNO055`-Start). Davon ausgenommen sind Treiber, die der
API ihres Chips bzw. Herstellers folgen und in sich einheitlich sind:

| Lib | Stil | Grund |
|---|---|---|
| `XM125` (`getXxx`) | `get` liest Register, `give` liefert gespeicherte Werte | liest vom Chip, siehe `XM125/doc/08-API-Referenz.md` |
| `SRF08` (`getXxx`), `INA219`, `PMW3389` | wie Original | Fremdcode bzw. eigene Doku |
| `BNO055` (`quaData…`, `eulHeading`) | Datenzugriffe ohne Präfix | Sensortreiber, nur `init` → `begin` |
| `IR` (`ir_ring.h`) | eigener Stil (`IR_ring`, `Device_handle`) | von anderer Hand geschrieben, Umbenennen ohne Roboter-Test zu riskant |
| `PDS` | wie Robotic_PDS | Kopie, wird per `tools/sync_pds.py` überschrieben |
| `systemlib` | Klasse klein geschrieben | Name der Hauptlib, seit Beginn so |

Neue Libs richten sich nach der Tabelle oben, nicht nach diesen Ausnahmen.
