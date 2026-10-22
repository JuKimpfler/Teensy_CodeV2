# Plan: Positionierung und Pfadplanung als externe Lib

Stand: Umgesetzt sind der A* (`lib/Pathfinder`), `lib/RoutePlanner` und `lib/MotionProfile` (Schritte 0, 4 und 5). Offen ist die **Positionierung** (Schritte 2 und 3) und der Umzug aller vier Klassen in ein eigenes Repo `Robotic_Navigation` (Schritte 1 und 6).

## 1. Ausgangslage

Beides steckt heute in `lib/systemlib/systemlib.cpp` (~2300 Zeilen) und ist mit der
Roboter-Hardware verwoben.

| Teil | Wo | Größe | Kopplung an den Roboter |
|---|---|---|---|
| Radar-Validierung (`radarAxisMeasurements`, `goalOffset`) | systemlib.cpp | ~170 Z. | `RadarData` (aus RadarArray), `params` |
| Kalman-Fusion (`AxisFilter`, `fusionPredict/Update`, Historie) | systemlib.cpp | ~150 Z. | nur Zahlen |
| `calculatePosition()` | systemlib.cpp | ~200 Z. | `Radar[]`, `giveOrbitDir()`, `motorSampleUs` (global aus `updateMotors()`), `PDS.Channel()`, `Util.wrapAngle`, `lastvaliddata_*` |
| Odometrie (`calculateFieldVelocityFromMotorRpm`) | systemlib.cpp | ~40 Z. | Motor-Reihenfolge VL/VR/HL/HR |
| Speed-Pattern, `moveToPoint`, Punkt-Array | **erledigt:** `lib/MotionProfile` (Klasse, `MotionInput`/`MotionCommand` statt Zugriff auf `Position` und die Motoren) | ~480 Z. | keine |
| Pfadplanung (Route, Vereinfachen, Neuplanung) | **erledigt:** `lib/RoutePlanner` (Klasse, liefert nur Geometrie) | ~190 Z. | nur `Pathfinder` |
| A* selbst | **erledigt:** `lib/Pathfinder` (Klasse, Gewichtskarten in `params_pathmap.h`) | ~420 Z. | keine, nur Zellen und `params.h` |

Probleme, die der Schnitt lösen soll:
- ~~A* fehlte im Repo~~ – ist jetzt als `lib/Pathfinder` integriert, die stille Rückfallebene `PATH_AVAILABLE` gibt es nicht mehr.
- **Versteckter Zustand:** `static`-Variablen in `calculatePosition()`, anonyme Namespaces
  (`filtX`). Zwei Instanzen oder ein Unit-Test sind unmöglich, ein Reset geht nur teilweise.
  ~~`mv`, `pa`, `pf`~~ – sind jetzt Member von `MotionProfile` bzw. `RoutePlanner`.
- **Nicht testbar ohne Roboter:** reine Mathematik hängt an `PDS`, `millis()` und `BNO`.
- **Datenübergabe über Globals:** `motorSampleUs` verbindet `updateMotors()` und die Fusion unsichtbar.

## 2. Zielbild

Ein eigenes Repo **`Robotic_Navigation`** (Muster wie `Robotic_PDS`), im Roboter-Repo vendored
unter `lib/Navigation/`, aktualisiert mit `tools/sync_nav.py` (Kopie von `sync_pds.py`).
Die Lib kennt **weder** `systemlib` **noch** RadarArray, BNO, PDS oder Motoren. Sie bekommt Zahlen
und liefert Zahlen.

```
Robotic_Navigation/            ->  lib/Navigation/ im Roboter-Repo
├── library.json
├── Positioning.h / .cpp       Radar-Validierung + Odometrie + Kalman  (Klasse Positioning)
├── Pathfinder.h  / .cpp       A* auf Zellen (existiert bereits als lib/Pathfinder)
├── RoutePlanner.h / .cpp      cm <-> Zellen, Hindernisse, Replan-Logik, Route (Klasse RoutePlanner, nutzt Pathfinder)
├── MotionProfile.h / .cpp     Speed-Pattern + Punkt-Array + Fortschritt (Klasse MotionProfile)
├── params_Nav.h               Defaults aller Werte (Vorlage, siehe 4.)
├── Navigation.md              Doku im Stil der anderen Libs
├── examples/                  Positioning_Replay.cpp, Pathfinder_Demo.cpp (vorhanden)
└── test/                      native Unit-Tests (PlatformIO env:native)
```

Abweichung von der Regel „genau 2 Dateien je Lib": eine Lib mit vier Klassen. Alternative wären Mini-Libs (`Positioning`, `Pathfinder`, `RoutePlanner`, `MotionProfile`) – sauberer nach Regel, aber mehrere Repos und Syncs.
**Empfehlung:** ein Repo, je Klasse zwei Dateien; Regel in `Coding_Rules.md` auf „je Klasse 2 Dateien" präzisieren.
`MotionProfile` ist optional für Phase 2 (siehe 6).

## 3. Schnittstellen (Entwurf, Stil wie RadarArray: `begin/update/calib`, `give/set`, `_privat`)

Reine Eingabe-Structs statt Zugriff auf Hardware-Klassen:

```cpp
struct PosRadar   { uint16_t Dist, DistRaw; int32_t Strenght; uint8_t Peaks; bool isCon; uint32_t Time; };
struct PosInput                       // ein Zyklus
{
    uint32_t NowMs, NowUs;
    float    HeadingDeg;              // +-180 CW, Coding_Rules
    float    VxField, VyField;        // cm/s, aus Odometrie
    uint32_t MotorSampleUs;           // ersetzt das globale motorSampleUs
    PosRadar Vo, Re, Hi, Li;          // statt RadarArray
    bool     UseRadar = true, UseOdometry = true;
};
struct PositionData { float X, Y, Certainty; bool XValid, YValid, AtGoal; };   // unveraendert

class Positioning
{
public:
    void begin(float startX, float startY);
    PositionData update(const PosInput &in);        // ersetzt calculatePosition(), blockiert nie
    void reset(float x, float y);                   // setzt Filter UND Historie zurueck
    PositionData givePosition() const;
    void giveDebug(PosDebug &d) const;              // Innovation, Sigma, Latenzschaetzung -> systemlib reicht an PDS weiter
    void setConfig(const PosConfig &c);             // Laufzeit-Tuning, Default = params_Nav.h
};
```

Hilfsfunktion in systemlib bleibt als Adapter: `Radar[...]` + BNO + Motor-RPM in `PosInput` packen,
`update()` aufrufen, `Position`/PDS-Kanäle 190–197 füllen. Das ist der gesamte Kleber (~40 Zeilen).

`Pathfinder` (Zellen, existiert: [lib/Pathfinder](../lib/Pathfinder/Pathfinder.md)) bleibt unverändert. Neu kommt
der Teil, der heute noch in `systemlib.cpp` steckt (`buildRoute`, `obstacleMoved`, `PathPlan`):

```cpp
class RoutePlanner                               // cm-Ebene ueber Pathfinder
{
public:
    void begin(Pathfinder &finder);
    void setObstacles(int16_t ballX, int16_t ballY, int16_t robX, int16_t robY);     // cm, -1 = keins
    bool update(float fromX, float fromY, uint16_t toX, uint16_t toY, uint32_t nowMs, Route &out);   // Replan-Logik (Zeit/Hindernis bewegt)
    bool giveFound() const;  uint8_t giveWaypoints() const;  uint32_t giveTimeUs() const;
};
```

`Route` = `ArrayPoint`-kompatible Liste. Das Tempo je Teilstück (Pattern über die ganze Strecke) rechnet
`MotionProfile`; `RoutePlanner` liefert nur Geometrie. Dadurch hängt weder `Pathfinder` noch `RoutePlanner` an `SPattern`.

## 4. Parameter

Regel „Parameter an einem Ort" gilt weiter, aber eine externe Lib braucht eigene Defaults:

- `params_Nav.h` in der Lib enthält **alle** Defaults (heutige `ODO_/POS_/FUS_/FIELD_/PATH_/ARRAY_`-Werte,
  mit `#ifndef`-Schutz bzw. `static constexpr` in einem `namespace nav_default`).
- Das Roboter-Repo überschreibt sie in `lib/include/params/params_position.h` usw. (Pathfinder-Raster und -Kosten stehen schon dort: `params_path.h`) über eine
  `PosConfig`-Initialisierung (z. B. `PosConfig cfg = Positioning::defaultConfig(); cfg.FusSigmaRadarCm = FUS_SIGMA_RADAR_CM;`).
  Damit bleibt **die Roboter-Datei die einzige, die jemand anfasst**, die Lib-Defaults sind Vorlage
  (wie `params_PDS.h` / `channel_config.h` bei PDS).
- Roboter-spezifisch und daher **nicht** in die Lib: Radar-Abstimmung (`RADAR_*`), Pins, Motor-Zuordnung,
  Kurs-Regler (`HDG_*`).
- Laufzeit-Config (`PosConfig`) statt reiner Compile-Zeit-Konstanten ist die Voraussetzung dafür,
  Werte aus der PDS-GUI zu tunen.

## 5. Konkrete Arbeitsschritte (Reihenfolge, jeweils bauen + auf dem Roboter gegenprüfen)

| # | Schritt | Ergebnis / Prüfung |
|---|---|---|
| 0 | ~~A*-Quellen beschaffen~~ **erledigt:** als `lib/Pathfinder` integriert (Klasse statt globalem Zustand, Differenztest gegen das Original bestanden). Übrig: Lizenz/Autor klären, auf dem Roboter `givePathFound()` und `givePathTimeUs()` prüfen | `moveToPointAvoid()` plant wirklich |
| 1 | Repo `Robotic_Navigation` anlegen, CI wie hier (`pio run`, `pio check`) plus `env:native`. | grüne Pipeline |
| 2 | **Positioning herauslösen**: `static`-Zustand und `filtX/filtY` in Klassenmember, `PosInput` einführen, `PDS.Channel` und `giveOrbitDir()` entfernen. Mathematik **unverändert**. | Replay-Test: aufgezeichnete Eingaben (PDS-Log) liefern vor und nach der Auslagerung dieselben Positionen (±1e-4 cm) |
| 3 | Adapter in `systemlib::readSensors()`; alte Funktionen löschen; `motorSampleUs`-Global durch `PosInput.MotorSampleUs` ersetzen. | Roboter fährt Referenzstrecke, Certainty und Innovation (PDS 192–197) wie vorher |
| 4 | **RoutePlanner** auslagern (`buildRoute`, `obstacleMoved`, Replan-Logik; `simplifyPath`/`lineClear` sind schon in `Pathfinder`); Speed-Pattern für Zwischenpunkte über `MotionProfile`. | `moveToPointAvoid()` Verhalten gleich, `givePathTimeUs()` nicht schlechter |
| 5 | (Phase 2) `MotionProfile` mit `moveToPoint`-Profil und Punkt-Array; systemlib behält nur `setOmniWheelDir`. | Array-Fahrt (`eck_route0/45` aus `src/`) wie vorher |
| 6 | `tools/sync_nav.py`, `lib/Navigation/library.json`, Doku `Navigation.md`, `include.md` ergänzen. | `python tools/sync_nav.py --check` ohne Drift |

## 6. Tests

- **Native Unit-Tests** (`env:native`, kein Teensy): Radar-Paar passt / widerspricht / Blindbereich, Tor-Band,
  Gate und Re-Akquise, Historie/Latenz, Reset; A*: freies Feld, Hindernis, Start in Sperrzone, Vereinfachen
  (nie durch teurere Zellen), Replan-Auslöser; Speed-Pattern: alle 19 Kurven bei p = 0, 0.5, 1 in 0…1.
- **Replay:** PDS-Aufzeichnung (Radar, Heading, RPM) als CSV in `Positioning_Replay.cpp` abspielen. Das ist der
  Beleg für „Auslagerung ändert nichts" (Schritt 2).

## 7. Risiken / offene Fragen

1. **A* liegt jetzt im Roboter-Repo** (`lib/Pathfinder`). Beim Umzug ins `Robotic_Navigation`-Repo bleibt es eine eigene Klasse; die Gewichtskarte (`params_pathmap.h`) ist Roboter-/Feldwissen und bleibt beim Roboter.
2. **Heap/RAM:** Gewichtskarte 90 × 120 = 10,8 kB und Pfad-Puffer; die Lib darf nur statisch allozieren
   (bisher auch so). Teensy 4.0 hat genug RAM, aber Instanzen nicht mehrfach anlegen.
3. **Mehrere Instanzen:** Aktuell gibt es genau einen Roboter-Zustand. Bei zwei Positioning-Instanzen
   (z. B. Partner-Roboter schätzen) muss die Gewichtskarte geteilt werden.
4. **Zeitbasis:** `NowMs`/`NowUs` kommen von außen – im Replay-Test frei wählbar, auf dem Roboter `millis()/micros()`.
5. **Einheiten:** Heading ±180° CW, Koordinaten cm ab Ecke eigenes Tor (Coding_Rules) gelten in der Lib unverändert.
6. **Regel „2 Dateien je Lib"** (siehe 2) – Entscheidung nötig.
7. Rechenzeit: vor und nach der Auslagerung `giveCycleTime_us()` auf dem Roboter vergleichen (am Rechner nicht messbar).
