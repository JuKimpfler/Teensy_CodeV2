# RoutePlanner – Pfadplanung auf cm-Ebene

Die Schicht zwischen [`Pathfinder`](../Pathfinder/Pathfinder.md) (rechnet in Zellen) und
[`MotionProfile`](../MotionProfile/MotionProfile.md) (fährt die Punkte ab).

Dateien: `RoutePlanner.h`, `RoutePlanner.cpp`

```
Position, Ziel, Ball, Partner
        │
        ▼
Pathfinder::findPath()        A* auf PATH_GRID_X_CELLS × PATH_GRID_Y_CELLS
        │  Zellpfad
        ▼
Pathfinder::giveWaypoints()   Pfad auf wenige Ecken vereinfachen
        │
        ▼
RoutePlanner                  Zellen → cm, Teilstück-Längen        ← diese Lib
        │  Route (reine Geometrie)
        ▼
MotionProfile::setRoute()     Tempo je Teilstück aus dem Speed-Pattern
```

`RoutePlanner` liefert **nur Geometrie**. Welches Tempo daraus wird, rechnet
`MotionProfile` – deshalb kennt weder `Pathfinder` noch `RoutePlanner` die
Speed-Pattern. Im Hauptprogramm macht `systemlib::moveToPointAvoid()` beides;
diese Doku beschreibt die Klasse dahinter.

---

## Datentypen

```cpp
struct RouteRequest          // ein Planungsauftrag, einmal je Zyklus
{
    uint32_t NowMs;            // millis()
    float    FromX, FromY;     // aktuelle Position [cm]
    uint16_t ToX, ToY;         // Ziel [cm]
    bool     Arrived;          // true = am Ziel, dann keine zyklische Neuplanung mehr
};

enum RouteResult : uint8_t
{
    RouteUnchanged,  // keine Neuplanung fällig, die bisherige Route gilt weiter
    RouteNew,        // neu geplant, out enthält die Route
    RouteFailed      // neu geplant, aber kein Weg gefunden
};
```

`Route` und `RoutePoint` stehen in `MotionProfile.h`, weil `MotionProfile` sie verbraucht:

```cpp
struct RoutePoint { uint16_t X, Y; float SegmentCm; };  // SegmentCm = Länge des Stücks bis hierher
struct Route      { RoutePoint Points[ARRAY_MAX_POINTS]; uint8_t Count; float TotalCm; };
```

---

## Funktionen

| Funktion | Zweck |
|---|---|
| `void begin(Pathfinder &finder)` | Den A* anmelden. Einmal beim Start (`systemlib::initialize()` macht das mit `AStar`) |
| `void setObstacles(ballX, ballY, robotX, robotY)` | Hindernisse in cm, **−1 = kein Hindernis**. Zyklisch aufrufbar, vor `update()` |
| `RouteResult update(const RouteRequest&, Route &out)` | Plant, wenn es nötig ist. `out` wird nur bei `RouteNew` gefüllt |
| `void reset()` | Planung verwerfen; der nächste `update()` plant wieder |
| `bool giveFound()` | A* hat beim letzten Versuch einen Weg gefunden |
| `uint8_t giveWaypoints()` | Anzahl Wegpunkte der letzten Route |
| `uint32_t giveTimeUs()` | Rechenzeit der letzten Planung [µs] |

---

## Wann wird neu geplant?

| Auslöser | Verhalten |
|---|---|
| Neues Ziel (`ToX`/`ToY` geändert) oder nach `reset()` | sofort |
| Hindernis hat sich um mehr als `PATH_OBST_MOVE_CM` (8 cm) bewegt, ist neu oder weg | sofort |
| zyklisch alle `PATH_REPLAN_MS` (300 ms), solange der Roboter unterwegs ist | laufend |
| `RouteRequest.Arrived` ist `true` **und** ein Weg wurde gefunden | keine Neuplanung |
| letzte Planung gescheitert | erst nach `PATH_REPLAN_FAIL_MS` (1000 ms) wieder |

Ein geändertes Speed-Pattern löst **keine** Neuplanung mehr aus – die Geometrie
hängt nicht davon ab. Dass das Tempo in diesem Fall trotzdem von vorn anläuft,
regelt `MotionProfile::setRoute()` über seine eigene Bezugsstrecke. Gegenüber dem
alten Code in `systemlib` spart das einen A*-Lauf, das Fahrverhalten bleibt gleich.

---

## Wenn kein Weg gefunden wird

Der Reihe nach wird versucht:

1. mit Ball und Partner als Hindernis,
2. ohne Ball (das Ziel liegt oft im Ballbereich),
3. ohne Hindernisse (der Start liegt im Sperrbereich).

Scheitert auch das, liefert `update()` `RouteFailed` und `giveFound()` ist `false`.
Was dann passiert, entscheidet der Aufrufer – `systemlib::moveToPointAvoid()`
richtet sich nach `PATH_FALLBACK_DIRECT` (`true` = geradeaus per `moveToPoint()`,
`false` = stehen bleiben und den Winkel halten).

---

## Parameter

Alle in [`lib/include/params/params_path.h`](../include/include.md).

| Name | Default | Bedeutung |
|---|---|---|
| `PATH_REPLAN_MS` | 300 | Neuplanung, solange unterwegs |
| `PATH_REPLAN_FAIL_MS` | 1000 | Neuplanung nach gescheiterter Planung |
| `PATH_OBST_MOVE_CM` | 8.0 | Hindernisbewegung, die sofort neu plant |
| `PATH_PASS_CM` | 12 | Durchfahrradius der Zwischenpunkte (wird von `MotionProfile` gesetzt) |
| `PATH_GRID_X_CELLS` / `PATH_GRID_Y_CELLS` | 90 / 120 | Raster des A* |
| `PATH_NO_OBSTACLE` | 255 | Zellwert „kein Hindernis“ |
| `PATH_FALLBACK_DIRECT` | true | ohne Weg: geradeaus statt stehen (wertet `systemlib` aus) |

---

## Rechenzeit

`update()` ruft den A* nur auf, wenn wirklich neu geplant wird – in den übrigen
Zyklen kostet die Funktion nichts. Ein Planungslauf **blockiert** aber (Messwerte
siehe [Pathfinder.md](../Pathfinder/Pathfinder.md)); scheitert der erste Versuch,
kann es bis zu dreimal so lange dauern. Die Positionsfusion begrenzt ihr `dt` auf
50 ms, ein einzelner langer Zyklus bleibt also ohne Folgen.

---

## Stand und Grenzen

- Auf dem Roboter nicht erprobt. Gebaut und gegen den bisherigen Ablauf in
  `systemlib` Zeile für Zeile abgeglichen, aber nicht gefahren.
- Die Planung kennt nur Ball und **einen** weiteren Roboter. Ein zweiter Gegner
  wird nicht umfahren.
- Eine Instanz je Roboter: `Pathfinder` hält die Gewichtskarte (~130 kB) und wird
  per Referenz geteilt, nicht kopiert.
- Nächster Schritt laut [Plan_Navigation-Lib.md](../../docs/Plan_Navigation-Lib.md):
  zusammen mit `Positioning` und `MotionProfile` in ein eigenes Repo
  `Robotic_Navigation` und von dort nach `lib/Navigation/` vendoren.
