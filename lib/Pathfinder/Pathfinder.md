# Pathfinder – A*-Wegsuche auf dem Feldraster

Findet einen Weg über das Spielfeld, der Zellen mit hohem Gewicht (Wand, Tor, Ball, anderer Roboter) meidet.
Die Klasse kennt weder Position noch Motoren: Sie rechnet in **Rasterzellen**. Die Umrechnung von und nach cm
und das Abfahren des Wegs macht [`systemlib::moveToPointAvoid()`](../systemlib/Bewegungsfunktionen.md#7-pfadplanung-movetopointavoid).

```cpp
#include <Pathfinder.h>

Pathfinder AStar;                 // global! Das Objekt ist ~130 kB groß

AStar.begin();
if (AStar.findPath(sx, sy, tx, ty, ballX, ballY, robX, robY))
{
    uint8_t wp[16][2];
    uint8_t n = AStar.giveWaypoints(wp, 16);   // wenige Wegpunkte statt hunderter Zellen
}
```

## Raster und Gewichte

| | |
|---|---|
| Raster | `PATH_GRID_X_CELLS` × `PATH_GRID_Y_CELLS` = 90 × 120 Zellen, eine Zelle ≈ 2 cm; Index `[x][y]`, x = kurze Feldachse |
| Gewichtskarte | `PATH_WEIGHTS_FIELD` in `lib/include/params/params_pathmap.h` (Flash) |
| Masken | `PATH_WEIGHTS_BALL`, `PATH_WEIGHTS_ROBOT` (32 × 32): werden vor jeder Suche mittig auf Ball bzw. Roboter gelegt, der Roboter überdeckt den Ball |
| Gewichte | 0 … 200, **100 = neutral**, kleiner = bevorzugt, größer = teurer, **255 = nicht begehbar** |
| Schrittkosten | `PATH_COST_STRAIGHT` 10, `PATH_COST_DIAGONAL` 14 |

Die Gewichtskarte gehört zu den Parametern: anpassen heißt `params_pathmap.h` ändern, ansehen kann man das Ergebnis
mit `tools/pathfinder_gui`. Die Datei wird **nur** von `Pathfinder.cpp` eingebunden, damit die Tabellen einmal im Flash liegen.

## Funktionen

| Funktion | Zweck |
|---|---|
| `begin()` | Karte laden, Zustand löschen. Vor der ersten Suche |
| `findPath(sx, sy, tx, ty, ballX, ballY, robotX, robotY)` | Weg suchen. Ball/Roboter = Zelle des Hindernisses, `PATH_NO_OBSTACLE` = keins. `true` bei Erfolg |
| `givePathLength()`, `givePathX(i)`, `givePathY(i)` | Zellpfad inkl. Start und Ziel |
| `giveWaypoints(wp, max)` | Pfad auf wenige Wegpunkte vereinfachen (siehe unten) |
| `lineClear(x0, y0, x1, y1, costLimit)` | ist die Gerade frei und nirgends teurer als `costLimit`? |
| `giveWeight(x, y)` | Gewicht inkl. der Masken der letzten Suche, außerhalb des Rasters 255 |
| `giveWeights()`, `givePath()` | die Rohfelder, für `PathfinderTelemetry` |
| `giveHeapPeak()` | höchster Füllstand der offenen Liste, zum Prüfen von `PATH_HEAP_CAPACITY` |

`findPath()` liefert `false`, wenn kein Weg existiert, eine Koordinate außerhalb des Rasters liegt, der Weg länger als
`PATH_MAX_CELLS` (600) wäre oder die offene Liste voll lief (`PATH_HEAP_CAPACITY`, im Test höchstens 3043 von 6000).
Das Ziel zählt immer als Gewicht 0, auch wenn es in einer Maske liegt.

### Vereinfachen

`giveWaypoints()` springt von jedem Ankerpunkt so weit voraus (höchstens `PATH_LOS_MAX_CELLS`), wie eine freie Gerade
möglich ist. Die Gerade darf nirgends teurer sein als die teuerste Zelle des A*-Wegs auf diesem Stück, die Gewichte
bleiben also wirksam. Ergebnis: ohne Startzelle, der letzte Punkt ist die Zielzelle.

## Speicher und Zeit

| | |
|---|---|
| RAM | ca. 132 kB (`sizeof(Pathfinder)`: Arbeitskarte, Kostenfelder, offene Liste, Pfad) |
| Flash | ca. 13 kB Gewichtskarten |
| Rechenzeit | wenige ms, blockiert (Messwerte siehe Abschnitt *Rechenzeit* in Bewegungsfunktionen.md). Nicht im Regelzyklus unbegrenzt oft aufrufen |

Das Objekt muss global oder statisch angelegt werden, `systemlib` hat dafür `AStar`.

## Telemetrie und Oberfläche

`PathfinderTelemetry.h` schickt Karte, Weg und Metadaten binär über `Serial`; die PC-Oberfläche
[`tools/pathfinder_gui`](../../tools/pathfinder_gui/README.md) zeigt sie als Heatmap. Beispiele:
`examples/Pathfinder_Demo.cpp` (Ausgabe über die serielle Konsole) und `examples/Pathfinder_Gui.cpp` (Telemetrie).

## Geprüft

Neue Klasse und das ursprüngliche Pathfinding-Projekt liefern auf dem Rechner bei 30 000 zufälligen Suchen
(inkl. Start = Ziel, Ball im Weg, Start in gesperrter Zelle) **dieselben** Wege, dieselben Karten (inkl. Masken),
dieselben Wegpunkte und dieselben `lineClear`-Ergebnisse; Lauf auch mit AddressSanitizer/UBSan ohne Befund.
Neu gegenüber dem Original: Koordinaten werden geprüft, eine volle offene Liste bricht die Suche ab, statt hinter das Feld zu schreiben.
