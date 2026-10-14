# Bewegungsfunktionen (`systemlib`)

Dokumentation aller Funktionen, mit denen der Roboter zu Punkten, über Routen und um Hindernisse herum fährt.
Alle Funktionen sind Teil der Klasse `systemlib` und werden **zyklisch** aufgerufen (jeden Programmdurchlauf).

**Inhalt**

1. [Überblick](#1-überblick)
2. [Koordinaten, Einheiten, Voraussetzungen](#2-koordinaten-einheiten-voraussetzungen)
3. [Grundfunktionen (bestehend)](#3-grundfunktionen-bestehend)
4. [`moveToPoint` mit Speed-Pattern](#4-movetopoint-mit-speed-pattern)
5. [Speed-Pattern](#5-speed-pattern)
6. [Punkt-Array (Routen)](#6-punkt-array-routen)
7. [Pfadplanung (`moveToPointAvoid`)](#7-pfadplanung-movetopointavoid)
8. [Parameter in `params.h`](#8-parameter-in-paramsh)
9. [Beispiele](#9-beispiele)
10. [Fehlersuche](#10-fehlersuche)
11. [Stand und Grenzen](#11-stand-und-grenzen)

---

## 1. Überblick

Die Funktionen bauen aufeinander auf. Jede höhere Schicht benutzt die darunterliegende:

```
moveToPointAvoid()      Pfadplanung: A* um Ball und Partner herum
        │  erzeugt Wegpunkte
        ▼
Punkt-Array             setPointArray / startArray / updateArray ...
        │  fährt Punkt für Punkt, schaltet weiter
        ▼
moveToPoint()           ein Ziel, Speed-Pattern, Zielkreis, Winkel
        │
        ▼
setOmniWheelDir()       Richtung + Speed + Drehung → 4 Motoren
```

| Ebene | Wann nutzen |
|---|---|
| `moveToPoint` | Ein Ziel, freie Bahn |
| Punkt-Array | Feste Route aus mehreren Punkten (Eckenfahrt, Aufstellung, Patrouille) |
| `moveToPointAvoid` | Ein Ziel, aber Ball oder Partner können im Weg sein |

Die Position kommt aus `calculatePosition()` (Odometrie + Radar, siehe Kalibrier-Checkliste).

---

## 2. Koordinaten, Einheiten, Voraussetzungen

| Größe | Einheit / Bedeutung |
|---|---|
| Position X, Y | cm, Feldkoordinaten. X = kurze Achse (0 … `FIELD_X_CM`), Y = lange Achse (0 … `FIELD_Y_CM`) |
| Speed | Prozent, 0 … 100 |
| Andrehwinkel | Grad, `int16_t`, Soll-Heading des Roboters (gleiches System wie der BNO, ±180°) |
| Fahrtrichtung | Grad, 0° = in +Y-Richtung, positive Winkel drehen gegen den Uhrzeigersinn (+X entspricht −90°) |
| `setHeading()` | **Radiant!** Die Funktionen unten rechnen `Andrehwinkel · DEG_TO_RAD` selbst um |

**Voraussetzungen, die immer erfüllt sein müssen**

- `readSensors()` läuft jeden Zyklus (liefert `Position`).
- `updateHeadingController()` läuft jeden Zyklus (Heading-Regler), wie bisher.
- `Position.X` oder `Position.Y` = **−1** bedeutet „Position verloren“. Alle Bewegungsfunktionen halten dann an und regeln nur den Winkel weiter.

---

## 3. Grundfunktionen (bestehend)

Diese Funktionen gab es schon, die neuen bauen darauf auf.

| Funktion | Zweck |
|---|---|
| `setOmniWheelDir(Richtung°, Speed%, omega)` | Fährt in eine **Feldrichtung** (der Roboter wird um das aktuelle Heading zurückgerechnet), `omega` = Drehanteil vom Heading-Regler |
| `setOmniWheelVek(vx, vy, omega)` | Wie oben, aber mit Geschwindigkeitsvektor im Roboter-Frame |
| `moveDir(Richtung, Speed, Andrehwinkel)` | Fährt in eine Richtung; näher als 20 cm an der Wand wird der Speed auf 5 % begrenzt |
| `moveToPoint(X, Y, Speed, Andrehwinkel)` | Alte 4-Argument-Variante: Speed nimmt linear mit dem Abstand ab (voller Speed ab 40 cm), unter 5 cm Stopp |
| `setHeading(rad)`, `giveHeadingController()` | Soll-Winkel setzen, Reglerausgang lesen |

---

## 4. `moveToPoint` mit Speed-Pattern

```cpp
void moveToPoint(uint16_t X, uint16_t Y,
                 uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed,
                 int16_t Andrehwinkel);
```

Fährt zum Punkt (X, Y) und dreht dabei auf `Andrehwinkel`. **Jeden Zyklus aufrufen.**

| Parameter | Bedeutung |
|---|---|
| `X`, `Y` | Ziel in cm (wird auf das Feld begrenzt) |
| `SpeedPattern` | `SPattern_*` aus `params.h`, siehe [Abschnitt 5](#5-speed-pattern) |
| `MaxSpeed` | Speed in % bei Pattern-Wert 1 (max. 100) |
| `MinSpeed` | Speed in % bei Pattern-Wert 0 (Mindestfahrt, wird auf `MaxSpeed` begrenzt) |
| `Andrehwinkel` | Soll-Heading in Grad, wird die ganze Fahrt über geregelt |

### Wie der Speed berechnet wird

```
Fortschritt  p = 1 − (Restabstand / Startabstand)     0 am Start, 1 im Ziel
Speed        = MinSpeed + (MaxSpeed − MinSpeed) · f(p)
```

`f(p)` ist der Verlauf des gewählten Patterns (0 … 1). Das Pattern gibt nur die **Form** vor, die Skalierung kommt aus Min und Max.

### Neustart des Patterns

Das Pattern beginnt wieder bei p = 0 (Startabstand wird neu erfasst), wenn

- sich **Ziel X/Y**, **Pattern**, **MaxSpeed** oder **MinSpeed** gegenüber dem letzten Aufruf ändern,
- die Funktion **länger als `MOVE_RESTART_GAP_MS`** (300 ms) nicht aufgerufen wurde,
- `resetMoveProfile()` aufgerufen wird,
- die Position verloren war.

Wiederholte Aufrufe mit unveränderten Werten setzen den Verlauf fort. `Andrehwinkel` löst **keinen** Neustart aus, er darf sich während der Fahrt ändern.

### Ziel, Zielkreis, Nachkorrektur

- **Zielkreis `MOVE_ARRIVE_CM`** (4 cm): Darin ist der Speed 0, `giveMoveArrived()` liefert `true`. Der Winkel wird weiter geregelt.
- **Hysterese `MOVE_ARRIVE_HYST_CM`** (3 cm): Wird der Roboter weiter als Zielkreis + Hysterese weggeschoben, fährt er mit `MinSpeed` zurück (Feinkorrektur, kein Pattern).
- **Weicher Anlauf `MOVE_MAX_ACCEL_PCT_S`** (600 %/s): Der Speed darf höchstens so schnell **steigen**. Bremsen ist nie begrenzt. Nach einer Aufrufpause beginnt der Anlauf bei 0.
- **Position verloren (−1):** Stopp, Profil wird verworfen, Winkel wird weiter geregelt.

### Abfragen und Reset

| Funktion | Ergebnis |
|---|---|
| `bool giveMoveArrived()` | `true` im Zielkreis |
| `float giveMoveProgress()` | Fortschritt p, 0 … 1 |
| `void resetMoveProfile()` | Nächster Aufruf startet das Pattern von vorn |

---

## 5. Speed-Pattern

Die Nummern stehen als `enum SPattern` in `params/params_motion.h`. Ein unbekanntes Pattern fährt konstant mit `MaxSpeed`.

| Nr. | Name | Verlauf f(p) | Verhalten |
|---|---|---|---|
| 0 | `SPattern_Linear` | 1 − p | Max am Start, linear bremsen bis Min am Ziel |
| 1 | `SPattern_LinearUp` | p | Min am Start, linear beschleunigen bis Max am Ziel |
| 2 | `SPattern_Constant` | 1 | Konstant Max, Halt erst im Zielkreis |
| 3 | `SPattern_Trapezoid` | 20 % hoch, Reise, 40 % runter | Klassisches Trapez |
| 4 | `SPattern_Triangle` | 1 − \|1 − 2p\| | Hoch bis zur Mitte, dann runter |
| 5 | `SPattern_QuadDecel` | (1 − p)² | Früh bremsen, sanfter Auslauf |
| 6 | `SPattern_ConstDecel` | √(1 − p) | Konstante Verzögerung, spät bremsen |
| 7 | `SPattern_SmoothStep` | 1 − smoothstep(p) | Weiche S-Kurve runter |
| 8 | `SPattern_Cosine` | ½ (1 + cos πp) | Cosinus-Rampe runter |
| 9 | `SPattern_Sine` | sin πp | Weich hoch, Spitze in der Mitte, weich runter |
| 10 | `SPattern_SCurve` | weich hoch (25 %), Reise, weich runter | Ruckfrei an Start und Ziel |
| 11 | `SPattern_LateBrake` | 1 bis 70 %, dann Cosinus runter | Vollgas, spät bremsen |
| 12 | `SPattern_FastThenCrawl` | 1 bis 50 %, runter bis 70 %, dann 0 | Schnell hin, letzte 30 % Schleichfahrt |
| 13 | `SPattern_QuadUp` | p² | Anlauf, am Ziel schnell (Rammen) |
| 14 | `SPattern_CubicUp` | p³ | Noch späteres Beschleunigen |
| 15 | `SPattern_Expo` | exponentiell abklingend | Starker Abfall am Anfang |
| 16 | `SPattern_Stairs` | 4 Stufen: 100 / 67 / 33 / 0 % | Gestuftes Bremsen |
| 17 | `SPattern_Wave` | (1 − p)·(0,6 + 0,4·sin 6πp) | Pulsierend, klingt ab |
| 18 | `SPattern_Pulse` | 5 Perioden Max / Min | Wechsel zwischen Max und Min |
| – | `SPattern_COUNT` | | Anzahl der Pattern (19) |

Hinweise zur Wahl:

- Soll der Roboter **sicher am Ziel stehen**, nimm ein Pattern, das gegen Ende gegen 0 geht (`Linear`, `ConstDecel`, `SCurve`, `Cosine`).
- `Constant`, `QuadUp`, `CubicUp`, `Pulse` bremsen **nicht** vor dem Ziel, sie halten erst im Zielkreis an.
- `Trapezoid` und `SCurve` starten bei `MinSpeed`, `MinSpeed` darf dort also nicht 0 sein, sonst steht der Roboter.

---

## 6. Punkt-Array (Routen)

Eine Liste von Zielpunkten, die nacheinander angefahren werden. Jeder Punkt hat eigenes Ziel, eigenen Winkel und eigenes Speed-Pattern.

### Datentypen

```cpp
struct ArrayPoint {
    uint16_t X;            // Ziel X [cm]
    uint16_t Y;            // Ziel Y [cm]
    int16_t  Andrehwinkel; // Soll-Heading [Grad]
    uint8_t  SpeedPattern; // SPattern_*
    uint8_t  MaxSpeed;     // % bei Pattern-Wert 1
    uint8_t  MinSpeed;     // % bei Pattern-Wert 0
    uint16_t HoldMs;       // Wartezeit nach Erreichen [ms]
    uint8_t  PassCm;       // >0: Durchfahrpunkt (kein Halt), 0 = anhalten
};

enum ArrayState : uint8_t { ArrayIdle, ArrayRunning, ArrayPaused, ArrayFinished };
```

Ein `ArrayPoint` wird in dieser Reihenfolge initialisiert: `{ X, Y, Winkel, Pattern, Max, Min, Wartezeit, PassCm }`. Weggelassene Felder am Ende sind 0 (`HoldMs = 0`, `PassCm = 0` = Haltepunkt).

### Funktionen

| Funktion | Verhalten |
|---|---|
| `bool setPointArray(const ArrayPoint* points, uint8_t count, bool loop = false)` | Ersetzt die Liste. **Zyklisch aufrufbar:** unveränderte Liste = nichts passiert. Bei Änderung beginnt sie wieder beim ersten Punkt. Liefert `false`, wenn `count` größer als `ARRAY_MAX_POINTS` ist |
| `bool setPointArray(const ArrayPoint (&points)[N], bool loop = false)` | Wie oben, die Anzahl wird aus dem Array bestimmt |
| `bool addPointToArray(X, Y, Winkel, Pattern, Max, Min, HoldMs = 0)` | Hängt einen Punkt an. Eine laufende Fahrt wird nicht unterbrochen. War die Liste beendet, fährt der Roboter zum neuen Punkt weiter. `false` bei voller Liste |
| `bool addPointToArray(const ArrayPoint& point)` | Wie oben mit fertigem Punkt |
| `void resetArray()` | Leert die Liste, Zustand `ArrayIdle` |
| `void startArray(bool restart = false)` | Startet beim ersten Punkt. **Zyklisch aufrufbar:** läuft sie schon, ist sie pausiert oder beendet, passiert nichts. `startArray(true)` beginnt in jedem Zustand von vorn |
| `void pauseArray()` | Hält an, Winkel des aktuellen Punkts wird weiter geregelt, Fortschritt bleibt |
| `void resumeArray()` | Setzt am aktuellen Punkt fort (Pattern startet für diesen Punkt neu) |
| `void updateArray()` | **Jeden Zyklus aufrufen.** Fährt und schaltet weiter |
| `ArrayState giveArrayState()` | Aktueller Zustand |
| `uint8_t giveArrayIndex()` | Aktueller Zielpunkt (0-basiert) |
| `uint8_t giveArrayCount()` | Anzahl Punkte |

### Zustände

```
            startArray()                 letzter Punkt erreicht
 Idle ─────────────────────► Running ───────────────────────────► Finished
  ▲                           │   ▲                                 │
  │ resetArray()              │   │ resumeArray()                   │ hält Position
  │ (aus jedem Zustand)       ▼   │                                 │ und Winkel
  └───────────────────────── Paused                    startArray(true) → Running
```

| Zustand | `updateArray()` macht |
|---|---|
| `ArrayIdle` | Nichts, keine Motorbefehle. Der Roboter gehört wieder deiner übrigen Logik |
| `ArrayRunning` | Fährt den aktuellen Punkt an, schaltet weiter |
| `ArrayPaused` | Steht, regelt den Winkel des aktuellen Punkts |
| `ArrayFinished` | Hält Position und Winkel des letzten Punkts |

### Wann wird weitergeschaltet?

**Haltepunkt** (`PassCm = 0`, sowie immer der letzte Punkt): Weiter, wenn **alle drei** Bedingungen erfüllt sind:

1. Zielkreis erreicht (`giveMoveArrived()`),
2. Heading innerhalb `Andrehwinkel ± ARRAY_HEADING_TOL_DEG` (5°; bei 0 wird der Winkel nicht geprüft),
3. `HoldMs` ist abgelaufen.

**Durchfahrpunkt** (`PassCm > 0`): Weiter, sobald der Abstand zum Punkt ≤ `PassCm` ist. Der Roboter hält nicht an, Winkel und Wartezeit zählen nicht. Das Tempo läuft stetig weiter.

Bei jedem Weiterschalten startet das Speed-Pattern für den neuen Punkt von vorn, auch bei gleichem Ziel.

Nach dem letzten Punkt: `ArrayFinished`, bei `loop = true` stattdessen weiter bei Punkt 0.

### Änderungen und zyklische Aufrufe

- `setPointArray()` mit **gleichem Inhalt** und gleichem `loop` ändert nichts.
- Bei **geändertem Inhalt** beginnt die Liste bei Punkt 0. Ein laufender oder pausierter Zustand bleibt erhalten, ein beendeter oder ruhender Zustand wird `ArrayIdle` (dann `startArray()`).
- `addPointToArray()` zählt **nicht** als Änderung, die laufende Fahrt wird nicht unterbrochen.
- Eine **beendete** Liste startet durch zyklisches `startArray()` **nicht** neu, dafür `startArray(true)` oder `resetArray()` + neue Liste.

---

## 7. Pfadplanung (`moveToPointAvoid`)

Fährt zu einem Ziel und weicht dabei Ball und Partner aus. Der A*-Pathfinder ([`lib/Pathfinder`](../Pathfinder/Pathfinder.md)) plant auf einem Raster von 90 × 120 Zellen (eine Zelle ≈ 2 cm), die Gewichte stehen in `params/params_pathmap.h` (255 = gesperrt).

### Funktionen

```cpp
void setPathObstacles(int16_t ballX, int16_t ballY, int16_t robotX, int16_t robotY);
void moveToPointAvoid(uint16_t X, uint16_t Y,
                      uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed,
                      int16_t Andrehwinkel);
void resetPathPlan();
bool     givePathFound();      // A* hat einen Weg gefunden
uint8_t  givePathWaypoints();  // Anzahl Wegpunkte der Route
uint32_t givePathTimeUs();     // Rechenzeit der letzten Planung [µs]
```

| Funktion | Zweck |
|---|---|
| `setPathObstacles(...)` | Hindernisse in cm setzen, **−1 = kein Hindernis**. Ball = Ball, Roboter = Partner (oder Gegner). Zyklisch aufrufbar, vor `moveToPointAvoid()` |
| `moveToPointAvoid(...)` | Parameter wie bei `moveToPoint`. **Jeden Zyklus aufrufen** |
| `resetPathPlan()` | Verwirft Planung, Punkt-Array und Fahrprofil. **Beim Wechsel in einen anderen Modus aufrufen** |

### Ablauf

1. **A\*** plant von der aktuellen Position zum Ziel, mit Ball und Partner als Hindernis.
2. Der Zellpfad wird **vereinfacht**: von jedem Punkt so weit voraus, wie eine freie Gerade möglich ist (höchstens `PATH_LOS_MAX_CELLS` Zellen). Die Gerade läuft nie durch teurere Zellen als der A\*-Weg.
3. Die Wegpunkte werden zu **ArrayPoints** und vom [Punkt-Array](#6-punkt-array-routen) gefahren. Der letzte Punkt ist das exakte Ziel, nicht die Zellmitte.
4. Das **Speed-Pattern gilt für die ganze Strecke:** Zwischenpunkte fahren konstant mit dem Tempo, das das Pattern an dieser Stelle der Strecke hat, und werden ohne Halt durchfahren (`PATH_PASS_CM`). Erst das letzte Teilstück fährt das Pattern selbst, mit Zielkreis und Winkel.

### Wann wird neu geplant?

| Auslöser | Verhalten |
|---|---|
| Neues Ziel, Pattern, MaxSpeed oder MinSpeed | Sofort, das Pattern beginnt neu |
| Hindernis hat sich um mehr als `PATH_OBST_MOVE_CM` (8 cm) bewegt oder ist neu / weg | Sofort |
| Zyklisch alle `PATH_REPLAN_MS` (300 ms), solange der Roboter unterwegs ist | Das Pattern läuft dabei durch und startet nicht neu |
| Am Ziel angekommen | Keine Neuplanung |
| Planung gescheitert | Nächster Versuch erst nach `PATH_REPLAN_FAIL_MS` (1000 ms) |

`Andrehwinkel` darf sich ändern, ohne dass neu geplant wird.

### Wenn kein Weg gefunden wird

Der Reihe nach wird versucht:

1. mit Ball und Partner als Hindernis,
2. ohne Ball (das Ziel liegt oft im Ballbereich),
3. ohne Hindernisse (der Start liegt im Sperrbereich).

Scheitert auch das, gilt `PATH_FALLBACK_DIRECT`: `1` = geradeaus per `moveToPoint()` fahren, `0` = stehen bleiben und den Winkel halten. `givePathFound()` ist dann `false`.

### Einbinden ins Projekt

Nichts zu tun: `lib/Pathfinder/` gehört zum Projekt, `systemlib::initialize()` ruft `AStar.begin()` auf. Raster,
Kosten und Gewichtskarten stehen in `lib/include/params/` (`params_path.h`, `params_pathmap.h`). Ohne `-O2` wird die
Suche deutlich langsamer, die Flags in `platformio.ini` also beibehalten.

### Rechenzeit

Gemessen auf dem Teensy (912 MHz, `-O2`), pro `findPath`-Aufruf:

| Anteil der Aufrufe | Dauer |
|---|---|
| 50 % | ≤ 1 ms |
| 90 % | ≤ 6 ms |
| 99 % | ≤ 18 ms |
| Maximum | 35 ms |

Bei 600 MHz mit etwa dem 1,5-Fachen rechnen. Scheitert der erste Versuch, kann es bis zu dreimal so lange dauern. Die Positionsfusion rechnet mit `micros()` und begrenzt `dt` auf 50 ms, ein Stillstand durch die Planung bleibt also ohne Folgen. Bei Bedarf `PATH_REPLAN_MS` erhöhen.

### Hinweise

- **Solange `moveToPointAvoid()` benutzt wird, gehört ihm das Punkt-Array.** Nicht gleichzeitig eigene Routen setzen.
- Die Ballposition in cm wird erst verfügbar, wenn aus der IR-Messung eine Position berechnet wird. Bis dahin `−1` übergeben.

---

## 8. Parameter

Alle Werte stehen in `lib/include/params/` (`params_motion.h`, `params_path.h`), Übersicht in
[include.md](../include/include.md). Die Zahlen nur dort ändern, diese Tabelle nennt den Stand beim Schreiben.

| Name | Default | Bedeutung |
|---|---|---|
| `SPattern_Linear` … `SPattern_Pulse` | 0 … 18 | Pattern-Nummern (`enum SPattern`), siehe Abschnitt 5 |
| `MOVE_ARRIVE_CM` | 4.0 | Zielkreis: darin Speed 0 |
| `MOVE_ARRIVE_HYST_CM` | 3.0 | Hysterese, ab der nachgefahren wird |
| `MOVE_MAX_ACCEL_PCT_S` | 10000 | Max. Speed-Zunahme in %/s, 0 = aus |
| `MOVE_RESTART_GAP_MS` | 200 | Aufrufpause, ab der das Pattern neu startet |
| `MOVE_TARGET_TOL_CM` | 3.0 | Zieländerung bis zu diesem Wert startet das Pattern nicht neu |
| `MOVE_WALL_MARGIN_CM` / `MOVE_WALL_SPEED_PCT` | 20.0 / 5 | `moveDir()`: näher als das an der Wand wird der Speed begrenzt |
| `ARRAY_MAX_POINTS` | 16 | Max. Punkte in der Liste (ca. 14 Byte je Punkt) |
| `ARRAY_HEADING_TOL_DEG` | 5.0 | Winkeltoleranz zum Weiterschalten, 0 = nicht prüfen |
| `PATH_REPLAN_MS` | 300 | Neuplanung solange unterwegs |
| `PATH_REPLAN_FAIL_MS` | 1000 | Neuplanung nach gescheiterter Planung |
| `PATH_OBST_MOVE_CM` | 8.0 | Hindernisbewegung, die sofort neu plant |
| `PATH_PASS_CM` | 12 | Durchfahrradius der Zwischenpunkte |
| `PATH_LOS_MAX_CELLS` | 80 | Vorausschau beim Vereinfachen (Zellen) |
| `PATH_FALLBACK_DIRECT` | `true` | Ohne Weg: `true` = geradeaus, `false` = stehen |

Raster, Schrittkosten und Heap-Größe des A* (`PATH_GRID_*`, `PATH_COST_*`, `PATH_HEAP_CAPACITY`): [Pathfinder.md](../Pathfinder/Pathfinder.md).

---

## 9. Beispiele

### Ein Ziel anfahren

```cpp
// jeden Zyklus
Robot.moveToPoint(90, 150, SPattern_Linear, 80, 25, 0);
if (Robot.giveMoveArrived()) {
    // Ziel erreicht
}
```

### Feste Route (Ecken abfahren)

```cpp
ArrayPoint route[] = {
    // X    Y   Winkel  Pattern             Max Min  Wartezeit
    {  60,  60,   0,   SPattern_Linear,     80, 10,  2000 },
    {  60, 180,   0,   SPattern_Trapezoid,  80, 10,     0 },
    { 120, 180,   0,   SPattern_LateBrake,  80, 10,   100 },
    { 120,  60,   0,   SPattern_QuadDecel,  80, 10,   100 },
};

// jeden Zyklus (alle drei Aufrufe sind zyklisch ungefährlich)
Robot.setPointArray(route);        // Anzahl wird automatisch bestimmt
Robot.startArray();
Robot.updateArray();
```

Dauerschleife: `Robot.setPointArray(route, true);`

### Pausieren und fortsetzen

```cpp
if (Robot.giveStart() == false) Robot.pauseArray();
else                            Robot.resumeArray();
Robot.updateArray();
```

### Route zur Laufzeit aufbauen

```cpp
Robot.resetArray();
Robot.addPointToArray(60, 80, 0, SPattern_Linear, 80, 25);
Robot.addPointToArray(120, 160, 90, SPattern_Trapezoid, 70, 25, 500);
Robot.startArray();
// jeden Zyklus:
Robot.updateArray();
```

### Ausweichen um Partner und Ball

```cpp
// jeden Zyklus
Robot.setPathObstacles(ballX, ballY, partnerX, partnerY);   // -1, wenn unbekannt
Robot.moveToPointAvoid(120, 180, SPattern_Trapezoid, 80, 25, 0);
```

### Moduswechsel

```cpp
// beim Wechsel z. B. von "Ausweichen" zu "Route fahren"
Robot.resetPathPlan();
Robot.setPointArray(route);
Robot.startArray();
```

### Was setzt was zurück?

| Funktion | Fahrprofil | Punkt-Array | Pfadplanung |
|---|---|---|---|
| `resetMoveProfile()` | ✓ | | |
| `resetArray()` | ✓ | ✓ | |
| `resetPathPlan()` | ✓ (inkl. Speed 0) | ✓ | ✓ |

---

## 10. Fehlersuche

| Symptom | Ursache und Abhilfe |
|---|---|
| Roboter bewegt sich nicht, dreht sich nur | Position ist −1 (Radar verloren): `Position.X`/`Y` prüfen. Oder `MinSpeed = 0` bei Pattern, die bei Min starten |
| Roboter bleibt vor dem Ziel stehen | Zielkreis erreicht (`MOVE_ARRIVE_CM`). Ist das zu früh, den Wert verkleinern |
| Roboter schwingt um das Ziel | `MOVE_ARRIVE_CM` zu klein oder Position zu unruhig: Innovation (Kanal 192/193) prüfen |
| Pattern startet ständig neu | Ziel oder Parameter ändern sich jeden Zyklus. Dann `MOVE_TARGET_TOL_CM` erhöhen oder die Werte konstant halten |
| Array schaltet nicht weiter | Winkel liegt außerhalb `ARRAY_HEADING_TOL_DEG` (Heading-Regler läuft? `updateHeadingController()`), oder Zielkreis nicht erreicht (Hindernis, Wand). Zum Test Toleranz auf 0 setzen |
| Array läuft nach dem Ende nicht neu | Gewollt. `startArray(true)` oder `loop = true` |
| Array fährt gar nicht | Zustand ist `ArrayIdle`: `startArray()` fehlt. `updateArray()` nicht vergessen |
| `moveToPointAvoid` fährt immer geradeaus | `givePathFound()` ist `false` (kein Weg, Fallback `PATH_FALLBACK_DIRECT` aktiv) oder die Position ist unbekannt |
| Ruckeln alle 300 ms | Neuplanung dauert zu lange: `givePathTimeUs()` prüfen, `PATH_REPLAN_MS` erhöhen, Ball/Partner-Position glätten |
| Anlauf zu träge / zu ruppig | `MOVE_MAX_ACCEL_PCT_S` anpassen (0 = keine Begrenzung) |

---

## 11. Stand und Grenzen

- Alle Funktionen wurden in einer Simulation mit dem echten Code getestet (Pattern, Zielkreis, Neustart, Array-Zustände, Pfadplanung mit dem echten Pathfinder). **Auf dem Roboter sind sie noch nicht erprobt.**
- Zwischen den Punkten einer Route hält der Roboter an (Haltepunkte). Durchfahren ohne Halt gibt es nur bei Wegpunkten der Pfadplanung (`PassCm > 0`).
- Es gibt keinen Timeout pro Punkt: Steckt der Roboter fest, schaltet die Liste nicht von selbst weiter.
- Die Pfadplanung kennt nur Ball und einen weiteren Roboter. Ein zweiter Gegner wird nicht umfahren.
- Eine Zelle des Pathfinder-Rasters ist ≈ 2 cm, die Abstände zu Hindernissen kommen aus der Gewichtskarte in `params/params_pathmap.h`.
- Die Genauigkeit der Fahrt hängt direkt von der Positionsbestimmung ab. Kalibrierung: siehe *Kalibrier-Checkliste Positionsbestimmung*.
