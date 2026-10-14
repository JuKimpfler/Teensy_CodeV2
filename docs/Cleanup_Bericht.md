# Cleanup-Bericht

## Wie geprüft wurde
Alles mit `arm-none-eabi-g++` 13.2 gegen den Teensy-4-Core gebaut – **nicht** mit PlatformIO und **nicht** auf dem Roboter.

| Prüfung | Ergebnis |
|---|---|
| Alle 45 Quelldateien in `lib/`, `src/`, `test/` (inkl. Beispiele) übersetzen | 0 Fehler, 0 Warnungen (vorher: 1 Fehler, 5 Warnungen) |
| Echter Link-Test: Teensy-Core + alle Libs + `src/MotorControll_Test.cpp`, Archiv-Linking wie PlatformIO | Firmware linkt: 63 kB Code, 26 kB Daten, 195 kB BSS (davon `AStar` 132 kB) |
| `static_assert`-Wertetest: jeder verschobene/umbenannte Parameter hat den alten Wert (über 100 Konstanten) | bestanden |
| Formatieren ändert den erzeugten Code nicht (Objektvergleich vor/nach `clang-format`) | 22 von 22 identisch |
| Differenztest Pathfinder: Original-Lib gegen neue Klasse, 30 000 zufällige Suchen auf dem Rechner (Pfad, Karte inkl. Masken, Wegpunkte, `lineClear`), auch mit AddressSanitizer/UBSan | 0 Abweichungen |
| Heap der offenen Liste: höchster Füllstand 3043 von 6000; volle Liste bricht sauber ab (Test mit Kapazität 40) | bestanden |

**Nicht geprüft:** Verhalten auf dem Roboter (Fahren, Positionsgenauigkeit, Rechenzeit des A* auf dem Teensy), PlatformIO-Build.

## Neu: Pathfinder integriert
- `lib/Pathfinder/` (Klasse `Pathfinder`, `PathfinderTelemetry.h`, Doku, 2 Beispiele). Der A*-Algorithmus ist unverändert,
  der globale Zustand steckt jetzt in der Klasse. `lineClear` und `simplifyPath` wanderten aus `systemlib` in die Klasse (`giveWaypoints`).
- Gewichtskarten (`weights.cpp`) → `lib/include/params/params_pathmap.h` (`PATH_WEIGHTS_*`, nur von `Pathfinder.cpp` eingebunden).
  Raster, Schrittkosten, Heap: `params_path.h`.
- `systemlib` nutzt `AStar` fest; die stille Rückfallebene `PATH_AVAILABLE` (Pfadplanung ohne Fehlermeldung aus) gibt es nicht mehr.
- PC-Oberfläche: `tools/pathfinder_gui/`. Nicht übernommen: die Experimente `test/Optimized*`, `TestOnESP`, `weights-*Test.cpp`
  und die 57 Logdateien `Outputs/` des Pathfinding-Projekts.
- Neu gegenüber dem Original: Koordinaten werden geprüft (vorher Zugriff außerhalb des Arrays), eine volle offene Liste bricht die Suche ab (vorher Speicherüberschreiber).

**Zu prüfen:** `Gui_tester.cpp` im Pathfinding-Projekt enthält eigene, leicht abweichende Ball-/Roboter-Masken. Übernommen wurde `include/weights.cpp`, weil die Lib darauf verweist.

## Platine V2.0 (Phase 1 des Plans: Teensy-Seite)
Der Teensy-Code ist an den Schaltplan *2vs2-Lightweight-Main V2.0* angepasst (Pins, Busse, Expander, Motorzuordnung
VL = Motor 1, HL = Motor 2, VR = Motor 3, HR = Motor 4). Die Pinliste des Teams hatte drei Abweichungen vom Schaltplan
(SPI statt `SPI1`, Lichtschranke auf Pin 23, Pins 26/27 sind Dribbler-Rückmeldungen); maßgeblich ist der Schaltplan.
- `enum.h` neu: Pins, UART-Makros (`UART_DBG` = `Serial1`, `UART_CAM1/2`, `UART_DIO`, `UART_ESP`), Expander-Bits, I²C-Adressen.
  `UART_DBG` kommt zusätzlich als **build_flag** (`-DUART_DBG=Serial1`), weil `PDS.cpp` `enum.h` nicht sieht; ohne das Flag nähme `params_PDS.h` `Serial3`.
- `systemlib`: drei Expander (`Exp_MotorA` 0x22, `Exp_MotorB` 0x20, `Exp_System` 0x23) statt vier, Motor-ADC auf `SPI` (CS 30),
  Linien-ADCs `(&SPI, 31, 32, 33, n)`, drei INA219 mit Shunt (5 mΩ, 5 mΩ, 10 mΩ), Dribbler (PWM Pin 2, Enable Pin 20, Richtung Expander), `VMOT_EN`/`VDRIB_EN`/Lichtschranken-LED/`I2C_INTERCON_3V3_EN` werden gesetzt.
- Neu in der API: `setMotorPower()`/`giveMotorPower()` (schaltet `VMOT_EN`), `giveDribblerCurrent()`, PDS-Kanal 30 `Dribbler_I`.
- `TCA9534DW`: Ausgaberegister getrennt vom Eingangs-Zwischenspeicher, damit Schreiben die Schalterzustände nicht überschreibt.
- `test/Pinbelegung_Check.cpp`: Compile-Zeit-Prüfung aller Pins, ADC-Kanäle, Expander-Bits und Adressen gegen den Schaltplan (Gegenprobe mit absichtlich falschem Pin schlägt an).

**Annahmen, die du am Roboter bestätigen musst** (je eine Stelle in `enum.h`/`params_system.h`):
1. ESCON-Belegung je Motor: `DO1` (→ DI2) = Enable, `DIO3` = Richtung, `DIO4` bleibt Eingang. Falsch angenommen → Motoren laufen nicht oder drehen falsch herum.
2. `_AP1` = Drehzahl, `_AP2` = Strom (ESCON-Ausgänge `AO1`/`AO2`), wie im alten Code.
3. Startschalter = `SWITCH1` (Expander 0x20, P6, hoch = EIN). `BUTTON1` speist `BTN1`.
4. `LINE_EN_T` wird nicht getrieben (`LINE_EN_LEVEL = -1`); die Polarität steht nur auf dem Schaltplan der Linien-Platine.
5. Der BNO055, Radar und IR-Ring bleiben bis zum Link auf `Wire` (Pins 18/19). Das geht nur mit **geschlossenen Jumpern** `SDA_Jumper`/`SCL_Jumper` und einem ESP, der nicht als Master am Bus sendet.

**Verhalten gegenüber v3, das sich ändert:** Der Schalter, der die Motoren sperrte (`SWI[1]`), existiert auf V2.0 nicht;
dafür gibt es `setMotorPower()`. `SWI[1..4]` und die RCJ-Eingänge (`giveRCJ`, `giveRCJpresent`) sind immer `false`; `giveStart()` liefert `SWITCH1`.
Der Dribbler wird jetzt angesteuert (vorher schrieb `updateMotors()` keine Dribbler-PWM). `Cam::begin()` verlangt den Port.
`giveExpander(0/1/2, pin)` meint jetzt 0x20 / 0x22 / 0x23.

## Laufzeit und Takt (Details: `docs/Laufzeit_Optimierung.md`)
- **CPU-Takt 912 MHz** (`board_build.f_cpu`), vorher Standard 600 MHz. Übertakten, auf dieser Platine **nicht** dauergetestet. `CPU_Temp` steht in PDS-Kanal 29.
- **Bus-Zeit je Zyklus grob 10 ms → 1,2 ms (Modell, nicht gemessen):** Expander einmal statt 14-mal lesen, kein Schalter-Read je Motor, `Motor_EXP` nur bei Änderung schreiben,
  Radar-Statusabfragen erst in der Endphase der Messrunde, `Wire1` mit 400 kHz.
- Mathe: Schnellpfade in `wrapAngle()`/`wrapPi()`, Radwinkel-Trigonometrie einmal, Odometrie-Konstante, `-fno-math-errno`.
- Geprüft: Expander-Logik (100 000 Zyklen, 0 Abweichungen), Radar-Simulation (gleiche Ergebnisse, gleiche Verspätung, 3,67 → 1,01 Abfragen/Zyklus), Winkelfunktionen (≤ 3,8·10⁻⁵ ° bzw. 1,2·10⁻⁷ rad).
- `docs/PDS_Belegung.md` war gegenüber dem Code veraltet (Kanäle 12–28) und stimmt wieder.

## Parameter und Namen
- **Parameter an einem Ort:** `lib/include/params.h` bindet `lib/include/params/params_*.h` ein (system, radar, field, position, motion, path; Pathfinder-Karte separat). Alles `static constexpr`, `SPattern_*` ist `enum SPattern`.
- **Namensschema:** `PRÄFIX_NAME_EINHEIT` (`FIELD_X_CM`, `ODO_GEAR_RATIO`, `PATH_REPLAN_MS`) und die Klassen-/Methodenregeln in `docs/Coding_Rules.md`.
- **API der eigenen Libs vereinheitlicht** (alle Aufrufstellen angepasst, Compiler-geprüft):
  `Cam::init/Update/give_Angle/give_Angle_Cam/give_BlobH` → `begin/update/giveAngle/giveAngleCam/giveBlobH`, Klasse `CamC` → `Cam`;
  `BNO055/ADC108S102::init` → `begin`; `TCA9534DW::setup` → `begin`, `getPinState` → `givePinState`;
  `ADC108S102::getLastValue` → `giveLastValue`; `fancyBTN::getStatusCode` → `giveStatusCode`; `commlib::getOtherFlush` → `giveOtherFlush`;
  `Utilslib` → `Utils`, `Util.AMap` → `Util.wrapAngle`, `Util.Range` → `Util.inRange`.
- **Bewusst nicht umbenannt** (Tabelle in `Coding_Rules.md`): `XM125` und `SRF08` (`getXxx` liest vom Chip), `INA219`, `PMW3389`, die
  BNO055-Datenzugriffe, die `IR`-Lib, `PDS` (wird per `sync_pds.py` überschrieben), Klasse `systemlib`.
  **Alle Aufrufer außerhalb dieses Repos** (andere Sketches) müssen die neuen Namen übernehmen.

## Stil und Doku
- `.clang-format`, `.editorconfig`, `.gitattributes`; alles formatiert (4 Leerzeichen, Allman), LF, keine Tabs, `#pragma once` überall. Ausgenommen: `lib/PDS`, `lib/PMW3389`, `test/systemlib_gemini.cpp`.
- Doku neu bzw. korrigiert: `include.md` (war veraltet: Motor-IDs, PWM-Pins, Radar-Adressen), `Bewegungsfunktionen.md` (Parametertabelle hatte falsche Defaults, z. B. `MOVE_MAX_ACCEL_PCT_S` 600 statt 10000), `RadarArray.md` (Indizes Li/Re), `systemlib.md`, `Coding_Rules.md`, `lib/README`, Haupt-`README.md`, `Pathfinder.md`.
- XM125-Doku verwies noch auf die entfernte Lib `positioning` (alte API `Positioning::update`, `rawCountToCm`); jetzt auf `RadarArray`/`systemlib` umgeschrieben. Ausgabetexte der Kalibrier-Beispiele nennen `params_radar.h`.
- Toter Code entfernt: `systemlib::calcAxisPosition()`, Member `feld_x/feld_y`, nie benutzte Variable `line_depth`.

## Fehler, die dabei gefunden und behoben wurden – mit Wirkung
1. `resetHeadingController()` rief immer den ADRC-Regler auf; im Standardbetrieb (PID) ist der ein Nullzeiger → Absturz. Jetzt je nach Regler.
2. `setPointArray(route, count)` in der Pfadplanung war mehrdeutig (hätte `count` als `loop`-Flag deuten können). Der Code wurde vorher nie mitgebaut, weil der Pathfinder fehlte. Jetzt eindeutig.
3. `#if DPDS_Game`: Makro gab es nie. Jetzt `DPDS_Mode`; mit `-DDPDS_Mode=D_Mode_Game` wird PDS wirklich abgeschaltet. Standard unverändert.
4. `moveDir()`-Wandabstand rechnete mit Feldbreite 183 statt 182 cm → jetzt `FIELD_X_CM`.
5. `BNO055.h` band `../Wire/Wire.h` ein → `<Wire.h>`.
6. `drachen_test.cpp` baute nicht (`INA219::begin()` ohne Argument) → `begin(Wire1)`.
7. XM125-Kalibrier-Beispiele: 4. Sensor war auskommentiert, obwohl `RADAR_NUM_SENSORS` 4 ist, und die Namen passten nicht zu `RadarPos`. Jetzt `{Vo, Re, Hi, Li}` wie in `RadarArray.h`.

## Offen / bitte entscheiden
- **Radar-Werte:** Kommentare nannten Signalqualität 25.0 und Empfindlichkeit 0.75, im Code stehen 30000 und 0. Die Werte wurden übernommen. Stimmen sie?
- `POS_RADAR_OFFSET_CM`: Die XM125-Doku nannte früher 9,0 cm („Ringradius") und `0.902`-Skalierung, im Code stehen 2.0 cm und `DistRaw / 10`. Doku sagte „nicht ausgemessen (0.0)", im Code steht 2.0. Gemessen?
- **Sensorreihenfolge:** `RadarPos` im Code: Vo = 0, Re = 1, Hi = 2, Li = 3 (zu den Adressen `0x58, 0x56, 0x55, 0x52`). Stimmt das mit der Einbaulage?
- **Pathfinder auf dem Roboter:** `givePathFound()` und `givePathTimeUs()` ansehen und gegen die Messwerte in `Bewegungsfunktionen.md` (912 MHz) vergleichen; das Projekt läuft jetzt ebenfalls mit 912 MHz.
- **Doppelter `TrackingDifferentiator`** in `lib/Regler/PID` und `lib/Regler/ADRC` (identisch). Unter PlatformIO harmlos (Archiv-Linking), beim direkten Linken ein Fehler. Nicht angefasst, weil beide Ordner Kopien eigener Repos sind.
- **Löschkandidaten (nicht angefasst):** `test/systemlib_gemini.cpp` (veraltete Kopie, 1251 Z.), `test/old/`, `ZeilenMessen.py`, `lib/commlib/communication.md` (leer), `VS_Projekt.code-workspace`.
- Namen mit anderer Hand bzw. Treiberstil (siehe oben) bleiben, bis jemand am Roboter testen kann.
