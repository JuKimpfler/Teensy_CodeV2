# Laufzeit-Optimierung

Stand: Analyse und Umsetzung auf dem Rechner. **Auf dem Roboter nicht gemessen** – die Zeiten unten sind ein Modell
aus der Buszeit (100 kHz = 90 µs je Byte, Quelle: `lib/XM125/doc/10-Interna-und-Timing.md`). Vergleichen kannst du
sofort selbst: PDS-Kanal 9/10 (`Zykluszeit`, `Zykluszeit_us`) vor und nach dem Flashen.

> **Hinweis:** Diese Analyse beschreibt den Stand vor der Umstellung auf die Platine V2.0. Namen wie `SW_EXP`/`Motor_EXP`
> heißen dort jetzt `Exp_MotorB`/`Exp_MotorA`/`Exp_System`; die Maßnahmen (ein Lesevorgang je Expander, Schreiben nur bei Änderung)
> gelten unverändert. Auf V2.0 liest `readExpanders()` nur noch den Expander 0x20 (eine Transaktion je Zyklus).

## 1. Wo ging die Zeit hin?

Ein Zyklus (`systemlib::readSensors()`) wartet blockierend auf den I²C-Bus. Die Rechenzeit ist dagegen klein.

| Posten je Zyklus (Roboter fährt) | vorher | jetzt | Ursache |
|---|---|---|---|
| `readExpanders()`, 14 Einzel-Lesungen (12 an `Wire`, 2 an `Wire1`) | ≈ 5,3 ms | ≈ 0,5 ms | `TCA9534DW::readPin()` liest bei **jedem** Aufruf neu vom Chip. Jetzt: ein `update()` je Expander, Pins aus dem Zwischenspeicher |
| `updateMotors()`, bis zu 5 × `giveExpander(0,3)` | ≈ 1,9 ms | 0 | je Motor eine Lesung desselben Schalter-Pins. Jetzt: der Stand von `readExpanders()` |
| `updateMotors()`, `Motor_EXP.write()` | ≈ 0,3 ms | ≈ 0 | schrieb jeden Zyklus dasselbe Byte. Jetzt nur bei Änderung, spätestens nach `MOTOR_EXP_REFRESH_MS` |
| `RadarArray::update()`, Statusabfragen | ≈ 2,6 ms (3,7 je Zyklus) | ≈ 0,7 ms (1,0 je Zyklus) | 4 Sensoren wurden in jedem Zyklus der ~50-ms-Messung abgefragt. Jetzt erst ab 70 % der letzten Rundendauer |
| **Summe Bus** | **≈ 10 ms** | **≈ 1,2 ms** | |

Annahmen der Tabelle: Expander-Lesung = 2 Phasen × 2 Byte ≈ 0,38 ms bei 100 kHz; `Wire1` läuft jetzt mit 400 kHz.

Rechenzeit (µs-Bereich, mit Ausnahme des A*): `Utils::wrapAngle()` (18 Aufrufstellen) und `wrapPi()` im PID/ADRC
(3 × `sinf`/`cosf`/`atan2f` je Reglerlauf) sind jetzt im Normalfall ein Vergleich; die Radwinkel-Trigonometrie in
`setOmniWheelDir()` (4 × `sinf`, 4 × `cosf` je Aufruf) wird einmal berechnet; `sqrtf` ist ein FPU-Befehl
(`-fno-math-errno`); die Odometrie rechnet mit einer Konstante statt einer Double-Kette.

## 2. Was verändert das Verhalten?

| Änderung | Wirkung |
|---|---|
| Expander-Pins aus einer Lesung | Gleiche Werte, aber alle Pins stammen aus **einem** Zeitpunkt. Test: 100 000 Zyklen inkl. Lesefehler, 0 Abweichungen |
| `Motor_EXP` nur bei Änderung | Ein von außen zurückgesetzter Expander wird nach ≤ 50 ms wieder beschrieben |
| Radar-Abfrage ab 70 % | Gleiche Ergebnisse, gleiche Verspätung der Übernahme (Simulation: 3 / 5 / 10 ms Zyklus, jeweils identisch) |
| `wrapAngle()` | im Bereich (−180, 180] unverändert statt über `fmodf`: Abweichung ≤ 3,8·10⁻⁵ ° (BNO055 löst 0,0625 ° auf), außerhalb bitgleich |
| `wrapPi()` | im Bereich (−3,14; 3,14) unverändert: Abweichung ≤ 1,2·10⁻⁷ rad |
| `-fno-math-errno` | nur `errno` wird nicht mehr gesetzt, der Code liest es nicht |
| `Wire1` mit 400 kHz (`I2C_WIRE1_CLOCK_HZ`) | **Hardwareabhängig, ungetestet** (siehe 4.) |
| CPU 912 MHz | **Hardwareabhängig, ungetestet** (siehe 3.) |

## 3. CPU-Takt

`platformio.ini`: `board_build.f_cpu = 912000000L` (vorher Standard 600 MHz). Das ist **Übertakten**, 600 MHz ist die
Nennfrequenz des i.MX RT1062. Was der Teensy-Core (`clockspeed.c`) dabei tut: er hebt die Kernspannung an
(816 MHz: 1,425 V, 912 MHz: 1,525 V, 1008 MHz: 1,575 V; NXP empfiehlt höchstens 1,3 V), und bei 85 °C / 90 °C
löst der Temperaturalarm aus bzw. der Chip hält an.

- Die Chiptemperatur steht jetzt im PDS-Kanal 29 (`CPU_Temp`, 1 Hz).
- **912 MHz** ist der höchste Takt, für den es im Team einen Nachweis gibt (das Pathfinding-Projekt lief damit).
  Für diese Platine habe ich keinen Dauertest. „Maximal stabil" heißt hier: nach unten absichern.
- Test vor dem Wettkampf: 10 Minuten fahren/Motoren belasten, `CPU_Temp` beobachten. Dauerhaft über ~80 °C → `816000000L`
  eintragen oder Kühlkörper aufkleben. Ohne Kühlkörper nicht 960/1008 MHz.
- Der Takt beschleunigt nur die Rechenzeit (A*, Fusion, Filter), **nicht** die Buszeiten.

## 4. Was noch geht, aber nicht ohne Roboter

| Idee | Gewinn | Warum nicht umgesetzt |
|---|---|---|
| `Wire` auf 400 kHz (BNO055, IR, Schalter-Expander, Radar) | alle Buszeiten ÷ 4, ca. −0,9 ms | Das XM125 streckt den Takt; die Radar-Doku nennt 100 kHz als Grund (`11-Hardware-und-Integration.md`). Erst testen, ob die Radare bei 400 kHz fehlerfrei laufen |
| `Wire1` / Pull-Ups prüfen | – | 400 kHz verlangen ≤ 300 ns Anstiegszeit; die Pull-Up-Werte stehen nur in den Platinen-PDFs. Bei Lesefehlern in `Wire1` (Schalter springen, Akkuwerte 0) `I2C_WIRE1_CLOCK_HZ = 100000` |
| Schalter nicht jeden Zyklus lesen | −0,4 ms | Schalter ändern sich nie schneller als einige 10 ms; ein `% 5` im Zähler genügt. Ändert aber die Reaktionszeit der Tasten, das sollte jemand entscheiden |
| `BNO055` nur alle 10 ms lesen | −0,4 ms bei schnelleren Zyklen | Der Chip liefert in NDOF 100 Hz. Erst sinnvoll, wenn der Zyklus unter 10 ms liegt; kostet bis zu einen Zyklus Latenz im Winkel |
| `D_Mode_Game` im Wettkampf (`-DDPDS_Mode=D_Mode_Game`) | entfernt `PDS.update()` (USB-Serial) | schaltet die Telemetrie ab; die Zeit von `PDS.update()` ist nicht gemessen |
| `-O3`, `-ffast-math`, LTO | unklar | Ohne Messung am Roboter nicht belegbar. `-ffast-math` verändert Rundung und NaN-Verhalten der Filter, deshalb nicht gesetzt |
| `FASTRUN` für den A* | gering | Der gesamte Firmware-Code ist nur 63 kB; der Befehls-Cache hält die Schleifen. Die Daten liegen schon im DTCM (kein Wartezustand) |
| PWM-Berechnung in `updateMotors()` (Double-Multiplikation `× 2.55`) | wenige Takte | Mit Float würde der PWM-Wert bei manchen Geschwindigkeiten um 1 abweichen; Nutzen ist vernachlässigbar |

## 5. Messen
1. Vorher/nachher: `Zykluszeit_us` (PDS 10) bei stehendem und bei fahrendem Roboter.
2. `CPU_Temp` (PDS 29) 10 Minuten unter Last.
3. Radar: `Pos_certanty_x/y` (PDS 18/19) und die Aktualisierungsrate der Positionswerte sollen unverändert bleiben.
4. `Wire1`: Schalter und Akkuwerte (PDS 1–4) dürfen nicht springen.
