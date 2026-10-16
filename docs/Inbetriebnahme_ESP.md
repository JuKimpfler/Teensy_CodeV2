# Inbetriebnahme ESP32-S3 und Abnahme

Reihenfolge am Roboter. Jeder Schritt hat ein Kriterium; erst weiter, wenn es erfüllt ist. **Nichts davon ist bisher gelaufen** –
der Code ist auf dem Rechner getestet (`test/native/run.sh`), nicht auf der Hardware. Messwerte tragt ihr in die rechte Spalte ein.

Zum Mitlesen: PDS-Kanäle 31 – 37 (`docs/PDS_Belegung.md`), `Zykluszeit_us` (Kanal 10).

## 0. Vorher (nur Teensy, Quelle Teensy)

| # | Schritt | Kriterium | Messwert |
|---|---|---|---|
| 0.1 | Teensy mit `SENSOR_SOURCE=0` flashen, Jumper `SDA_Jumper`/`SCL_Jumper` zu | Winkel, Radar und IR wie bisher | |
| 0.2 | `Zykluszeit_us` (PDS 10) bei Stand und bei Fahrt aufschreiben | Referenz für Schritt 6 | Stand: ___ µs, Fahrt: ___ µs |
| 0.3 | Motoren: Richtung, Enable und Bremse prüfen (`DI2` = Richtung, `DIO3` = Enable, `DIO4` = Bremse) | jeder Motor dreht richtig herum, bremst nur im Stillstand | |
| 0.4 | `AO1`/`AO2` der ESCON: dreht sich ein Rad, steigt `_AP1` (Drehrate?) oder `_AP2` (Strom?) | Annahme `_AP1` = Drehzahl bestätigt oder in `enum.h` getauscht | |

## 1. ESP allein (Quelle Teensy, ESP passiv)

| # | Schritt | Kriterium | Messwert |
|---|---|---|---|
| 1.1 | ESP mit `pio run -d esp -t upload` flashen (`-DSENSOR_SOURCE=0` in beiden `platformio.ini`) | Build läuft durch, Flashen klappt | |
| 1.2 | Teensy neu starten, PDS ansehen | `Link_Status` (32) = 0, `Link_CRC` (35) und `Link_Luecken` (36) bleiben 0 | |
| 1.3 | 10 Minuten laufen lassen | CRC-Fehler 0 (oder sehr selten), keine Lücken | CRC: ___ Lücken: ___ |
| 1.4 | `Link_Laufzeit` (37) | unter 1000 µs | ___ µs |
| 1.5 | `SWITCH2` betätigen, `giveSwitch2()` ausgeben | Wert folgt dem Schalter | |
| 1.6 | `setServo(1500)`, `setServo(0)`, `playPiezo(2000, 200)` aus einem Sketch | Servo bewegt sich und wird entspannt, Piezo piept | |
| 1.7 | **Fehlerfall:** ESP abziehen | `Link_Status` = 1, Roboter läuft mit den Teensy-Sensoren unverändert weiter | |

## 2. Winkel über den Link (`-DHEADING_SOURCE=1`)

Vorher: Jumper **öffnen**, `-DHEADING_SOURCE=1` in beiden `platformio.ini`, `-DALLOW_MIXED_SENSOR_SOURCES` auf dem Teensy (Radar und IR sind
noch am Teensy: sie dürfen dann nicht am ESP-Bus stecken, sondern am Teensy-Stecker J13 mit `-DI2C_RADAR=Wire1 -DI2C_IR=Wire1` und `I2C_WIRE1_CLOCK_HZ` = 100000).

| # | Schritt | Kriterium | Messwert |
|---|---|---|---|
| 2.1 | Roboter von Hand drehen | `BNO_OrbitDir` folgt, `Heading_Alter` (31) unter 12 ms | max: ___ ms |
| 2.2 | Winkel mit dem Stand aus 0.1 vergleichen (gleiche Drehung) | Abweichung klein, kein Vorzeichenfehler (`giveGyroSign()` ±1 stabil) | |
| 2.3 | Kursregler: auf 90° drehen lassen | Überschwinger und Einschwingzeit wie vor dem Umbau | |
| 2.4 | **Fehlerfall:** ESP abziehen während der Roboter steht | `giveHeadingValid()` = `false`, Kursregler gibt 0 aus, Roboter dreht nicht | |
| 2.5 | ESP wieder stecken | Winkel kommt zurück, Nullpunkt unverändert | |

## 3. Radar und IR über den Link (`-DSENSOR_SOURCE=1`)

Jumper offen, `-DSENSOR_SOURCE=1` in beiden `platformio.ini`, Radar und IR am ESP-Stecker J14.

| # | Schritt | Kriterium | Messwert |
|---|---|---|---|
| 3.1 | `ESP_I2C_Fehler` (34) und `ESP_Loop_max` (33) beobachten | Fehler 0, Schleifenzeit unter 10 ms | max: ___ µs |
| 3.2 | **Buslast am ESP-Bus** (Oszilloskop oder Logikanalysator an SDA/SCL, Anteil Busy) | unter 70 % (Modell: ≈ 43 %); sonst IR-Ring auf `-DIR_SOURCE=0` mit J13 | ___ % |
| 3.3 | Radarwerte: `Radar[RadarVo].DistRaw` usw. mit Quelle Teensy vergleichen | gleiche Größenordnung, gleiche Aktualisierungsrate (~20 Hz) | |
| 3.4 | `Pos_certanty_x/y` (PDS 18/19) beim Fahren | wie vorher oder besser | |
| 3.5 | IR: `giveIRAngle()`/`giveIRDistance()` mit dem Ball | wie vorher | |
| 3.6 | **Fehlerfall:** ESP abziehen beim Fahren | Radar `isCon` = `false`, Position fällt auf Odometrie zurück (`Certainty` sinkt), Roboter bleibt steuerbar | |
| 3.7 | **Fehlerfall:** nur den Teensy neu starten | ESP bleibt an, Link baut sich nach dem HELLO wieder auf | |

## 4. Zeit (Abnahme)

| # | Schritt | Kriterium | Messwert |
|---|---|---|---|
| 4.1 | `Zykluszeit_us` (PDS 10) wie in 0.2 messen | deutlich kürzer als die Referenz; Modell: Busanteil ≈ 0,1 ms statt ≈ 1,2 ms | Stand: ___ µs, Fahrt: ___ µs |
| 4.2 | `CPU_Temp` (PDS 29) 10 Minuten unter Last | dauerhaft unter 80 °C bei 912 MHz, sonst `board_build.f_cpu = 816000000L` | max: ___ °C |
| 4.3 | `Link_Laufzeit` (37) und Zeitversatz | stabil | |

## Kehrtwendung

Läuft etwas nicht: beide `platformio.ini` auf `-DSENSOR_SOURCE=0` zurück (oder den Eintrag entfernen), Jumper schließen. Das ist der Stand ohne ESP-Sensoren.
