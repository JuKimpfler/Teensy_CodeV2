# ESP32-S3-Firmware (Sensor-Koprozessor)

Läuft auf dem Seeed XIAO ESP32S3 der Platine V2.0 und liest die langsamen, blockierenden Sensoren, damit der Teensy im
Regelzyklus nicht auf den Bus warten muss. Konzept und Zeitbudget: [Plan im Dokument](../docs/Plan_Navigation-Lib.md),
[Laufzeit_Optimierung.md](../docs/Laufzeit_Optimierung.md). Protokoll: [Link](../lib/Link/Link.md), Teensy-Seite: [SensorLink](../lib/SensorLink/SensorLink.md).
Inbetriebnahme und Messungen: [Inbetriebnahme_ESP.md](../docs/Inbetriebnahme_ESP.md).

## Was der ESP liest: das Debug-Flag

`lib/include/params/params_sources.h` bestimmt, welche Geräte am ESP laufen. Der Wert muss im Teensy-`platformio.ini` und hier gleich sein
(build_flag, die Prüfsumme im HELLO meldet einen Unterschied):

| Flag | BNO055 | Radar (4 × XM125) | IR-Ring |
|---|---|---|---|
| `-DSENSOR_SOURCE=0` (Standard) | Teensy | Teensy | Teensy |
| `-DSENSOR_SOURCE=1` | **ESP** | **ESP** | **ESP** |
| einzeln `-DHEADING_SOURCE=` / `-DRADAR_SOURCE=` / `-DIR_SOURCE=` | 0 oder 1 | 0 oder 1 | 0 oder 1 |

Ist kein Gerät auf ESP gestellt, fasst die Firmware den I²C-Bus **nicht** an (der Teensy ist dann dort Master) und macht nur den Link, `SWITCH2`, Servo und Piezo.

## Was die Firmware sendet und tut

| Richtung | Inhalt | Takt |
|---|---|---|
| ESP → Teensy | Winkel + Drehrate des BNO055 (`LinkImu`) | `LINK_IMU_PERIOD_MS` (10 ms) |
| ESP → Teensy | Radar: vier Werte je Messrunde, Zeitstempel = Rundenmitte (`LinkRadar`) | ~50 ms |
| ESP → Teensy | IR-Ring: Ballwinkel und Distanz (`LinkIr`) | `LINK_IR_PERIOD_MS` (10 ms) |
| ESP → Teensy | `SWITCH2` (`LinkInput`) | bei Änderung, sonst 1 Hz |
| ESP → Teensy | Zustand: Schleifenzeit, I²C-Fehler, Heap (`LinkHealth`) | 5 Hz |
| Teensy → ESP | `LinkCmd`: BNO055 neu starten, Radar neu kalibrieren | selten |
| Teensy → ESP | `LinkAct`: Servo (500 … 2500 µs, 0 = aus), Piezo (Hz, ms) | nach Bedarf |

Der BNO055 hat Vorrang in der Schleife. Ein Radarstart (`radar.begin`) dauert Sekunden; der ESP sendet sein HELLO vorher, und der Teensy wartet höchstens
`LINK_BOOT_TIMEOUT_MS` (danach läuft er ohne ESP-Sensoren weiter und übernimmt sie, sobald der Link da ist).

```
pio run -d esp            # bauen
pio run -d esp -t upload  # flashen (USB-C am ESP, BOOT halten und RESET druecken, falls noetig)
pio device monitor -d esp # USB-Debug (Serial)
```

**Nicht auf dem ESP gebaut oder getestet.** Geprüft ist nur: Rechnertest der Protokolllogik (`test/native/run.sh`) und ein Syntaxlauf gegen ein grobes
Abbild des Arduino-ESP32-Cores in vier Flag-Kombinationen und für beide LEDC-APIs (2.x und 3.x), siehe `test/native/check_esp_syntax.sh`. Der echte Build,
`Serial1` auf D6/D7, I²C-Scan am ESP-Bus (BNO055 0x28) und die Servo-/Piezo-Ausgabe sind am Roboter zu prüfen.

**Wichtig:**
- Bleiben die Jumper `SDA_Jumper`/`SCL_Jumper` **geschlossen**, hängt der Teensy am selben Bus. Es darf nur einer Master sein: Quelle Teensy → Jumper zu, ESP
  passiv; Quelle ESP → Jumper **öffnen**.
- Beim Booten schreibt der ESP-ROM-Loader Text auf D6; der Link-Parser verwirft das.
- Kalibrierung des IR-Rings und BNO055-Kalibrier-Speicher laufen mit Quelle ESP noch nicht über den Link (`LinkCmdBnoCalibSave/Load` sind reserviert).
