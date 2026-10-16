# Link – Protokoll Teensy ↔ ESP32-S3

Reines C++ ohne Arduino-Abhängigkeit (`Link.h`, `Link.cpp`): dieselben Dateien laufen auf dem Teensy, dem ESP und im
Rechnertest ([`test/native`](../../test/native/link_test.cpp)). Die Teensy-Seite ist [SensorLink](../SensorLink/SensorLink.md),
die ESP-Seite `LinkEsp` (hier) in [`esp/src/main.cpp`](../../esp/src/main.cpp). Parameter: `params/params_link.h`.

## Rahmen

```
0xAA 0x55 | Typ | Seq | Länge | Nutzdaten (0 … 64) | CRC16 (little-endian)
```

CRC-16/CCITT-FALSE (Polynom 0x1021, Start 0xFFFF) über Typ bis Nutzdaten. Alle Zahlen little-endian, Byte für Byte
geschrieben. `Seq` zählt je Sender hoch (Überlauf bei 255 ist normal); Lücken zählt der Parser als `lostFrames()`.

## Nachrichten

| Typ | Richtung | Nutzdaten | Größe |
|---|---|---|---|
| `LinkImu` 0x01 | ESP → T | `TimeUs`, Winkel (1/16 °, im Uhrzeigersinn +), Drehrate Z (1/16 °/s, gegen den Uhrzeigersinn +), `Calib` | 9 |
| `LinkRadar` 0x02 | ESP → T | `TimeUs` (Mitte der Messrunde), `ConBits`, `PeakBits`, 4 × Distanz (gefiltert), 4 × Distanz roh, 4 × Signalstärke | 38 |
| `LinkIr` 0x03 | ESP → T | `TimeUs`, Ballwinkel (Rohwert des Rings, 1/180 °), Distanz | 8 |
| `LinkInput` 0x05 | ESP → T | Tastenbits (Bit 0 = `SWITCH2`) | 1 |
| `LinkHealth` 0x10 | ESP → T | Laufzeit, größte Schleifenzeit, I²C-/RX-Fehler, freier Heap, Flags | 15 |
| `LinkDebug` 0x11 | ESP → T | Kanal, Wert (für PDS) | 5 |
| `LinkHello` 0x80 | beide | Version, Rolle, Parameter-Prüfsumme, IMU-Rate | 8 |
| `LinkPing` 0x81 / `LinkPong` 0x82 | T → ESP / ESP → T | Zeitabgleich (`T0`; `T0`, `T1`, `T2`) | 4 / 12 |
| `LinkCmd` 0x83 | T → ESP | Befehl (BNO055 neu starten, …), Argument, Wert | 4 |
| `LinkAct` 0x84 | T → ESP | Servo oder Piezo | 5 |

Typ 0x04 (früher für einen Maus-Sensor vorgesehen) entfällt. `LinkCmdBnoCalibSave/Load` sind reserviert, die ESP-Firmware ignoriert sie bisher;
`LinkCmdRadarCalib` lässt den ESP die Radare neu kalibrieren, `LinkCmdBnoReset` startet den BNO055 neu.

## Klassen

| Klasse | Aufgabe |
|---|---|
| `LinkCodec` | `crc16()`, `encode()` |
| `LinkParser` | byteweiser Parser. Fehlerhafte Rahmen werden verworfen und gezählt, danach sucht er im bereits gelesenen Strom weiter (ein gültiger Rahmen, der in einem kaputten beginnt, geht nicht verloren) |
| `LinkPort` / `StreamLinkPort` | Anschluss an die UART (`Stream`), im Test ein Puffer |
| `LinkNode` | Sendezähler und Parser an einem Port; senden blockiert nie, passt der Rahmen nicht in den Sendepuffer, wird er verworfen und gezählt |
| `LinkEsp` | ESP-Seite: antwortet auf HELLO und PING, reicht `LinkCmd`/`LinkAct` weiter, sendet HELLO, bis der Teensy sich meldet |
| `LinkClock` | Zeitabgleich (NTP-Prinzip, vier Zeitpunkte, alles modulo 2³²); von den letzten 8 Messungen gilt die mit der kürzesten Laufzeit |
| `HeadingTracker` | rechnet den letzten Winkel mit der Drehrate auf den Abfragezeitpunkt weiter (höchstens `LINK_EXTRAP_MAX_US`) und lernt das Vorzeichen zwischen Drehrate und Winkel aus den Messungen |

## Ablauf

1. **Anlauf:** Beide Seiten senden HELLO (alle `LINK_HELLO_INTERVAL_MS`), bis die Gegenseite sich gemeldet hat. Version oder
   Parameter-Prüfsumme (`LINK_PARAMS_HASH`: Radar-Abstimmung, Baudrate, IMU-Takt und die **Sensorquellen** `HEADING_/RADAR_/IR_SOURCE`) stimmen nicht → `giveParamsMismatch()`.
2. **Zeitabgleich:** Der Teensy pingt zuerst 10 × alle 100 ms, danach 1 Hz. Jede Messung trägt die ESP-Zeit; der Teensy rechnet sie um.
3. **Ausfall:** Kein gültiger Rahmen für `LINK_ALIVE_MS` (50 ms) → `isAlive()`/`isUsable()` und `giveHeadingValid()` sind `false`; der Kursregler dreht dann nicht, die Radarwerte altern und die Fusion fällt auf die Odometrie zurück. Bei Parameter-Abweichung (`isUsable()` = `false`) werden die ESP-Daten gar nicht benutzt.
4. **ESP-Neustart:** Ein neues HELLO des ESP setzt Zeitabgleich und Winkel auf dem Teensy zurück.

## Geprüft (auf dem Rechner, mit ASan/UBSan)
- CRC gegen den bekannten Prüfwert (`"123456789"` = 0x29B1), Rundlauf aller Nachrichten.
- 3 Millionen Zufallsbytes: kein Rahmen angenommen. 20 000 Rahmen mit Bitfehlern (1 von 200 Bytes): 18 004 angekommen, **0 verfälscht**.
- Zeitabgleich über beliebige Versätze inkl. Zählerüberlauf: Fehler < 200 µs (die halbe Hin-/Rückweg-Asymmetrie).
- Simulation ESP ↔ Teensy über eine 2-Mbaud-Strecke (5 µs je Byte): Winkelfehler 0,02 … 0,08 ° RMS bei 120 °/s, ältestes Winkelalter 9,9 ms,
  Vorzeichen gelernt, ESP-Neustart und Linkausfall erkannt, 1 % Bitfehler und 0,5 % Verlust ohne falschen Winkel. Radar (99 Runden) und IR (499 Messungen)
  kommen mit dem richtigen Inhalt und einem Zeitfehler von 150 µs an. `RadarArray` mit simulierten Sensoren: Rundenzähler, Mitte der Runde, 1,0 statt 3,7 Statusabfragen je Zyklus.

**Nicht geprüft:** echte UART-Hardware, echte Teensy-/ESP-Zeitgenauigkeit, die Arduino-ESP32-Core-API (nur ein grobes Abbild, siehe `test/native/check_esp_syntax.sh`).
