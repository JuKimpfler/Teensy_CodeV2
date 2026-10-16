# SensorLink – Teensy-Seite des Links zum ESP32-S3

Nimmt die Messungen des ESP entgegen, gleicht die Zeit ab und liefert den Winkel mit der Drehrate auf den Abfragezeitpunkt
weitergerechnet. Protokoll und Klassen: [Link](../Link/Link.md). Alle Funktionen kehren sofort zurück; die Zeit (`micros()`)
wird übergeben, das macht die Lib auf dem Rechner testbar.

```cpp
// in systemlib: UART_ESP.begin(LINK_BAUD); EspLink.begin(&EspLinkPort, micros());
EspLink.update(micros());                    // jeden Zyklus
if (EspLink.giveHeadingValid(micros()))
    float h = EspLink.giveHeading(micros()); // 0 … 360, im Uhrzeigersinn
```

| Funktion | Zweck |
|---|---|
| `begin(port, nowUs)` | Verbindungsaufbau (HELLO) |
| `update(nowUs)` | Rahmen lesen, HELLO/PING senden |
| `isAlive`, `isUsable`, `giveHandshakeDone`, `giveParamsMismatch`, `giveBootTimeout` | Zustand der Verbindung (`isUsable` = lebt, Handshake fertig, Parameter passen) |
| `giveHeadingValid`, `giveHeading`, `giveHeadingAgeUs`, `giveGyroSign`, `giveImuCalib` | Winkel |
| `takeRadar`, `takeIr` | liefern genau einmal je neuer Messung die Daten und die Messzeit in Teensy-Zeit |
| `giveInputBits`, `giveSwitch2`, `giveHealth` | Tasten (Bit 0 = `SWITCH2`) und ESP-Zustand (Schleifenzeit, I²C-Fehler, Heap) |
| `giveClockValid`, `giveClockOffsetUs`, `giveClockDelayUs`, `giveFramesOk`, `giveCrcErrors`, `giveLostFrames` | Zeitabgleich und Zähler |
| `sendCmd`, `sendServo`, `sendPiezo` | Befehle an den ESP |

## Einbindung in `systemlib` und das Debug-Flag für die Sensorquellen

`lib/include/params/params_sources.h` entscheidet, **wo** BNO055, Radar und IR-Ring laufen. Der Wert muss im Teensy- und im
ESP-`platformio.ini` gleich sein (build_flag; die Prüfsumme im HELLO meldet einen Unterschied):

| Flag | Wirkung |
|---|---|
| `-DSENSOR_SOURCE=0` (Standard) | alles am Teensy: der Teensy liest die Sensoren selbst am ESP-Bus (`Wire`, nur mit geschlossenen Jumpern); die ESP-Firmware fasst den Bus nicht an |
| `-DSENSOR_SOURCE=1` | alles am ESP: der Teensy fasst `Wire` nicht mehr an (Jumper öffnen!) und bekommt Winkel, Radar und IR über den Link |
| `-DHEADING_SOURCE=0\|1`, `-DRADAR_SOURCE=0\|1`, `-DIR_SOURCE=0\|1` | je Gerät einzeln (überschreibt `SENSOR_SOURCE`) |

Gemischte Quellen (z. B. Radar am Teensy, BNO055 am ESP) bedeuten zwei Busmaster am ESP-Bus. `systemlib.cpp` gibt dafür eine Compiler-Warnung aus; sie ist nur
gewollt, wenn die Teensy-Geräte am Teensy-Stecker J13 (`Wire1`, `-DI2C_RADAR=Wire1` o. Ä., `I2C_WIRE1_CLOCK_HZ` = 100000) stecken, dann `-DALLOW_MIXED_SENSOR_SOURCES` setzen.

Was `systemlib` je Quelle tut:

| Gerät | Quelle Teensy | Quelle ESP |
|---|---|---|
| Winkel | `BNO.eulHeading()` | `EspLink.giveHeading()`, mit der Drehrate auf jetzt weitergerechnet; ohne gültigen Winkel dreht der Kursregler nicht |
| Radar | `Radar.update()` je Zyklus | `readRadar()` übernimmt die Runde vom Link (`Radar.feedRemote()`), Zeit = Rundenmitte, `FUS_RADAR_LATENCY_LINK_MS` statt `FUS_RADAR_LATENCY_MS`; ohne Link werden die Sensoren als nicht verbunden markiert und die Werte altern |
| IR-Ring | `ir_ring.read_ball_*()` | `readIR()` übernimmt die letzten Werte vom Link und kostet keine Buszeit. Die Kalibrierfunktionen (`calibIR`, `writeIRCalibration`) tun dann nichts |

Weitere Punkte:
- Beim Start wartet der Teensy höchstens `LINK_BOOT_TIMEOUT_MS` auf den ESP (nur wenn mindestens ein Gerät am ESP hängt). Danach läuft er ohne dessen Sensoren weiter.
- `giveLinkState()`: `LinkStateOk`, `LinkStateNoEsp`, `LinkStateMismatch`, `LinkStateNotUsed`; PDS-Kanäle 32 … 37 zeigen Zustand, ESP-Schleifenzeit, I²C-Fehler, CRC-Fehler, Lücken und Laufzeit.
- `calibBNO()` setzt den Nullpunkt bei der nächsten gültigen Messung, falls jetzt keine vorliegt. PDS-Kanal 31 `Heading_Alter` zeigt das Alter der Winkelmessung.
- Aktoren am ESP: `setServo(us)` (0 = aus), `playPiezo(hz, ms)`, `giveSwitch2()`.
- Teensy-RX-Puffer 1024 Byte, TX 256 Byte (keine Flusssteuerung); `UART_ESP` = `Serial6`, Pins 24/25.
