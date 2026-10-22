# include – Zentrale Projektkonfiguration

Keine Bibliothek im eigentlichen Sinn, sondern die **einzige Stelle für Pins und
Parameter**. Bibliotheken binden nur `params.h` / `enum.h` ein und legen keine
eigenen Kopien an. Namensschema und Einheiten: [Coding_Rules.md](../../docs/Coding_Rules.md).

```
lib/include/
├── enum.h               Teensy-Pins, UART-Makros, Expander-Bits, I²C-Adressen
├── enum_esp.h           Pins des ESP32-S3 (XIAO-Namen D0 … D10)
├── params.h             Sammel-Header: bindet alle params_*.h ein
└── params/
    ├── params_system.h    Busse, Adressen, Debug-Modus, Linien-Schwellen, Kurs-Regler (HDG_)
    ├── params_radar.h     XM125-Radarsensoren (RADAR_)
    ├── params_field.h     Feld- und Torgeometrie (FIELD_)
    ├── params_position.h  Odometrie, Radar-Validierung, Fusion (ODO_, POS_, FUS_)
    ├── params_motion.h    Speed-Pattern, moveToPoint, Wandabstand, Punkt-Liste (MOVE_, ARRAY_)
    ├── params_path.h      Pfadplanung und A*-Raster (PATH_)
    ├── params_sources.h   DEBUG-FLAG: Sensoren am Teensy oder am ESP (SENSOR_SOURCE, HEADING_/RADAR_/IR_SOURCE)
    ├── params_link.h      Link Teensy ↔ ESP32-S3 (LINK_)
    └── params_pathmap.h   Gewichtskarten des A* (PATH_WEIGHTS_), nur von Pathfinder.cpp eingebunden
```

> **Änderungen hier wirken sich auf das ganze Projekt aus.** Was ein Mensch
> einstellt oder was die Verkabelung festlegt, steht hier. Register- und
> Protokollkonstanten eines Chips gehören dagegen in dessen Treiber.
> `lib/PDS/params_PDS.h` gehört zur PDS-Bibliothek (wird per `tools/sync_pds.py`
> kopiert) und bleibt deshalb dort.

---

## `enum.h` – Pins und Hardware-IDs (Platine V2.0)

Quelle ist der Schaltplan *2vs2-Lightweight-Main V2.0* (Blatt 1, 4, 5, 9). Eine Compile-Zeit-Prüfung
gegen die Schaltplanwerte steht in `test/Pinbelegung_Check.cpp` (bauen genügt).

**Serielle Schnittstellen**

| Makro | Port | Pins | Zweck |
|---|---|---|---|
| `USBC_UART` | `Serial` | USB | Konsole |
| `UART_DBG` | `Serial1` | 0 / 1 | PDS. Zusätzlich als **build_flag** `-DUART_DBG=Serial1`, weil `PDS.cpp` `enum.h` nicht sieht |
| `UART_CAM1` | `Serial2` | 7 / 8 | Kamera 1 |
| `UART_DIO` | `Serial3` | 14 / 15 | Schuss (`UART_DIO_TX/RX`) |
| `UART_ESP` | `Serial6` | 24 / 25 | Link zum ESP32-S3 (`SUBCPU_TX/RX`) |
| `UART_CAM2` | `Serial7` | 28 / 29 | Kamera 2 |

**Motoren** – Motor 1 = VL, Motor 2 = HL, Motor 3 = VR, Motor 4 = HR (Angabe des Teams):

| Motor | ID | PWM-Pin | Motor-ADC `_AP1` / `_AP2` | Expander (Richtung · Enable · Bremse) |
|---|---|---|---|---|
| `MotorVL` vorne links (Motor 1) | 1 | 3 | `IN0` / `IN1` | 0x22: P0 · P1 · P2 |
| `MotorHL` hinten links (Motor 2) | 2 | 4 | `IN2` / `IN3` | 0x22: P3 · P4 · P5 |
| `MotorVR` vorne rechts (Motor 3) | 0 | 5 | `IN4` / `IN5` | 0x20: P0 · P1 · P2 |
| `MotorHR` hinten rechts (Motor 4) | 3 | 6 | `IN6` / `IN7` | 0x20: P3 · P4 · P5 |
| `MotorDR` Dribbler | 4 | 2 | Pins 27 / 26 (analog) | Richtung: Pin 20; Enable · Bremse: 0x22 P6 · P7 |

> **ESCON-Belegung (Angabe des Teams):** `DO1` (→ ESCON DI2) = Richtung, `DIO3` = Enable, `DIO4` = Bremse. Die Pegel sind
> **Annahmen** (Richtungsbit 0 bei `Dir = true` wie bisher, Enable hoch = an, Bremse hoch = Bremse an) und am Roboter zu prüfen.
> Geändert wird nur in `enum.h` (`_BIT`-Konstanten) und in `systemlib::updateMotors()`. `_AP1` = Drehzahl, `_AP2` = Strom ist weiter
> eine Annahme (ESCON-Ausgänge `AO1`/`AO2`).

**Weitere Pins**

| Konstante | Pin | Bedeutung |
|---|---|---|
| `KICKER_PORT` | 9 | Schussmechanik |
| `SPI_CS_INTERCON` / `SPI_MOSI` / `SPI_MISO` / `SPI_SCK` | 10 / 11 / 12 / 13 | Bus `SPI` (nicht `SPI1`) |
| `I2C_SCL_3V3` / `I2C_SDA_3V3` | 16 / 17 | Teensy-Bus `Wire1` |
| `I2C_SDA_ESP` / `I2C_SCL_ESP` | 18 / 19 | `Wire`: ESP-Bus, nur mit Jumpern `SDA_Jumper`/`SCL_Jumper` |
| `BLDC_DO1` | 20 | Dribbler-Richtung (Ausgang, ESCON DI2) |
| `ROBO_ID` | 21 | Roboter-ID (Jumper JP4) |
| `RGB_DATA_3V3` | 22 | LED-Datenleitung |
| `LIGHT_BARRIER` | 23 | Lichtschranke (analog, `LDR_IN`) |
| `BLDC_AO2` / `BLDC_AO1` | 26 / 27 | Dribbler-Rückmeldung (analog, A12 / A13) |
| `ADC108S102_Motor_CS` | 30 | Chip-Select Motor-ADC |
| `LINE_DEC_A` / `_B` / `_C` | 31 / 32 / 33 | Dekoder der Linien-Platine, A = niedrigstwertiges Bit |

Damit sind alle 34 Pins belegt. `LED_LENGTH` = 19.

**I²C-Adressen und Expander** (Bit n = Pin Pn):

| Chip | Adresse | P0 · P1 · P2 · P3 · P4 · P5 · P6 · P7 |
|---|---|---|
| IC9 `Exp_MotorA` | 0x22 | M1 Richtung · M1 Enable · M1 Bremse · M2 Richtung · M2 Enable · M2 Bremse · Dribbler Enable · Dribbler Bremse |
| IC7 `Exp_MotorB` | 0x20 | M3 Richtung · M3 Enable · M3 Bremse · M4 Richtung · M4 Enable · M4 Bremse · SWITCH1 · BUTTON1 |
| IC8 `Exp_System` | 0x23 | VDRIB_EN · VMOT_EN · LIGHT_BARRIER_EN · LINE_EN_T · UART_DIO1 · I2C_INTERCON_3V3_EN · – · – |

INA219: 0x40 (Akku, 5 mΩ), 0x41 (Motorversorgung, 5 mΩ), 0x45 (Dribbler, 10 mΩ).

Pegel: `VMOT_EN` und `VDRIB_EN` hoch = Versorgung an; `LIGHT_BARRIER_EN` und `I2C_INTERCON_3V3_EN` niedrig = an (P-FET).
Alle vier sind ohne Treiber standardmäßig „an" (Pull-Up bzw. Pull-Down am Gate). `LINE_EN_T` ist undefiniert, siehe
`LINE_EN_LEVEL` in `params_system.h`.

---

## `params/` – Parameter

Alle Werte sind `static constexpr` (typisiert, prüfbar). Ausnahmen: die
`I2C_*`-Aliase und `DPDS_Mode`, weil sie Bezeichner bzw. Compile-Schalter sind.
Die Zahlen stehen **nur in den Headern** – diese Tabelle nennt die Gruppen:

| Datei | Präfix | Inhalt | Verwendet in |
|---|---|---|---|
| `params_system.h` | `I2C_`, `BNO_`, `LINE_`, `HDG_`, `MOTOR_EXP_` | Bus-Zuordnung und -Takte, Adresse, Linien-Schwellen, Gains des Kurs-Reglers (PID und ADRC), Auffrischung des Motor-Expanders | `systemlib` |
| `params_radar.h` | `RADAR_` | Adressen, Reichweite, Empfindlichkeit, Messrunden, Ausreißerfilter | `RadarArray` |
| `params_field.h` | `FIELD_` | Feldbreite/-länge, Tor (Breite, Tiefe, Kantenband) | Position, Fahrfunktionen, Pfad |
| `params_position.h` | `ODO_`, `POS_`, `FUS_` | Rad-Odometrie, Plausibilität und Signalstärke der Radarwerte, Kalman-Filter | `calculatePosition()` |
| `params_motion.h` | `SPattern_`, `MOVE_`, `ARRAY_` | Speed-Pattern (`enum SPattern`), Zielkreis, Anlaufbegrenzung, Wandabstand, Punktliste | [`MotionProfile`](../MotionProfile/MotionProfile.md) |
| `params_path.h` | `PATH_` | Neuplanung, Durchfahrradius, Fallback; Raster, Schrittkosten, Heap des A* | `moveToPointAvoid()`, [Pathfinder](../Pathfinder/Pathfinder.md) |
| `params_sources.h` | `SENSOR_SOURCE`, `HEADING_/RADAR_/IR_SOURCE` | **Debug-Flag:** BNO055, Radar, IR-Ring am Teensy (0) oder am ESP (1); Wert muss in beiden `platformio.ini` gleich sein | [SensorLink](../SensorLink/SensorLink.md) |
| `params_link.h` | `LINK_` | Baudrate, Zeitlimits, Pausen, Parameter-Prüfsumme | Link | [Link](../Link/Link.md), [SensorLink](../SensorLink/SensorLink.md), ESP-Firmware |
| `params_pathmap.h` | `PATH_WEIGHTS_` | Gewichtskarte des Feldes und Masken für Ball/Roboter (nicht in `params.h` enthalten) | [Pathfinder](../Pathfinder/Pathfinder.md) |

### Debug-Modus

`DPDS_Mode` ist `D_Mode_Test` (PDS + serielle Ausgaben) oder `D_Mode_Game`
(Build ohne Debug-Signale). Umschalten per `build_flags`:
`-DDPDS_Mode=D_Mode_Game`. Ohne Flag gilt `D_Mode_Test`.

### Radar abstimmen

Die Abstimmung (`RADAR_START_MM` … `RADAR_THRESHOLD_SENSITIVITY`) ist das
Ergebnis der Kalibrierung und die einzige Stelle zum Nachjustieren. Nachmessen:
[XM125_Calib_Sensitivity.cpp](../XM125/examples/XM125_Calib_Sensitivity.cpp),
[XM125_Calib_Crosstalk.cpp](../XM125/examples/XM125_Calib_Crosstalk.cpp), Übersicht in
[07-Kalibrierung.md](../XM125/doc/07-Kalibrierung.md). Alle Sensoren messen
gleichzeitig; `RadarArray::update()` wartet nie auf ein Ergebnis.

---

## `platformio.ini` – Build-Flags

| Flag | Bedeutung |
|---|---|
| `-DUSB_SERIAL` | USB als serielle Schnittstelle |
| `-O2` / `-Wall` | Optimierung / Warnungen |
| `-fno-math-errno` | `sqrtf()` als FPU-Befehl, siehe [Laufzeit_Optimierung.md](../../docs/Laufzeit_Optimierung.md) |
| `-DACTIVE_CHANNELS=200` | Anzahl PDS-Kanäle |
| `-DPDS_AUTO_CHANNEL_BASE=40` | Kanäle 0..37 gehören `systemlib::setupPDS()` |
| `-DPDS_MAX_UNITS=48` | Einheiten-Tabelle von PDS |
| `-DDPDS_Mode=…` | Debug-Modus, siehe oben |
| `-DSENSOR_SOURCE=0\|1` (und einzeln `HEADING_/RADAR_/IR_SOURCE`) | Sensoren am Teensy oder am ESP, **auch in `esp/platformio.ini`** |

`monitor_speed = 115200`, Board `teensy40`, **`board_build.f_cpu = 912000000L`** (Übertakten, Begründung und Test in [Laufzeit_Optimierung.md](../../docs/Laufzeit_Optimierung.md#3-cpu-takt)).
