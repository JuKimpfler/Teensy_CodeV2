# include – Zentrale Projektkonfiguration

Keine Bibliothek im eigentlichen Sinn, sondern die **zentrale
Konfigurationsstelle** des Projekts. Beide Header sind reine Definitionen und
werden von praktisch allen anderen Bibliotheken eingebunden.

Dateien: `enum.h` (Pinbelegung), `params.h` (Bus-/Protokollparameter)

> **Änderungen hier wirken sich auf das ganze Projekt aus.** Pins, Adressen
> und Paketgrößen sind an genau einer Stelle definiert – Bibliotheken sollen
> keine eigenen Kopien anlegen.

---

## `enum.h` – Pinbelegung und Hardware-IDs

Stand der Platinen in `pcbs/` (Main V2.0, Bottom, DasSwitchBoard V2.0). Die
Referenz dafür ist die Hardware-Karte in
[`src/CheckSystems.cpp`](../../src/CheckSystems.cpp).

### Schnittstellen

| Define | Wert | Bedeutung |
|---|---|---|
| `USBC_UART` | `Serial` | USB-Serial für Konsolenausgaben |
| `UART_DBG` | `Serial1` | DBG-INTERCON (Teensy 0/1) → Debugger-Board, [Power Debug System](../PDS/PDS.md). Steht zusätzlich in den `build_flags`, weil `PDS.cpp` diese Datei nicht einbindet |
| `UART_CAM1` / `UART_CAM2` | `Serial2` / `Serial7` | Kameras CAM1 (Teensy 7/8) / CAM2 (28/29) |
| `UART_DIO` | `Serial3` | UART/DIO-Stecker J6 (Teensy 14/15) |
| `UART_SUBCPU` | `Serial6` | ESP32-C3 SubCPU (Teensy 24/25) |
| `LED_LENGTH` | `19` | Anzahl LEDs am RGB-Ausgang |

### Motor-IDs

| Define | Wert | Motor |
|---|---|---|
| `MotorVR_ID` | 0 | vorne rechts |
| `MotorVL_ID` | 1 | vorne links |
| `MotorHL_ID` | 2 | hinten links |
| `MotorHR_ID` | 3 | hinten rechts |
| `MotorDR_ID` | 4 | Dribbler |

### Rad → ESCON-Modul

Die Main-Platine hat vier ESCON-Module MO1…MO4 (Blatt 6/7) und ein fünftes für
den Dribbler (Blatt 13). Welches Rad an welchem Modul hängt, steht in keinem
Plan, sondern ergibt sich aus der Verkabelung (bestätigt am 30.10.26):

| Rad | Modul | PWM (AI1+) | Freigabe / Richtung / Stopp | ADC Drehzahl / Strom |
|---|---|---|---|---|
| VR | MO1 | 3 | Bit 0 / 1 / 2 | IN1 / IN0 |
| HR | MO2 | 4 | Bit 3 / 4 / 5 | IN3 / IN2 |
| HL | MO3 | 5 | Bit 8 / 9 / 10 | IN5 / IN4 |
| VL | MO4 | 6 | Bit 11 / 12 / 13 | IN7 / IN6 |
| DR | Dribbler | 2 | Bit 6 / 7 / – | Teensy 27 / 26 (`MotorDR_AO1/AO2`) |

Stimmt die Zuordnung nicht, werden in `enum.h` alle Zeilen eines Rads
zusammen getauscht – PWM, Expanderbits und ADC-Kanäle gehören immer zu einem
Modul.

Konstanten je Rad: `MotorXX_PWM`, `MotorXX_EN`, `MotorXX_DIR`, `MotorXX_STOP`,
`MotorXX_RPM`, `MotorXX_CUR`.

**ESCON-Steuerleitungen** (Bitnummern 0…15 über zwei
[TCA9534DW](../TCA9534DW/TCA9534DW.md)): Bit 0…7 = IC9 (`0x22`) P0…P7,
Bit 8…15 = IC7 (`0x20`) P0…P7. Je Modul drei Leitungen am Stück:

| Offset | Leitung | ESCON-Eingang | Funktion |
|---|---|---|---|
| +0 | `MOx_DO1` | DI2 | Freigabe, HIGH = frei |
| +1 | `MOx_DIO3` | DI3 | Richtung |
| +2 | `MOx_DIO4` | DI4 | Stopp, HIGH = Stopp (angenommen) |

IC7 P6/P7 (`MAIN_SWITCH1` = 14, `MAIN_BUTTON1` = 15) sind Eingänge.

**Motor-ADC** IC10 ([ADC108S102](../ADC108S102/ADC108S102.md), CS 30):
IN0…IN7 = MO1_AO1, MO1_AO2, MO2_AO1, … MO4_AO2. Was AO1/AO2 ausgeben, legt
die ESCON-Konfiguration fest; angenommen ist wie auf der alten Platine
AO2 = Drehzahl, AO1 = Strom. Nachprüfen mit `CheckSystems`, Stufe 12 (`m`).

### Freigaben auf IC8 (`0x23`), Bitnummern

| Konstante | Bit | Bedeutung |
|---|---|---|
| `VDRIB_EN` | 0 | Dribblerspannung (zusätzlich Schalter SW19) |
| `VMOT_EN` | 1 | Motorspannung (zusätzlich Schalter SW3) |
| `LIGHT_BARRIER_EN` | 2 | LED der Lichtschranke |
| `LINE_EN` | 3 | Linien-LEDs der Bottom-Platine |
| `UART_DIO1` | 4 | DIO zum UART/DIO-Stecker |
| `I2C_INTERCON_3V3_EN` | 5 | 3V3 am I2C-INTERCON, **LOW = ein** |

`systemlib::initialize()` setzt Bit 0…3 und lässt 4/5 auf LOW.

### Sonstige Pins

| Konstante | Pin | Bedeutung |
|---|---|---|
| `KICKER_PORT` | 9 | Schuss, Pulsdauer hardwareseitig auf 150 ms begrenzt |
| `ROBO_ID` | 21 | Roboterkennung (Roboter 1 / 2) |
| `RGB_DATA_3V3` | 22 | WS2812B-Datenleitung |
| `LIGHT_BARRIER` | 23 | Lichtschranke (LDR), analog |
| `MotorDR_DO1` | 20 | Rückmeldung Dribbler-ESCON |

Der Startschalter ist kein Teensy-Pin mehr, sondern SwitchBoard IC1 P5
(`giveExpStartSwitch()`).

**SPI** (`SPI`): `SPI_MOSI` / `SPI_MISO` / `SPI_SCK` = 11 / 12 / 13,
`ADC108S102_Motor_CS` = 30, `SPI_TNS_CS` = 10 (CS des SPI-INTERCON, von der
Bottom-Platine nicht benutzt). Die sechs Linien-ADCs werden über den 74HC138
der Bottom-Platine gewählt: `SPI2_DEC_A` / `_B` / `_C` = 31 / 32 / 33,
Adresse 0…5 = Linien-ADC 1…6, 6 = Maussensor, 7 = keiner.

**I²C:** `I2C_TNS_SCL` / `_SDA` = 16 / 17 (`Wire1`), `I2C_ESP_SDA` / `_SCL` =
18 / 19 (`Wire`, über die Jumper JP1/JP2).

**UARTs:**

| Konstante | Pin | Verwendung |
|---|---|---|
| `DBG_RX` / `DBG_TX` | 0 / 1 | DBG-INTERCON (`UART_DBG`) |
| `UART1_TNS_RX` / `_TX` | 7 / 8 | CAM1 (`UART_CAM1`) |
| `UART_DIO_TX` / `_RX` | 14 / 15 | UART/DIO-Stecker (`UART_DIO`) |
| `SUBCPU_TX` / `_RX` | 24 / 25 | ESP32-C3 SubCPU (`UART_SUBCPU`) |
| `UART2_TNS_RX` / `_TX` | 28 / 29 | CAM2 (`UART_CAM2`) |

---

## `params.h` – Busse, Adressen, Protokollparameter

### I²C-Zuordnung

```cpp
#define I2C_BNO   Wire
#define I2C_IR    Wire
#define I2C_SW    Wire
#define I2C_US    Wire
#define I2C_RADAR Wire
#define I2C_MAIN  Wire1
```

`Wire` ist der „ESP“-Bus (Teensy 18/19): BNO055 und alles am I2C-INTERCON
(IR-Ring, SwitchBoard, Radar). `Wire1` ist der „TNS“-Bus (16/17) mit den
Portexpandern und INA219 der Main-Platine (`I2C_MAIN`).

Ein Gerät lässt sich auf den anderen Bus umziehen, indem hier das Alias
geändert wird (z.B. `#define I2C_SW Wire1`, wenn das SwitchBoard am
Intercon des TNS-Busses steckt).

| Define | Wert | Bedeutung |
|---|---|---|
| `BNO_ADDRESS` | `0x28` | [BNO055](../BNO055/BNO055.md) |
| `IR_ADDRESS` | `0x0A` | [IR-Ring](../IR/README.md) |
| `INA_MAIN_ADDRESS` / `INA_MOTOR_ADDRESS` / `INA_DRIB_ADDRESS` | `0x40` / `0x41` / `0x45` | [INA219](../INA219/INA219.md) Akku / VMOT / VDrib |
| `EXP_MO34_ADDRESS` | `0x20` | IC7: MO3, MO4, SWITCH1/BUTTON1 |
| `EXP_MO12_ADDRESS` | `0x22` | IC9: MO1, MO2, Dribbler |
| `EXP_POWER_ADDRESS` | `0x23` | IC8: Freigaben |
| `SW_SWITCH_ADDRESS` / `SW_BUTTON_ADDRESS` | `0x26` / `0x27` | SwitchBoard IC1 / IC2, per Jumper einstellbar |

### Radar (XM125)

| Konstante | Wert | Bedeutung |
|---|---|---|
| `RADAR_NUM_SENSORS` | `4` | Anzahl [XM125](../XM125/doc/README.md)-Module |
| `RADAR_SENSOR_ADDRS` | `0x51`…`0x54` | I²C-Adressen, hardwareseitig eingestellt |
| `RADAR_WAKE_PIN` / `RADAR_INT_PIN` | `-1` | `WAKE_UP`/`MCU_INT` bewusst unverkabelt |

**Abstimmung auf eine gerade Holzwand bis 2,5 m** – diese vier Werte sind das
Ergebnis der Kalibrierung und die einzige Stelle, an der nachjustiert wird:

| Konstante | Wert | Bedeutung |
|---|---|---|
| `RADAR_START_MM` | `200` | untere Bereichsgrenze; näher liegt der Roboter selbst |
| `RADAR_END_MM` | `2700` | obere Bereichsgrenze, 200 mm Reserve über dem Ziel von 2500 mm |
| `RADAR_SIGNAL_QUALITY` | `25000` | 25.0 statt Default 15.0 – mehr SNR für den schwachen Holz-Reflex |
| `RADAR_THRESHOLD_SENSITIVITY` | `750` | 0.75 statt Default 0.50 – CFAR lässt schwache Wand-Peaks durch |
| `RADAR_MEASURE_TIMEOUT_MS` | `3000` | PROFILE5 + hohe Signalqualität braucht länger als der 1000-ms-Default; geht per `setMeasureTimeout()` in die Library |
| `RADAR_PEAKS_TO_FETCH` | `1` | Wie viele Peaks je Messung wirklich aus dem Sensor geholt werden. Jeder Peak kostet zwei Registerlesezugriffe, ausgewertet wird nur Peak 0 |
| `RADAR_REINIT_INTERVAL_MS` | `2000` | Mindestabstand zwischen zwei Wiederanlaufversuchen für einen toten Sensor — die Neuinitialisierung blockiert bis ~2 s |

Alle Sensoren messen **gleichzeitig**; eine Gruppengröße gibt es nicht mehr.
`systemlib::readRadar()` stößt eine Runde bei allen Sensoren an und wartet nie
auf ein Ergebnis, siehe [02-Distance-Detector.md](../XM125/doc/02-Distance-Detector.md).

Verwendet in `systemlib::setupRadarSens()`. Hintergrund und Messverfahren:
[04-Konfiguration.md](../XM125/doc/04-Konfiguration.md#in-diesem-projekt-verwendete-werte),
Nachmessen mit [XM125_Calib_Sensitivity.cpp](../XM125/examples/XM125_Calib_Sensitivity.cpp)
(`RADAR_SIGNAL_QUALITY`, `RADAR_THRESHOLD_SENSITIVITY`) und
[XM125_Calib_Crosstalk.cpp](../XM125/examples/XM125_Calib_Crosstalk.cpp)
(stören sich die gleichzeitig messenden Sensoren?); Übersicht aller Kalibrierskripte in
[07-Kalibrierung.md](../XM125/doc/07-Kalibrierung.md#ein-skript-je-kalibriermethode).

### Debug-UART

| Konstante | Wert | Bedeutung |
|---|---|---|
| `UART_DBG` | `Serial3` | Default in `params_PDS.h`; das Projekt setzt per `-DUART_DBG=Serial1` den DBG-INTERCON der Main V2.0 |
| `UART_DBG_BAUD` | `1 000 000` | 1 Mbps |

### Parameter-Downlink (GUI → Teensy)

**Slow-Kanal**, ~2 Hz:

| Konstante | Wert |
|---|---|
| `PARAM_SLOW_MAGIC` | `0xCAFEFEED` |
| `PARAM_SLOW_FLOAT_COUNT` | `50` |
| `PARAM_SLOW_BOOL_COUNT` | `50` |
| `PARAM_SLOW_PACKET_BYTES` | `258` |
| `PARAM_SLOW_TIMEOUT_MS` | `1000` |

**Fast-Kanal**, ~100 Hz (z.B. Joystick):

| Konstante | Wert |
|---|---|
| `PARAM_FAST_MAGIC` | `0xFA57DA7A` |
| `PARAM_FAST_FLOAT_COUNT` | `5` |
| `PARAM_FAST_PACKET_BYTES` | `28` |
| `PARAM_FAST_TIMEOUT_MS` | `150` |

`PARAM_HEADER_BYTES` = `8` (Magic 4 Byte + Sequenznummer 4 Byte), für beide
Pakettypen gleich.

**Timeouts:** Slow entspricht 2 verpassten Zyklen. Fast ist bewusst
großzügiger als 2 Zyklen (150 ms statt 20 ms), weil bei 100 Hz über WLAN sonst
dauernd Fehlalarm ausgelöst würde – **am Feld ggf. nachjustieren**.

Details zum Protokoll: [PDS.md](../PDS/README.md)

---

## `platformio.ini` – Build-Flags

Ergänzend zu diesen Headern werden im Projekt-Root Build-Flags gesetzt:

| Flag | Wert | Bedeutung |
|---|---|---|
| `-DUSB_SERIAL` | – | USB als serielle Schnittstelle |
| `-O2` | – | Optimierungsstufe |
| `-DACTIVE_CHANNELS` | `200` | Anzahl aktiver Debug-Kanäle |
| `-DUART_DBG` | `Serial1` | Debug-UART für PDS (DBG-INTERCON, Teensy 0/1) |

`monitor_speed = 115200`, `upload_protocol = teensy-cli`,
Board `teensy40`.
