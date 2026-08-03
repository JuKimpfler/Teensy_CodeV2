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

### Schnittstellen

| Define | Wert | Bedeutung |
|---|---|---|
| `USBC_UART` | `Serial` | USB-Serial für Konsolenausgaben |
| `UART_DBG` | `Serial3` | Debug-UART für das [Power Debug System](../Debugger/PDS.md) |
| `LED_LENGTH` | `19` | Anzahl LEDs im RGB-Ring |

### Motor-IDs

| Define | Wert | Motor |
|---|---|---|
| `MotorVL_ID` | 0 | vorne links |
| `MotorVR_ID` | 1 | vorne rechts |
| `MotorHL_ID` | 2 | hinten links |
| `MotorHR_ID` | 3 | hinten rechts |
| `MotorDR_ID` | 4 | Dribbler |

### `enum PinOut` – Teensy-4.0-Pins

**PWM-Ausgänge (direkt am Teensy):**

| Konstante | Pin |
|---|---|
| `MotorVL_PWM` | 4 |
| `MotorVR_PWM` | 5 |
| `MotorHL_PWM` | 6 |
| `MotorHR_PWM` | 9 |
| `MotorDR_PWM` | 33 |

**Motor-Steuerbits am Portexpander** (Bitpositionen, **keine** Teensy-Pins –
siehe [TCA9534DW](../TCA9534DW/TCA9534DW.md)):

| Konstante | Bit | Bedeutung |
|---|---|---|
| `MotorVL_P1` / `_P2` | 0 / 1 | Richtung / Enable vorne links |
| `MotorVR_P1` / `_P2` | 2 / 3 | Richtung / Enable vorne rechts |
| `MotorHL_P1` / `_P2` | 4 / 5 | Richtung / Enable hinten links |
| `MotorHR_P1` / `_P2` | 6 / 7 | Richtung / Enable hinten rechts |

**Motor-ADC-Kanäle** (Kanalnummern am
[ADC108S102](../ADC108S102/ADC108S102.md), gleiche Zahlenwerte wie `_P1/_P2`):

| Konstante | Kanal | Bedeutung |
|---|---|---|
| `MotorVL_AP1` / `_AP2` | 0 / 1 | Drehzahl / Strom vorne links |
| `MotorVR_AP1` / `_AP2` | 2 / 3 | Drehzahl / Strom vorne rechts |
| `MotorHL_AP1` / `_AP2` | 4 / 5 | Drehzahl / Strom hinten links |
| `MotorHR_AP1` / `_AP2` | 6 / 7 | Drehzahl / Strom hinten rechts |

**Sonstige Pins:**

| Konstante | Pin | Bedeutung |
|---|---|---|
| `KICKER_PORT` | 18 | Schussmechanik |
| `START_SWITCH` | 10 | Startschalter |
| `LIGHT_BARRIER` | 23 | Lichtschranke (Ball im Dribbler), analog gelesen |
| `RGB_DATA_3V3` | 20 | LED-Datenleitung |
| `EMPTY_27` | 27 | frei |
| `ADC108S102_Motor_CS` | 32 | Chip-Select Motor-ADC |

**SPI:**

| Konstante | Pin |
|---|---|
| `SPI_MOSI` / `SPI_MISO` / `SPI_SCK` | 11 / 12 / 13 |
| `SPI_DIO0` … `SPI_DIO4` | 19, 31, 30, 21, 22 |

**I²C:** `I2C_SCL_3V3` = 16, `I2C_SDA_3V3` = 17 (das ist `Wire1` auf dem
Teensy 4.0).

**UARTs:**

| Konstante | Pin | Verwendung |
|---|---|---|
| `TNS_TX_PIXY_RX` / `TNS_RX_PIXY_TX` | 14 / 15 | Kamera ([Cam](../Cam/Cam.md)) |
| `UART1_TNS_TX` / `_RX` | 24 / 25 | Teensy↔Teensy (frei) |
| `UART2_TNS_TX` / `_RX` | 28 / 29 | Teensy↔Teensy (frei) |

---

## `params.h` – Busse, Adressen, Protokollparameter

### I²C-Zuordnung

```cpp
#define I2C_BNO   Wire
#define I2C_IR    Wire
#define I2C_SW    Wire
#define I2C_US    Wire
#define I2C_RADAR Wire
```

Alle Aliase zeigen aktuell auf `Wire`. Die INA219 und drei der vier
Portexpander werden in `systemlib.cpp` allerdings **direkt** auf `Wire1`
gelegt – die Aufteilung ist also nicht vollständig über diese Defines
gesteuert.

Ein Sensor lässt sich auf den anderen Bus umziehen, indem hier das Alias
geändert wird (z.B. `#define I2C_US Wire1`).

| Define | Wert | Bedeutung |
|---|---|---|
| `BNO_ADDRESS` | `0x28` | I²C-Adresse des [BNO055](../BNO055/BNO055.md) |

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
| `RADAR_MEASURE_TIMEOUT_MS` | `3000` | PROFILE5 + hohe Signalqualität braucht länger als der 1000-ms-Default |
| `RADAR_PARALLEL_GROUP` | `2` | Wie viele Sensoren gleichzeitig messen: `1` nacheinander, `2` = Vo+Hi dann Li+Re, `4` = alle. Muss `RADAR_NUM_SENSORS` teilen (`static_assert`) |

Verwendet in `systemlib::setupRadarSens()`. Hintergrund und Messverfahren:
[04-Konfiguration.md](../XM125/doc/04-Konfiguration.md#in-diesem-projekt-verwendete-werte),
Nachmessen mit [XM125_Calib_Sensitivity.cpp](../XM125/examples/XM125_Calib_Sensitivity.cpp)
(`RADAR_SIGNAL_QUALITY`, `RADAR_THRESHOLD_SENSITIVITY`) und
[XM125_Calib_Crosstalk.cpp](../XM125/examples/XM125_Calib_Crosstalk.cpp)
(`RADAR_PARALLEL_GROUP`); Übersicht aller Kalibrierskripte in
[07-Kalibrierung.md](../XM125/doc/07-Kalibrierung.md#ein-skript-je-kalibriermethode).

### Debug-UART

| Konstante | Wert | Bedeutung |
|---|---|---|
| `UART_DBG` | `Serial3` | wird gesetzt, falls nicht schon aus `enum.h` definiert |
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

Details zum Protokoll: [PDS.md](../Debugger/PDS.md)

---

## `platformio.ini` – Build-Flags

Ergänzend zu diesen Headern werden im Projekt-Root Build-Flags gesetzt:

| Flag | Wert | Bedeutung |
|---|---|---|
| `-DUSB_SERIAL` | – | USB als serielle Schnittstelle |
| `-O2` | – | Optimierungsstufe |
| `-DACTIVE_CHANNELS` | `200` | Anzahl aktiver Debug-Kanäle |

`monitor_speed = 115200`, `upload_protocol = teensy-cli`,
Board `teensy40`.
