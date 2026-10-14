# PDS – Kanalbelegung

Gegenstelle ist der [Power Debug Monitor](../lib/PDS/README.md) auf dem
RPi 5 / PC. Der Teensy schickt **200 Kanäle mit 100 Hz** raus und empfängt
**50 Floats + 50 Bools + 5 Fast-Floats** zurück.

Die Bibliothek liegt in [`lib/PDS/`](../lib/PDS/PDS.md) und ist eine Kopie aus
dem Repository `Robotic_PDS`. Aktualisiert wird sie mit:

```bash
python tools/sync_pds.py
```

---

## Wer vergibt welche Kanäle?

| Bereich | Wer | Wie |
|---|---|---|
| **0 – 30** | `systemlib::setupPDS()` | fest verdrahtet, siehe unten |
| 31 | frei | Reserve für weitere Systemwerte |
| **32 – 193** | der Sketch | automatisch über `PDS.plot()` / `PDS.track()` |
| 194 – 199 | `PDS.enableSelfDiagnostics()` | optional, belegt die letzten 6 Kanäle |

Dass der Sketch erst bei 32 anfängt, macht `-DPDS_AUTO_CHANNEL_BASE=32` in
[`platformio.ini`](../platformio.ini). **Ohne dieses Flag würde
`PDS.plot()` im Sketch bei Kanal 0 anfangen und die Systemwerte
überschreiben.**

Die festen Nummern 0–30 sind kein Selbstzweck: die Overlays in
[`lib/PDS/channel_config.h`](../lib/PDS/channel_config.h) (Feld-Draufsicht,
Akkuanzeige, Wertetabelle) zeigen auf Kanal**nummern**, nicht auf Namen. Wer
hier etwas verschiebt, muss `channel_config.h` nachziehen.

---

## Read – Systemkanäle 0 – 30

Gebunden in `systemlib::setupPDS()`
([systemlib.cpp](../lib/systemlib/systemlib.cpp)). `PDS.bind()` merkt sich nur
den Zeiger, die Werte holt die Bibliothek danach selbst 100×/s ab.

| Kanal | Name | Einheit | Quelle |
|---|---|---|---|
| `0` | Timer | s | `uptime_s` (Laufzeit seit Start) |
| `1` | Akku_V | V | `giveMainVoltage()`, 10 Hz nachgeführt |
| `2` | Akku_I | mA | `giveMainCurrent()`, 10 Hz nachgeführt |
| `3` | Motor_V | V | `giveMotorVoltage()`, 10 Hz nachgeführt |
| `4` | Motor_I | mA | `giveMotorCurrent()`, 10 Hz nachgeführt |
| `5` | Heading | ° | `BNO_eulerZ` |
| `6` | Orbit_Dir | ° | `BNO_OrbitDir` |
| `7` | BNO_Nullpunkt | ° | `BNO_calibration` |
| `8` | Ball_Winkel | ° | `IR_Angle` – **0, solange `readIR()` auskommentiert ist** |
| `9` | Zykluszeit | ms | `cycleTime` |
| `10` | Zykluszeit_us | us | `cycleTime_us` |
| `11` | Ball_Dist | – | `IR_Distance` – **s. o.** |
| `12` | Radar_Vo | Counts | `Radar[RadarVo].Strenght` (Signalstärke) |
| `13` | Radar_Li | Counts | `Radar[RadarLi].Strenght` |
| `14` | Radar_Hi | Counts | `Radar[RadarHi].Strenght` |
| `15` | Radar_Re | Counts | `Radar[RadarRe].Strenght` |
| `16` | Pos_X | cm | `Position.X` → Feld-Overlay |
| `17` | Pos_Y | cm | `Position.Y` → Feld-Overlay |
| `18` | Pos_certanty_x | – | `Position.XValid` |
| `19` | Pos_certanty_y | – | `Position.YValid` |
| `20` | Motor_VL | % | `MoDataVL.Speed_real` |
| `21` | Motor_VR | % | `MoDataVR.Speed_real` |
| `22` | Motor_HL | % | `MoDataHL.Speed_real` |
| `23` | Motor_HR | % | `MoDataHR.Speed_real` |
| `24` | Motor_VL (Dir) | % | `MoDataVL.Dir_real` |
| `25` | Motor_VR (Dir) | % | `MoDataVR.Dir_real` |
| `26` | Motor_HL (Dir) | % | `MoDataHL.Dir_real` |
| `27` | Motor_HR (Dir) | % | `MoDataHR.Dir_real` |
| `28` | Start | – | `startSwitch` |
| `29` | CPU_Temp | °C | `giveCpuTemp()`, 1 Hz nachgeführt. Dauerhaft über ~80 °C: Takt senken (`board_build.f_cpu` in `platformio.ini`); bei 90 °C hält der Chip an |
| `30` | Dribbler_I | mA | `giveDribblerCurrent()`, INA219 0x45, 10 Hz |
| `31` | *frei* | | Reserve für systemlib |

> **Radar:** `Dist` ist der Rohwert des XM125 in „Counts", **nicht**
> Millimeter – siehe [RadarArray.h](../lib/RadarArray/RadarArray.h).

### Eigene Werte dazunehmen

Im Sketch, ohne Kanalnummer und ohne Eintrag in einer Tabelle:

```cpp
PDS.track("Fahrt_Winkel", &angle, "°");   // einmal in setup(), Zeiger gebunden
PDS.plot("Ball_X", ballX);                // direkt im loop()
```

Die Kanalnummer vergibt PDS selbst (ab 32), der Name geht automatisch an die
GUI. Eine feste Nummer braucht nur, wer ein Overlay darauf zeigen lässt –
dann `PDS.bind("Name", &wert, 40, "cm")`.

---

## Write – Parameter von der GUI

Definiert in [`lib/PDS/channel_config.h`](../lib/PDS/channel_config.h); die
GUI baut ihren Parameter-Tab genau daraus auf.

### Fast (5 Floats, 100 Hz) – Joystick / PS4-Controller

| Index | Name | Bereich | Benutzt in |
|---|---|---|---|
| `0` | Fahrt_Y | −100 … 100 | `MotorControll_Test.cpp` → `PDS.fastParam(0)` |
| `1` | Fahrt_X | −100 … 100 | `MotorControll_Test.cpp` → `PDS.fastParam(1)` |
| `2`–`4` | *frei* | | |

Index 0 und 1 sind in `channel_config.h` zusätzlich als **ein** Joystick-Pad
(`PARAM_JOYSTICKS`) gebündelt.

> Fast-Parameter können in der GUI nur `slider` oder `number` sein. Die
> Widgets `toggle`/`button` schreiben dort immer auf einen **Slow-Bool**
> (siehe `qml/ParamsView.qml` im PDS-Repo) – ein Taster gehört deshalb in die
> Tabelle darunter.

### Slow-Bools (50, 2 Hz) – Schalter und Taster

| Index | Name | Art | Wirkung |
|---|---|---|---|
| `0` | Zielpunkt_anfahren | Taster | `sys.moveToPoint(50, 50, 50, 0)` |
| `1` | BNO_kalibrieren | Taster | `sys.calibBNO()` |
| `2`–`49` | *frei* | | |

### Slow-Floats (50, 2 Hz) – Tuning

Aktuell keiner belegt. Wer einen Reglerwert von der GUI aus drehen will,
trägt ihn in `PARAM_SLOW_FLOATS` ein und liest ihn mit
`PDS.param("Kp_Heading")`.

---

## Verkabelung

```
Teensy Pin 14 (TX3) ──→ RPi Zero Pin 10 (GPIO15, UART RX)
Teensy Pin 15 (RX3) ←── RPi Zero Pin  8 (GPIO14, UART TX)   ← PFLICHT
Teensy GND          ─── RPi Zero Pin  6 (GND)
```

Ohne die RX-Leitung gibt es keine Parameter und keinen Joystick. Die
UART-Instanz steht als `UART_DBG` in
[`lib/PDS/params_PDS.h`](../lib/PDS/params_PDS.h).
