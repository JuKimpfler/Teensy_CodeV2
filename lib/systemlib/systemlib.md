# systemlib – Hauptbibliothek (Hardware-Abstraktion)

`systemlib` ist die **einzige Bibliothek, die das Hauptprogramm einbinden
muss**. Sie legt alle Sensor- und Aktor-Instanzen an, initialisiert sie, liest
sie zyklisch aus und stellt die Werte über `give*()`/`set*()`-Funktionen
bereit. Die Taktik arbeitet damit ausschließlich gegen diese Schnittstelle,
nicht gegen die Treiber.

Dateien: `systemlib.h`, `systemlib.cpp`

---

## Aufbau

```
Hauptprogramm (src/)
        │
        ▼
   systemlib  ──────────────────────────────────────────────┐
        │                                                    │
   ┌────┴─────┬──────────┬──────────┬──────────┬─────────┐   │
   ▼          ▼          ▼          ▼          ▼         ▼   ▼
 BNO055     IR-Ring    SRF08     INA219   TCA9534DW  ADC108S102
                                                              │
                         PID / ADRC ◄── Heading   PDS ◄── Debug
                                        commlib ◄── Partnerroboter
```

Konventionen aus [`lib/README`](../README): `systemlib` ist die Hauptlib,
Parameter liegen in [`include/`](../include/include.md), Sublibs haben genau
zwei Dateien und `give`/`set`-Funktionen.

---

## Instanzen (global in `systemlib.cpp`)

| Instanz | Typ | Bus / Adresse |
|---|---|---|
| `BNO` | [BNO055](../BNO055/BNO055.md) | `I2C_BNO` (`Wire`), `0x28` |
| `ir_ring` | [IR_ring](../IR/README.md) | `I2C_IR` (`Wire`), `0x0A` |
| `INA219_Main` | [INA219](../INA219/INA219.md) | `Wire1`, `0x40` |
| `INA219_Motor` | INA219 | `Wire1`, `0x41` |
| `SRF08_vo/re/hi/li` | [SRF08](../SRF08/SRF08.md) | `I2C_US` (`Wire`), `0xE2/E4/E6/E8` |
| `SW_EXP` | [TCA9534DW](../TCA9534DW/TCA9534DW.md) | `I2C_SW` (`Wire`), `0x22` |
| `Motor_EXP` / `Rand_EXP` / `Button_EXP` | TCA9534DW | `Wire1`, `0x20` / `0x21` / `0x23` |
| `ADC_Motor` | [ADC108S102](../ADC108S102/ADC108S102.md) | `SPI1`, CS 32 |
| `BTN1`…`BTN5` | [fancyBTN](../fancyBTN/fancyBTN.md) | – |
| `PDS` | [PowerDebugger](../Debugger/PDS.md) | `UART_DBG` (`Serial3`) |
| `communication` | [commlib](../commlib/commlib.md) | – |
| `headingControllerA/P` | [ADRC](../Regler/ADRC/README.md) / [PID](../Regler/PID/README.md) | – |

---

## Konfiguration

### In `initialize()` fest gesetzt

| Einstellung | Wert |
|---|---|
| `Wire` / `Wire1` Takt | `400 000` Hz |
| `USBC_UART` Baudrate | `115200` |
| `INA219_*.setMaxCurrentShunt(...)` | `15` A (Shunt = Default 0,002 Ω) |
| SRF08 `setEMAAlpha(...)` | `0.25f` (alle vier) |
| SRF08 `setJumpThreshold(...)` | `50` cm (alle vier) |
| `setDebugTimeout(...)` | `5` ms |
| `setupHeadingController(...)` | `true` → **ADRC** |
| Expander-Richtung | `Motor_EXP`/`Rand_EXP` = Ausgang, `SW_EXP`/`Button_EXP` = Eingang |
| fancyBTN-Parameter | `(50, 6, 8, true)` → 100 ms Doppelklick, 3 s Long-Press, 8 ms Entprellung |

### Heading-Regler

`setupHeadingController(bool useADRC)` legt einen der beiden Regler an.
Umschalten erfolgt über dieses eine Argument – die restliche API
(`setHeading()`, `updateHeadingController()`, `giveHeadingController()`)
bleibt gleich.

**ADRC (Default, `useADRC = true`):**

| Parameter | Wert | Bedeutung |
|---|---|---|
| `order` | `SECOND` | Winkel = Strecke 2. Ordnung |
| `Ts` | `0.005f` | nur Fallback (`TS_NOMINAL`), es wird mit echtem `dt` gerechnet |
| `dtMin` / `dtMax` | `1e-5f` / `0.03f` | 10 µs … 30 ms |
| `wc` | `15.0f` | Reglerbandbreite – wie schnell ausgerichtet wird |
| `wo` | `80.0f` | Beobachterbandbreite (≈ 5× `wc`) |
| `b0` | `8.0f` | Streckenverstärkung, **grobe Schätzung → per Test anpassen** |
| `uMin` / `uMax` | `-1.0f` / `1.0f` | Stellgrößengrenzen |
| `angleMode` | `true` | kürzester Drehweg, kein ±π-Sprung |
| `outputRateLimit` | `6.0f` | sanfter Anlauf |

**PID (`useADRC = false`):**

| Parameter | Wert |
|---|---|
| `Kp` / `Ki` / `Kd` | `2.0f` / `0.5f` / `0.05f` |
| `Tf` | `0.02f` (D-Anteil filtern) |
| `derivativeOnMeasurement` | `true` |
| `angleMode` | `true` |
| `uMin` / `uMax` | `-1.0f` / `1.0f` |
| `outputRateLimit` | `6.0f` |
| `dtMin` / `dtMax` | `1e-5f` / `0.03f` |

Tuning-Anleitungen: [PID/README.md](../Regler/PID/README.md),
[ADRC/README.md](../Regler/ADRC/README.md)

Der Regler rechnet intern im **Bogenmaß** (`DEG_TO_RAD * BNO_OrbitDir`),
`setHeading()` erwartet den Sollwert deshalb ebenfalls in Radiant.

---

## Ablauf im Hauptprogramm

```cpp
#include <systemlib.h>

systemlib sys;

void setup() {
    sys.initialize();
}

void loop() {
    sys.readSensors();          // Sensoren + Regler + Motoren + Debug

    sys.setHeading(0.0f);       // Sollrichtung [rad]
    float u = sys.giveHeadingController();

    MotorData m = sys.giveMotorData(MotorVL_ID);
    m.Speed  = 50;              // 0..100 %
    m.Dir    = true;
    m.Enable = true;
    sys.setMotorData(m);

    sys.setDBGSys();                              // Systemkanäle
    sys.setDBGChannel(0, sys.giveCycleTime_us()); // eigene Kanäle 0..20
}
```

`readSensors()` führt in dieser Reihenfolge aus:

1. Zykluszeit erfassen (`cycleTime`, `cycleTime_us`)
2. `delay(debugTimeout)` – **blockierende Pause**, Default 5 ms
3. `readBNO()`, `readIR()` (`readUS()` ist auskommentiert)
4. `readExpanders()` (Schalter + Taster)
5. `PDS.update()` (Telemetrie senden/empfangen)
6. `updateHeadingController()`
7. `updateMotors()`

---

## MotorData

```cpp
struct MotorData
{
    uint8_t  ID;          // MotorVL_ID … MotorDR_ID

    float    Speed;       // Sollgeschwindigkeit 0..100 (%) — wird ×2.55 → PWM
    bool     Dir;         // Richtung
    bool     Enable;      // Freigabe
    bool     Break;       // Bremse (noch nicht ausgewertet)

    uint16_t Current;     // gemessener Strom (ADC)
    float    Speed_real;  // gemessene Drehzahl (ADC)
    bool     Dir_real;    // (noch nicht befüllt)
    float    Speed_diff;  // (noch nicht befüllt)
    bool     is_running;  // (noch nicht befüllt)
};
```

Ablauf: `giveMotorData(ID)` holen → Felder ändern → `setMotorData(...)`
zurückschreiben. `setMotorData()` ruft direkt `updateMotors()` auf. Eine
unbekannte ID liefert einen Datensatz mit `ID = 100`.

`updateMotors()` schreibt die PWMs direkt am Teensy, die Richtungs-/Enable-Bits
gesammelt über `Motor_EXP` und liest anschließend Drehzahl und Strom aller vier
Antriebsmotoren vom `ADC_Motor`.

---

## API-Überblick

### System

| Funktion | Beschreibung |
|---|---|
| `void initialize()` | Busse, Sensoren, Expander, Regler – einmal in `setup()` |
| `void readSensors()` | zyklisch in `loop()`, siehe Ablauf oben |
| `uint16_t giveCycleTime()` / `giveCycleTime_us()` | Zykluszeit in ms / µs |

### Sensorwerte

| Funktion | Rückgabe |
|---|---|
| `giveOrbitDir()` | Ausrichtung relativ zur kalibrierten Nullrichtung, ±180° |
| `giveBallAngle()` / `giveBallDistance()` | Ball im Roboter-KS |
| `giveRawBallAngle()` / `giveRawBallDistance()` | Ball roh vom IR-Ring |
| `giveRawSensorData(...)` / `giveTSSPData(...)` | Rohdaten des IR-Rings – **derzeit deaktiviert**, siehe `#if false` in `readIR()` |
| `giveUS_vo/re/hi/li()` | Ultraschall [cm] – **derzeit nicht aktualisiert** (`readUS()` auskommentiert) |
| `giveMainVoltage()` / `giveMainCurrent()` | Hauptstromkreis (V / mA) |
| `giveMotorVoltage()` / `giveMotorCurrent()` | Motorstromkreis; Strom über 6 Werte gemittelt |
| `giveButtonStates(uint8_t (&)[5])` | Statuscodes der fünf Taster |
| `giveOtherRobotData()` | Daten des Partnerroboters (commlib, noch TODO) |

### Kalibrierung

| Funktion | Beschreibung |
|---|---|
| `void calibBNO()` | Aktuelle Richtung als Nullrichtung übernehmen |
| `void calibIR()` | Min/Max je IR-Sensor mitschreiben und als Offset/Gain in den Ring schreiben – während der Kalibrierfahrt zyklisch aufrufen |
| `void writeIRCalibration(uint8_t nr, int16_t offset, int16_t gain)` | Einzelner Sensor |
| `void writeIRCalibration(const int16_t (&offsets)[16], const uint16_t (&gains)[16])` | Alle 16 Sensoren |

### Regelung

| Funktion | Beschreibung |
|---|---|
| `void setupHeadingController(bool useADRC = false)` | Regler anlegen/umschalten |
| `void setHeading(float Heading)` | Sollrichtung **in Radiant** |
| `void updateHeadingController()` | wird von `readSensors()` aufgerufen |
| `float giveHeadingController()` | Stellgröße −1…+1 |

### Debug

| Funktion | Beschreibung |
|---|---|
| `void setDBGSys()` | Systemkanäle 21–40 füllen |
| `void setDBGChannel(uint8_t ch, float v)` | Freie Kanäle **0–20** (`Util.Range` prüft das) |
| `void setDebugTimeout(uint16_t ms)` | Länge des `delay()` in `readSensors()` |

Kanalbelegung: [Channel.md](../Debugger/Channel.md)

### Kommunikation

| Funktion | Beschreibung |
|---|---|
| `void communicationSend()` | Aktuellen Puffer senden |
| `void communicationSend(float x, float y, float ballAngle, float ballDistance, float goalAngle, float goalDistance)` | Setzen und senden |

---

## Bekannte Baustellen

- `readSensors()` enthält ein `delay(debugTimeout)` (Default 5 ms). Das
  begrenzt die Zykluszeit hart auf ≥ 5 ms und blockiert dabei alles andere.
- `readUS()` ist in `readSensors()` auskommentiert → alle `giveUS_*()` liefern
  `0`.
- Die IR-Rohdaten (`read_raw_values`, `read_tssp`) sind über `#if false`
  deaktiviert; `giveRawSensorData()`/`giveTSSPData()` liefern damit nur
  Nullen. `calibIR()` liest sie separat selbst.
- `IR_Distance = IR_rawDistance` – die Umrechnung fehlt noch („Hier muss noch
  Mathe hin").
- Die Regler werden mit `new` angelegt und nie freigegeben; mehrfaches
  `setupHeadingController()` erzeugt ein Leck.
- `giveMotorCurrent()` gibt einen `int`-Mittelwert als `float` zurück – die
  Nachkommastellen der 6 Messungen gehen verloren.
- `MotorData::Break`, `Dir_real`, `Speed_diff` und `is_running` sind
  deklariert, werden aber nirgends geschrieben oder ausgewertet.
- `updateMotors()` verwendet `Port_List` teilweise uninitialisiert, falls nicht
  alle acht Indizes belegt werden.
- Der Dribbler (`MotorDR_D`) wird in `updateMotors()` nicht angesteuert.
- `src/Test-Controller.cpp` ruft `sys.resetHeadingController()` und
  `MotorVR_D`/`MoHR`/`MoVL`/`MoVR` auf – diese existieren in `systemlib` nicht.
  Die Datei ist in diesem Zustand nicht übersetzbar.
