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
| `INA219_Main` | [INA219](../INA219/INA219.md) | `I2C_MAIN` (`Wire1`), `0x40` |
| `INA219_Motor` | INA219 | `I2C_MAIN` (`Wire1`), `0x41` |
| `SW_EXP` / `Button_EXP` | [TCA9534DW](../TCA9534DW/TCA9534DW.md) | `I2C_SW` (`Wire`), `0x26` / `0x27` (SwitchBoard IC1 / IC2) |
| `Motor12_EXP` / `Motor34_EXP` / `Power_EXP` | TCA9534DW | `I2C_MAIN` (`Wire1`), `0x22` / `0x20` / `0x23` (Main IC9 / IC7 / IC8) |
| `ADC_Motor` | [ADC108S102](../ADC108S102/ADC108S102.md) | `SPI`, CS 30 |
| `ADC_Line1`…`ADC_Line6` | ADC108S102 | `SPI`, CS über 74HC138 an 31/32/33, Adresse 0…5 |
| `BTN1`…`BTN5` | [fancyBTN](../fancyBTN/fancyBTN.md) | – |
| `PDS` | [PowerDebugger](../PDS/PDS.md) | `UART_DBG` (`Serial1`) |
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
| Expander-Richtung | `Motor12_EXP`/`Power_EXP` = Ausgang, `Motor34_EXP` = Ausgang bis auf P6/P7, `SW_EXP`/`Button_EXP` = Eingang |
| Freigaben (`Power_EXP`) | VMOT, VDRIB, Lichtschranke, Linien-LEDs ein; Intercon-3V3 ein (LOW) |
| fancyBTN-Parameter | `(50, 6, 8, true)` → 100 ms Doppelklick, 3 s Long-Press, 8 ms Entprellung |

### Hardware ein/aus (`SYS_EN_*` in `params.h`)

Jede Platine/jeder Sensor laesst sich per Flag abschalten (`1` = an, Default).
Abgeschaltet wird weder eingerichtet noch gelesen, die Werte bleiben auf 0.
Ueberschreibbar auch per `build_flags`, z. B. `-DSYS_EN_RADAR=0`.

| Flag | Betrifft | Abgeschaltet |
|---|---|---|
| `SYS_EN_BNO` | BNO055 | Heading bleibt 0 |
| `SYS_EN_RADAR` | 4x XM125 | spart mehrere Sekunden Start; Position nur aus Odometrie |
| `SYS_EN_LINE` | 6x Linien-ADC | keine Linie erkannt, `giveADC()` = 0 |
| `SYS_EN_POWER` | INA219 Akku/Motor | Spannungen/Stroeme = 0 |
| `SYS_EN_SWITCHBOARD` | SW_EXP, Button_EXP | Schalter/Taster/Start/RCJ = false |
| `SYS_EN_MOTORS` | Motor12/34_EXP, ADC_Motor | `updateMotors()` tut nichts, PWM bleibt 0 |

`Power_EXP` (Versorgungsfreigaben) wird immer eingerichtet.

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
3. `readBNO()`, `Radar.update()`, `calculatePosition()` (`readIR()`/`readUS()` sind auskommentiert)
4. `readExpanders()` (Schalter + Taster)
5. Akku-/Motorwerte nachführen (gedrosselt auf 10 Hz) und `PDS.update()`
   (Telemetrie senden/empfangen) – **bedingungslos**, siehe unten
6. `updateHeadingController()`
7. `updateMotors()`

---

## Radar (XM125)

Die Radarsensoren stecken komplett in [RadarArray](../RadarArray/RadarArray.md).
Die `systemlib` hält davon genau ein Objekt und ruft es an zwei Stellen auf:
`initialize()` → `Radar.begin()`, `readSensors()` → `Radar.update()`.

```cpp
uint16_t d = sys.Radar[RadarVo].Dist;   // Rohwert Peak 0 ("Counts", NICHT mm)
if (sys.Radar[RadarLi].isCon) { ... }   // antwortet der Sensor?

sys.Radar.config.endMm = 4000;          // Abstimmung aendern ...
sys.calibRadar();                       // ... und uebernehmen (blockiert)
```

| Position | Index | Einbau |
|---|---|---|
| `RadarVo` | 0 | vorne |
| `RadarLi` | 1 | links |
| `RadarHi` | 2 | hinten |
| `RadarRe` | 3 | rechts |

```cpp
struct RadarData
{
    uint16_t Dist;      // Rohwert von Peak 0 ("Counts", NICHT mm — siehe XM125-Doku)
    int32_t  Strenght;  // Signalstärke von Peak 0
    uint8_t  Peaks;     // Anzahl erkannter Peaks (0 = nichts gesehen)
    bool     isCon;     // false, wenn der Sensor als tot gilt
    uint32_t Time;      // millis() der letzten übernommenen Messung (0 = noch nie)
};
```

Alle Sensoren messen **gleichzeitig**, `Radar.update()` **wartet nie** auf ein
Messergebnis — Details und Zeitbedarf siehe
[RadarArray.md](../RadarArray/RadarArray.md).

---

## Position aus den Radarwerten

`calculatePosition()` läuft in `readSensors()` direkt nach `Radar.update()` und
schreibt das Ergebnis nach `Position` — dort lesen es `moveToPoint()` und der
PDS.

```cpp
struct PositionData
{
    float X;          // cm, 0 = linke Feldwand
    float Y;          // cm, 0 = Wand hinter dem eigenen Tor
    float Certainty;  // 0 = unbrauchbar ... 1 = plausibles Paar, brandaktuell
    bool  AtGoal;     // Roboter steht vor einer Toröffnung (Y-Achse verkürzt)
};

PositionData p = sys.givePosition();
if (p.Certainty > 0.5f) { /* Position ist belastbar */ }
```

Je Achse arbeiten zwei **gegenüberliegende** Sensoren zusammen — X aus
`RadarLi`/`RadarRe`, Y aus `RadarHi`/`RadarVo`. Beide messen dieselbe Strecke
aus, also muss gelten: *Weg zur einen Fläche + Weg zur anderen Fläche =
Strecke dazwischen* (`span`, siehe [Tore](#tore) — ohne Tor ist das die volle
Feldgröße). Daran wird das Paar geprüft:

1. **Einzelprüfung** — der Sensor muss antworten (`isCon`), einen Peak haben
   (`Peaks > 0`) und eine Distanz liefern, die überhaupt ins Feld passt.
2. **Blindbereich** — misst ein Sensor mehr als `span - POS_BLIND_CM`,
   steht der Roboter dicht an der gegenüberliegenden Wand. Der Sensor dort ist
   näher dran als der Radar-Blindbereich (`RADAR_START_MM` = 20 cm) und kann die
   Wand gar nicht mehr erkennen — sein Wert fliegt raus. Melden **beide** so
   weit, widersprechen sie sich und beide fliegen raus.
3. **Paarprüfung** — passt die Summe zur Strecke, wird gemittelt. Passt sie
   nicht, verrät die Fehlerrichtung den Schuldigen: Summe zu **klein** → einer
   hat ein Hindernis vor der Wand gesehen (Gegner, Ball), der **längere** Weg
   gewinnt. Summe zu **groß** → einer hat die Wand verpasst (Mehrwegecho), der
   **kürzere** Weg gewinnt.

Bleibt nichts Brauchbares übrig, bleibt die letzte Position stehen und
`Certainty` wird `0`.

### Tore

Das Feld ist `POS_FIELD_X_CM` × `POS_FIELD_Y_CM` (182 × 243 cm, Wand zu Wand).
An **beiden** Enden steht aber mittig ein Tor `POS_GOAL_DEPTH_CM` weit im Feld.
Steht der Roboter davor, sieht der vordere bzw. hintere Sensor die
**Torrückwand statt der Feldwand** — die Y-Achse ist dort um die Tortiefe
kürzer, an beiden Enden zusammen also um `2 × POS_GOAL_DEPTH_CM`:

```
y = 243  ┌───────┬───────────┬───────┐
         │       │  Tor 10   │       │   vor dem Tor:  span = 243 - 2*10 = 223
y = 233  │       └───────────┘       │   sonst:        span = 243
         │        ← 60 cm →          │
         │                           │
y =  10  │       ┌───────────┐       │
         │       │  Tor 10   │       │
y =   0  └───────┴───────────┴───────┘
```

Ob der Sensor Tor oder Wand trifft, sagt allein **X** — deshalb rechnet
`calculatePosition()` erst die X-Achse (die Seitenwände laufen durch, dort gibt
es nichts zu korrigieren) und daraus den Tor-Offset für Y.

Die Torkante ist für den Radar kein scharfer Sprung: die Keule ist breiter als
ein Punkt. Im Band `POS_GOAL_EDGE_CM` (± 12 cm) um die Kante wird deshalb
linear zwischen beiden Fällen geblendet — der Fehler bleibt damit unter der
**halben** Tortiefe (max. 5 cm statt 10 cm) — und `Certainty` der Y-Achse fällt
auf `POS_PLAUS_GOAL_EDGE` (0.5), weil eben nicht feststeht, was der Sensor
gesehen hat. `PositionData.AtGoal` zeigt, ob die Korrektur greift.

> Ist ein Tor bei euch eine **Nische in der Wand** (Feld dort länger statt
> kürzer), einfach `POS_GOAL_DEPTH_CM` negativ eintragen — die Rechnung stimmt
> genauso. `POS_GOAL_DEPTH_CM = 0` schaltet die Torkorrektur komplett ab.

### Certainty

`Certainty = Plausibilität × Altersfaktor`, für die Position als Ganzes die
**schlechtere** der beiden Achsen — eine Achse, die keiner kennt, macht auch die
andere als Position wertlos.

| Plausibilität | Fall |
|---|---|
| `POS_PLAUS_PAIR` (1.00) | Summe des Paares trifft die Strecke genau; bis `POS_PAIR_TOL_CM` Abweichung linear auf `POS_PLAUS_SINGLE` fallend |
| `POS_PLAUS_SINGLE` (0.60) | nur ein Sensor nutzbar — keine Gegenprobe möglich |
| `POS_PLAUS_CONFLICT` (0.30) | Paar widerspricht sich, ein Wert wurde geraten |
| `0` | kein brauchbarer Wert |

Die Y-Achse wird zusätzlich mit `POS_PLAUS_GOAL_EDGE` (0.50) gewichtet, solange
der Roboter auf der Torkante steht (siehe [Tore](#tore)).

Der Altersfaktor kommt aus `Radar.giveAge(pos)` (Zeitstempel `RadarData.Time`)
und fällt linear von `1` bei 0 ms auf `0` bei `POS_AGE_MAX_MS` (1000 ms). Bei
einem Paar zählt der **ältere** der beiden Werte. Ein Sensor, der noch nie
gemessen hat, liefert ein Alter von `millis()` und damit `Certainty` `0`.

Alle Stellschrauben stehen in `params.h` unter *Positionsbestimmung aus den
Radarwerten*, *Feldgeometrie* und *Tore*. Dort sitzt auch `POS_RADAR_OFFSET_CM`: der Abstand vom
Roboter-Mittelpunkt zur Sensorfläche, der auf jede Messung addiert wird, weil
die Koordinaten ab Mittelpunkt zählen. **Noch nicht ausgemessen (0.0)** —
solange dort `0` steht, ist die Position um genau diesen Betrag zur Wand hin
verschoben.

> Die Rechnung setzt voraus, dass der Roboter zum Feld ausgerichtet ist
> (Heading-Regler), sonst schaut kein Sensor senkrecht auf seine Wand.

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

`updateMotors()` schreibt die PWMs (vier Räder + Dribbler) direkt am Teensy und
die ESCON-Steuerleitungen gesammelt über `Motor12_EXP` (IC9) und `Motor34_EXP`
(IC7): `Enable` → Freigabe (ESCON DI2), `Dir` → Richtung (DI3, `true` = LOW),
`Break` → Stopp (DI4, HIGH = Stopp; der Dribbler hat keine Stopp-Leitung).
Anschließend liest es Drehzahl und Strom der vier Antriebsmotoren vom
`ADC_Motor`. Welches Rad an welchem ESCON-Modul hängt, steht in
[`enum.h`](../include/include.md).

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
| `Radar[RadarVo/Li/Hi/Re]` | `RadarData` des Radarsensors (Direktzugriff, auch `giveRadar(pos)`) |
| `giveRadarCon(pos)` | antwortet der Radarsensor? |
| `giveRadarAge(pos)` | Alter des Radarwerts in ms |
| `givePosition()` | zuletzt berechnete `PositionData` (X/Y in cm + Certainty) |
| `givePositionCertainty()` | nur die Certainty, 0 … 1 |
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
| `void calibRadar()` | `Radar.config` in alle Radarsensoren schreiben und neu kalibrieren – **blockiert** ~1 s je Sensor |
| `void calibIR()` | Min/Max je IR-Sensor mitschreiben und als Offset/Gain in den Ring schreiben – während der Kalibrierfahrt zyklisch aufrufen |
| `void writeIRCalibration(uint8_t nr, int16_t offset, int16_t gain)` | Einzelner Sensor |
| `void writeIRCalibration(const int16_t (&offsets)[16], const uint16_t (&gains)[16])` | Alle 16 Sensoren |

### Regelung

| Funktion | Beschreibung |
|---|---|
| `void setupHeadingController(bool useADRC = false)` | Regler anlegen/umschalten |
| `void setHeading(float Heading)` | Sollrichtung **in Radiant** |
| `void updateHeadingController()` | wird von `readSensors()` aufgerufen |
| `PositionData calculatePosition()` | Feldposition + Certainty aus den Radarwerten, wird von `readSensors()` aufgerufen |
| `float giveHeadingController()` | Stellgröße −1…+1 |

### Debug (PDS)

| Funktion | Beschreibung |
|---|---|
| `void setupPDS()` | *privat*, aus `initialize()`. Bindet die Systemwerte an die **festen** Kanäle 0–25 und setzt die Firmware-Version. |
| `void setDebugTimeout(uint16_t ms)` | Länge des `delay()` in `readSensors()` |

`setupPDS()` benutzt `PDS.bind()`, merkt sich also nur Zeiger – die Werte holt
die Bibliothek danach selbst mit 100 Hz ab. Im Sketch ist dafür nichts zu tun;
eigene Werte kommen mit `PDS.plot()` / `PDS.track()` dazu und bekommen
automatisch Kanäle ab 32 (`PDS_AUTO_CHANNEL_BASE`).

Zwei Dinge sind Absicht und sollten so bleiben:

* **`PDS.update()` läuft in jedem Zyklus**, nicht nur wenn `PDS.linkOk()`.
  Erst `update()` wertet die RX-Leitung aus. Hinge der Aufruf an `linkOk()`,
  bliebe die Verbindung nach dem ersten Aussetzer für immer tot. `update()`
  blockiert nie – die Bibliothek hat dafür ein Zeitbudget je Aufruf.
* **Akku- und Motorwerte werden auf 10 Hz gedrosselt.** Die INA219 hängen am
  I2C; vier Abfragen im 100-Hz-Takt würden die Zykluszeit spürbar verlängern,
  und schneller ändert sich eine Akkuspannung ohnehin nicht.

Kanalbelegung: [docs/PDS_Belegung.md](../../docs/PDS_Belegung.md)
Einbau der Bibliothek: [lib/PDS/PDS.md](../PDS/PDS.md)

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
