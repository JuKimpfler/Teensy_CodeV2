# positioning – Position im Feld aus Radar + Kamera

Bestimmt die Roboterposition `(x, y)` aus den vier XM125-Radarsensoren und den
beiden Kameras. **Die Radarsensoren haben immer Vorrang**, die Kamera ist nur
Schiedsrichter für den Fall, dass die Radarwerte in sich nicht plausibel sind.

Dateien: `positioning.h`, `positioning.cpp`

---

## Koordinatensystem

Nach `test/Coding_Rules.md`:

```
        gegnerisches Tor  (y = 243)
   +----------------------------------+
   |                                  |
 y |            * (x,y)               |
   |                                  |
   +----------------------------------+
  (0,0)        eigenes Tor            x = 182
```

| Größe             | Bedeutung                                                        |
|-------------------|------------------------------------------------------------------|
| `x` [cm]          | 0 = linke Wand … `fieldWidth_cm` (182)                            |
| `y` [cm]          | 0 = eigene Torlinie … `fieldLength_cm` (243)                      |
| `x_coord/y_coord` | Index im 2 cm-Raster, `x_coord = round(x / POS_GRID_CM)`          |
| `heading`         | ±180° im Uhrzeigersinn, 0° = Front zeigt Richtung +y (Gegnertor) |

Also z.B. `Util.AMap(bno.eulHeading())` als `heading` übergeben.

---

## Ablauf eines `update()`

1. **Sensorgeometrie** – aus `heading` (+ optionaler Servokorrektur) wird für
   jeden Radarsensor bestimmt, auf welche Wand er schaut. Sensoren, die
   schräger als `maxTiltDeg` (Default 20°) stehen, zählen für keine Achse.
   Bei kleiner Schrägstellung wird die Strahllänge mit `cos(tilt)` auf die
   Achse projiziert (`perpendicularModel = true`, falls der Sensor bei
   Schrägstellung stattdessen den Lotabstand misst).

2. **Radar zuerst** – für jede Achse werden alle Peak-Kombinationen der beiden
   gegenüberliegenden Sensoren gesucht, deren Summe (inkl. 2 × Ringradius) der
   Feldbreite bzw. Feldlänge entspricht (`pairTol_cm`, Default ±12 cm).
   *Gibt es genau ein solches Paar, ist die Achse fertig – die Kamera wird für
   diese Achse gar nicht erst ausgewertet.* Mehrere Paare, die auf (fast) die
   gleiche Position zeigen (`pairAgree_cm`), gelten dabei als ein Paar.

3. **Kamera nur bei Bedarf** – erst wenn eine Achse kein oder mehrere
   widersprüchliche Paare liefert, wird trianguliert: beide Torpeilungen
   werden geschnitten. Steht der Roboter nahe der Mittellinie, sind beide
   Peilungen fast parallel (`camMinQuality`) und die Triangulation entfällt;
   ist dann bereits eine Achse aus dem Radar bekannt, reicht eine einzelne
   Torpeilung, um die andere Achse zu bestimmen.

4. **Validieren** – mit der Kameraposition als Referenz wird entschieden,
   welcher Peak die Wand ist:
   * mehrere mögliche Wandpaare → das Paar, das am besten zur Kamera passt;
   * kein Paar (z.B. ein Sensor durch einen Gegner verdeckt) → der beste
     **Einzelwert** eines der beiden Sensoren, sofern er mit `camTol_cm`
     (Default 40 cm) grob zur Kamera passt. Die Kamera darf also ruhig ~30 cm
     danebenliegen, ohne den Radarwert zu verwerfen.
   * Ist keine Kamerareferenz verfügbar, wird die **letzte Position** mit
     `holdTol_cm` als Referenz benutzt.
   * Ist beides nicht verfügbar (erster Zyklus nach dem Einschalten), gilt der
     **stärkste** plausible Peak als Wand.

5. **Fallback** – erst wenn gar kein Radarwert brauchbar ist, wird die
   Kameraposition direkt übernommen (`AxisSource::Camera`), sonst die letzte
   Position gehalten (`AxisSource::Hold`).

6. **Peak-Klassifizierung** – mit der fertigen Position wird für jeden Sensor
   der erwartete Wandabstand berechnet (Strahl/Feld-Schnitt, auch über Ecken)
   und jeder Peak einsortiert:

   | `PeakType` | Bedeutung                                          |
   |------------|----------------------------------------------------|
   | `Wall`     | passt zum erwarteten Wandabstand (`wallTol_cm`)     |
   | `Robot`    | näher als die Wand → Hindernis / gegnerischer Roboter |
   | `Behind`   | weiter als die Wand → Mehrfachreflexion / Ausreißer |
   | `Unknown`  | kein Peak / Sensor ungültig                        |

---

## API

```cpp
void      begin(const Config& cfg = Config());
void      reset(float x_cm, float y_cm);        // z.B. Kickoff-Position
position  update(float headingDeg,
                 const CamReading& camOpponent, // Kamera auf das GEGNERISCHE Tor
                 const CamReading& camOwn,      // Kamera auf das EIGENE Tor
                 const RadarReading radar[RADAR_COUNT]);
position  getPosition();
const Status& getStatus();                       // Diagnose
PeakType  getPeakType(uint8_t sensor, uint8_t peakIdx);
float     radarServoAngle(float headingDeg);     // Ring feldparallel halten
float     rawCountToCm(uint8_t raw);             // XM125-Rohwert -> cm
```

`radar[]` erwartet die Ringreihenfolge `RADAR_FRONT, RADAR_RIGHT, RADAR_BACK,
RADAR_LEFT` (0°, 90°, 180°, 270° zur Roboterfront, anpassbar über
`Config::mountAngleDeg`).

### `Status` (Diagnose)

* `axis[AXIS_X|AXIS_Y]` – `coord_cm`, `source` (`RadarPair`, `RadarSingle`,
  `Camera`, `Hold`), `camUsed`, `ambiguous`, `pairCount`, `wallPeak[2]`
* `camValid`, `camTriangulated`, `camEvaluated`, `camX/camY`, `camQuality`
* `peak[Sensor][Index]` – Klassifizierung aller Peaks
* `sensorFacingDeg/sensorAxis/sensorSign/sensorTiltDeg`
* `confidence` – 0…1 (beide Achsen aus einem eindeutigen Radarpaar → 1.0)

---

## Kalibrierung der Rohwerte ⚠

Der XM125 liefert mit der aufgespielten Firmware nur einen **unkalibrierten
Rohwert 0–255** (siehe Warnung bei `XM125Distance::getPeakDistanceRaw()`).
`rawCountToCm()` rechnet linear um:

```cpp
distanz_cm = rawOffset_cm + raw * rawScale_cm;
```

Die Defaults (`20.0` / `0.902`) entsprechen der Annahme, dass die 255 Counts
den konfigurierten Messbereich `startMm = 200 … endMm = 2500` gleichmäßig
abbilden. **Das muss mit einem Objekt auf exakt bekannter Distanz überprüft
werden**, sonst stimmen Feldmaß-Vergleich und Paarbildung nicht. Alternativ die
Distanzen selbst umrechnen und direkt in `RadarPeak::distance_cm` schreiben.

---

## Beispiel

```cpp
#include "positioning.h"
#include "XM125Distance.h"
#include "Cam.h"
#include "BNO055.h"
#include "Utils.h"

static constexpr uint8_t ADDR[4] = {0x51, 0x52, 0x53, 0x54}; // vo, re, hi, li
XM125Distance radarSensor[4] = {
    XM125Distance(ADDR[0]), XM125Distance(ADDR[1]),
    XM125Distance(ADDR[2]), XM125Distance(ADDR[3])};

BNO055 bno(BNO_ADDRESS, &I2C_BNO);
CamC   camOpp, camOwn;

void setup() {
  Wire.begin();
  bno.init();
  camOpp.init(Serial6, 115200);
  camOwn.init(Serial7, 115200);

  for (uint8_t i = 0; i < 4; i++) {
    radarSensor[i].begin(Wire);
    radarSensor[i].config.startMm = 200;
    radarSensor[i].config.endMm   = 2500;
    radarSensor[i].config.peakSorting     = XM125D_SORT_CLOSEST;
    radarSensor[i].config.reflectorShape  = XM125D_REFLECTOR_PLANAR;
    radarSensor[i].applyConfigAndCalibrate();
  }

  Positioning::Config cfg;
  cfg.radarRingRadius_cm = 9.0f;   // Sensorstirn -> Roboterzentrum
  cfg.rawOffset_cm       = 20.0f;  // == startMm, siehe Kalibrierung!
  cfg.rawScale_cm        = 0.902f; // == (endMm - startMm) / 255 / 10
  Positioning::begin(cfg);
}

void loop() {
  float heading = Util.AMap(bno.eulHeading());

  camOpp.Update(heading);
  camOwn.Update(heading);

  Positioning::CamReading co, cw;
  co.valid = camOpp.isValid(); co.angle_field_deg = camOpp.give_Angle(); co.blobHeight_px = camOpp.give_BlobH();
  cw.valid = camOwn.isValid(); cw.angle_field_deg = camOwn.give_Angle(); cw.blobHeight_px = camOwn.give_BlobH();

  Positioning::RadarReading rd[Positioning::RADAR_COUNT];
  for (uint8_t i = 0; i < Positioning::RADAR_COUNT; i++) {
    rd[i].valid = radarSensor[i].measure();
    if (!rd[i].valid) continue;
    uint8_t n = radarSensor[i].getNumDistances();
    if (n > Positioning::RADAR_MAX_PEAKS) n = Positioning::RADAR_MAX_PEAKS;
    rd[i].peakCount = n;
    for (uint8_t p = 0; p < n; p++) {
      rd[i].peaks[p].distance_cm = Positioning::rawCountToCm(radarSensor[i].getPeakDistanceRaw(p));
      rd[i].peaks[p].strength    = radarSensor[i].getPeakStrength(p);
    }
  }

  Positioning::position pos = Positioning::update(heading, co, cw, rd);

  // Gegner vor dem vorderen Sensor?
  bool gegnerVorne = false;
  for (uint8_t p = 0; p < rd[Positioning::RADAR_FRONT].peakCount; p++)
    if (Positioning::getPeakType(Positioning::RADAR_FRONT, p) == Positioning::PeakType::Robot)
      gegnerVorne = true;
}
```

---

## Konfiguration (`Positioning::Config`)

| Feld | Default | Bedeutung |
|------|---------|-----------|
| `fieldWidth_cm` / `fieldLength_cm` | 182 / 243 | Feldmaße |
| `radarRingRadius_cm` | 9 | Sensorstirn → Roboterzentrum |
| `mountAngleDeg[4]` | 0/90/180/270 | Montagewinkel zur Front |
| `hasRadarServo`, `servoNeutralDeg`, `servoRangeDeg`, `servoInverted` | false, 90, 45, false | Ring wird gegen die Drehung gekontert (`radarServoAngle()`) |
| `maxTiltDeg` | 20 | max. Schrägstellung, damit ein Sensor für eine Achse zählt |
| `perpendicularModel` | false | Sensor misst bei Schrägstellung den Lotabstand statt der Strahllänge |
| `pairTol_cm` | 12 | Toleranz für „Summe == Feldmaß" |
| `pairAgree_cm` | 8 | Paare darunter gelten als dasselbe |
| `camTol_cm` | 40 | erlaubte Abweichung Radar ↔ Kamera |
| `holdTol_cm` | 35 | erlaubte Abweichung Radar ↔ letzte Position |
| `wallTol_cm` | 10 | Fenster für die Peak-Klassifizierung |
| `minWallDist_cm` | 4 | näher kann keine Wand sein |
| `outOfField_cm` | 15 | erlaubtes Überschießen über die Feldgrenze |
| `minPeakStrength` | 0 (aus) | schwächere Peaks ignorieren |
| `camMinBlobHeight_px` | 4 | kleinere Blobs sind zu unsicher |
| `camMinQuality` | 0.30 | Mindest-Schnittwinkel der beiden Peilungen |
| `camMinAxisSin` | 0.30 | Mindest-Projektion für die Einzelpeilung |
| `outputAlpha` | 1.0 (aus) | optionale EMA-Glättung der Ausgabe |
| `rawOffset_cm` / `rawScale_cm` | 20 / 0.902 | XM125-Rohwertumrechnung |

---

## Bekannte Grenzen

* Bei einem Heading nahe 45° (ohne Ring-Servo) steht **kein** Sensor
  achsparallel – dann liefern beide Achsen nur `Camera` oder `Hold`. Entweder
  `hasRadarServo` nutzen oder `maxTiltDeg` erhöhen (kostet Genauigkeit, weil
  die `cos`-Projektion bei großen Winkeln ungenau wird).
* Steht der Roboter genau auf der Mittellinie (`x ≈ 91`), liegen beide Tore in
  einer Flucht: die Kamera kann dort prinzipiell kein `y` liefern. In dem Fall
  greift die letzte Position als Referenz.
* Liegt die Kameraposition zufällig näher an einem Gegner-Peak als an der Wand,
  wird der falsche Peak gewählt. `camTol_cm` klein halten, sobald bekannt ist,
  wie genau die Triangulation im Feld wirklich ist.
