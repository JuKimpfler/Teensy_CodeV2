# Beispiele

Diese Beispiele sind gekürzte/kommentierte Auszüge aus den tatsächlichen
Skripten im Projekt. Vollständiger, lauffähiger Code steht in den verlinkten
Originaldateien unter [lib/XM125/examples/](../examples/README.md) — dort
auch eine ausführliche Erklärung, was jedes Skript genau tut und wie es zu
bedienen ist.

## 1. Connectivity-Test (Basis-Klasse, firmware-unabhängig)

Quelle: [XM125_Connectivity.cpp](../examples/XM125_Connectivity.cpp)
· Ausführen: nach `src/main.cpp` kopieren, `pio run -e teensy40 -t upload`

Prüft I²C-Erreichbarkeit, Wake/Ready-Sequenz sowie Version- und
Status-Register — unabhängig davon, ob Distance- oder Presence-Firmware
geflasht ist.

```cpp
#include <Wire.h>
#include "XM125.h"

XM125Base radar(XM125_I2C_ADDR_DEFAULT, /*wakePin=*/-1, /*intPin=*/-1);

void setup() {
  Wire.begin();
  Wire.setClock(100000);

  if (!radar.begin(Wire)) {
    Serial.println("FEHLER: Sensor nicht erreichbar!");
    return;
  }

  Serial.print("Version: 0x"); Serial.println(radar.getVersionRaw(), HEX);
  Serial.print("Protocol Status: 0x"); Serial.println(radar.getProtocolStatus(), HEX);
  Serial.print("Detector Status: 0x"); Serial.println(radar.getDetectorStatus(), HEX);
}

void loop() {
  delay(2000);
  Serial.println(radar.isConnected() ? "Verbindung ok" : "Verbindung verloren");
}
```

## 2. Einzelsensor-Distanzmessung

Quelle: [XM125_Distance.cpp](../examples/XM125_Distance.cpp)
· Ausführen: nach `src/main.cpp` kopieren, `pio run -e teensy40 -t upload`

```cpp
#include <Wire.h>
#include "XM125Distance.h"

XM125Distance radar(XM125_I2C_ADDR_DEFAULT, -1, -1);

bool setupSensor() {
  if (!radar.begin(Wire)) return false;

  radar.config.startMm         = 200;
  radar.config.endMm           = 2500;
  radar.config.maxProfile      = XM125D_PROFILE5;
  radar.config.thresholdMethod = XM125D_THRESHOLD_CFAR;
  radar.config.peakSorting     = XM125D_SORT_STRONGEST;

  return radar.applyConfigAndCalibrate(); // kann bis zu einigen Sekunden dauern
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  setupSensor();
}

void loop() {
  if (radar.isMeasuring()) return;        // laeuft noch -- nicht warten

  const XM125DistanceResult& res = radar.getLastResult();
  if (!res.valid) {
    if (res.calibrationNeeded) radar.recalibrate();
  } else {
    Serial.print("Peaks: "); Serial.print(res.numDistances);
    Serial.print("  Temp: "); Serial.println(res.temperature);

    for (uint8_t i = 0; i < res.numDistances; i++) {
      Serial.print("  Peak "); Serial.print(i);
      Serial.print(": raw="); Serial.print(res.peakDistanceRaw[i]);   // 0-255 Counts, unkalibriert
      Serial.print(" Staerke="); Serial.println(res.peakStrength[i]);
    }
  }

  radar.startMeasurement();               // naechste Runde
}
```

## 3. Vier Distanzsensoren an einem I²C-Bus

Quelle: [XM125_Distance_Multi.cpp](../examples/XM125_Distance_Multi.cpp)
· Ausführen: nach `src/main.cpp` kopieren, `pio run -e teensy40 -t upload`

Verkabelung: alle 4 Sensoren an `Wire` (SDA/SCL), unterschiedliche
Hardware-Adressen `0x51`/`0x52`/`0x53`/`0x54`.

```cpp
#include <Wire.h>
#include "XM125Distance.h"

static constexpr uint8_t NUM_SENSORS = 4;
static constexpr uint8_t SENSOR_ADDRS[NUM_SENSORS] = { 0x51, 0x52, 0x53, 0x54 };

XM125Distance sensors[NUM_SENSORS] = {
  XM125Distance(SENSOR_ADDRS[0], -1, -1),
  XM125Distance(SENSOR_ADDRS[1], -1, -1),
  XM125Distance(SENSOR_ADDRS[2], -1, -1),
  XM125Distance(SENSOR_ADDRS[3], -1, -1),
};
bool sensorReady[NUM_SENSORS] = {};

bool setupSensor(uint8_t idx) {
  XM125Distance& radar = sensors[idx];
  if (!radar.begin(Wire)) return false;

  radar.config.startMm = 200;
  radar.config.endMm   = 2500;
  return radar.applyConfigAndCalibrate();
}

void setup() {
  Wire.begin();
  Wire.setClock(100000);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) sensorReady[i] = setupSensor(i);
}

void loop() {
  // Alle Sensoren messen gleichzeitig: nur nachsehen, nie warten.
  bool laeuft = false;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (!measurePending[i]) continue;
    if (sensors[i].isMeasuring()) { laeuft = true; continue; }

    measurePending[i] = false;
    Serial.print("[0x"); Serial.print(SENSOR_ADDRS[i], HEX); Serial.print("] ");
    Serial.print("Peaks="); Serial.println(sensors[i].getLastResult().numDistances);
  }
  if (laeuft) return;

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {          // naechste Runde
    if (!sensorReady[i]) { sensorReady[i] = setupSensor(i); continue; }
    measurePending[i] = sensors[i].startMeasurement();
  }
}
```

## 4. Anwesenheitserkennung (Presence)

Quelle: [XM125_Presence.cpp](../examples/XM125_Presence.cpp)
· Ausführen: nach `src/main.cpp` kopieren, `pio run -e teensy40 -t upload`

```cpp
#include <Wire.h>
#include "XM125Presence.h"

XM125Presence radar(XM125_I2C_ADDR_DEFAULT, -1, -1);

bool setupSensor() {
  if (!radar.begin(Wire)) return false;

  radar.config.startMm = 300;
  radar.config.endMm   = 2000;

  if (!radar.applyConfiguration()) return false;
  return radar.start();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(100000);
  setupSensor();
}

void loop() {
  if (!radar.update()) { delay(200); return; }

  if (radar.hasDetectorError()) {
    radar.stop();
    radar.start();
    return;
  }

  Serial.print("Anwesenheit: "); Serial.print(radar.isPresenceDetected() ? "JA" : "nein");
  if (radar.isPresenceDetected()) {
    Serial.print("  Distanz: "); Serial.print(radar.getPresenceDistanceMm()); Serial.print(" mm");
  }
  Serial.print("  Intra="); Serial.print(radar.getIntraPresenceScore());
  Serial.print("  Inter="); Serial.println(radar.getInterPresenceScore());

  delay(100);
}
```

## 5. Kalibrierung auf eine gerade Holzwand (bis 2,5 m)

Quelle: [XM125_WallCalibration.cpp](../examples/XM125_WallCalibration.cpp)
· Ausführen: nach `src/main.cpp` kopieren, `pio run -e teensy40 -t upload`

Interaktives Werkzeug über die serielle Konsole. Es beantwortet die beiden
Fragen getrennt, die bei "das Radar erkennt die Wand nicht" gern
durcheinandergehen:

| Frage | Modus |
|---|---|
| Wird die Wand überhaupt als Peak erkannt? | `m` (Dauermessung), `s` (Parameter-Sweep) |
| Welche echte Distanz gehört zu welchem Rohwert? | `d <cm>` (Punkt aufnehmen), `f` (auswerten) |
| Stören sich die Sensoren, wenn sie gleichzeitig messen? | `i` (Übersprech-Test) |

**Ablauf für die Empfindlichkeit** (das, was "reagiert auf die Wand" bestimmt):

1. Wand auf den *größten* gewünschten Abstand stellen — 2,5 m. Dort ist die
   Erkennung am schwierigsten, näher wird sie nur besser.
2. `s` eingeben. Der Sketch fährt 16 Kombinationen aus `signalQuality` ×
   `thresholdSensitivity` ab, misst jede 20-mal und meldet Trefferquote,
   Signalstärke und Messzeit.
3. Der Sketch empfiehlt am Ende die *günstigste* Kombination mit 20/20
   Treffern (möglichst wenig Signalqualität = kurze Messzeit, möglichst wenig
   Empfindlichkeit = wenig Fehlalarme). Diese Zahlen nach
   [params_radar.h](../../include/params/params_radar.h) übertragen.

Ausgabeformat (Zahlen hier nur als Beispiel, nicht gemessen):

```
 SigQual  Sens  Erkannt  Rohwert  Staerke   Zeit
 -------  ----  -------  -------  -------   ----
  15000   500     3/20      41      1120    28 ms
  15000   750    14/20      42      1180    28 ms
  25000   750    20/20      42      1240    46 ms
```

**Ablauf für die Distanz-Zuordnung:** siehe
[02-Distance-Detector.md](02-Distance-Detector.md#kalibrieren-mit-testxm125_wallcalibrationcpp).

**Ablauf für das Übersprechen:** Wand ebenfalls auf 2,5 m, dann `i`. Der Sketch
misst denselben Sensor einmal allein und einmal, während alle anderen
gleichzeitig senden, und sagt daraus, ob der Gleichzeitigbetrieb Trefferquote
kostet. Hintergrund:
[05-Fehlerbehandlung-und-Troubleshooting.md](05-Fehlerbehandlung-und-Troubleshooting.md#8-übersprechen-zwischen-mehreren-sensoren).

> Der Messaufbau entscheidet über das Ergebnis: senkrecht auf die Fläche
> ausrichten (schon 10–15° Schräglage lenken die Reflexion an glattem Holz
> weg vom Sensor), Abstand ab Sensor-Vorderkante messen, und im Messkegel
> (ca. ±25°) sonst nichts stehen lassen.
