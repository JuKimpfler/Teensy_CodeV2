# Beispiele

Diese Beispiele sind gekürzte/kommentierte Auszüge aus den tatsächlichen
Testskripten im Projekt. Vollständiger, lauffähiger Code steht in den
verlinkten Originaldateien.

## 1. Connectivity-Test (Basis-Klasse, firmware-unabhängig)

Quelle: [test/XM125_Connectivity.cpp](../../../test/XM125_Connectivity.cpp)
· Ausführen mit `pio test -e teensy40 -f XM125_Connectivity`

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

Quelle: [test/XM125_Distance.cpp](../../../test/XM125_Distance.cpp)
· Ausführen mit `pio test -e teensy40 -f XM125_Distance`

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
  if (!radar.measure(3000)) {
    if (radar.getCalibrationNeeded()) radar.recalibrate();
    return;
  }

  uint8_t n = radar.getNumDistances();
  Serial.print("Peaks: "); Serial.print(n);
  Serial.print("  Temp: "); Serial.println(radar.getTemperature());

  for (uint8_t i = 0; i < n; i++) {
    Serial.print("  Peak "); Serial.print(i);
    Serial.print(": raw="); Serial.print(radar.getPeakDistanceRaw(i));   // 0-255 Counts, unkalibriert
    Serial.print(" Staerke="); Serial.println(radar.getPeakStrength(i));
  }
}
```

## 3. Vier Distanzsensoren an einem I²C-Bus

Quelle: [src/XM125_Distance_Multi.cpp](../../../src/XM125_Distance_Multi.cpp)
· Ausführen mit `pio test -e teensy40 -f XM125_Distance_Multi`

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
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (!sensorReady[i]) { sensorReady[i] = setupSensor(i); continue; }

    XM125Distance& radar = sensors[i];
    if (radar.measure(3000)) {
      Serial.print("[0x"); Serial.print(SENSOR_ADDRS[i], HEX); Serial.print("] ");
      Serial.print("Peaks="); Serial.println(radar.getNumDistances());
    }
  }
}
```

## 4. Anwesenheitserkennung (Presence)

Quelle: [test/XM125_Presence.cpp](../../../test/XM125_Presence.cpp)
· Ausführen mit `pio test -e teensy40 -f XM125_Presence`

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
