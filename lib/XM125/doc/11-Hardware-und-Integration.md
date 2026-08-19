# Hardware, Verkabelung & Integration ins Projekt

---

## 1. Was das Modul ist

Das **Acconeer XM125** ist ein fertiges Modul, das zwei Dinge kombiniert:

- den **A121** — einen gepulsten kohärenten 60-GHz-Radarchip (misst über die
  Laufzeit *und* Phase des Echos),
- eine **STM32-MCU**, auf der eine der fertigen Acconeer-Applikationen läuft
  (Distance Detector oder Presence Detector) und die nach außen ein
  **I²C-Registerinterface** anbietet.

Der Teensy spricht also nie direkt mit dem Radarchip, sondern immer mit der
Applikation auf der Modul-MCU. Das erklärt viele Eigenheiten: Kommandos
brauchen Zeit (`BUSY`), das Modul kann den Bustakt strecken, und der
Funktionsumfang hängt an der **geflashten Firmware** — nicht am Chip.

> Welche Firmware auf einem Modul läuft, erkennt die Library **nicht**
> automatisch. Passt die Klasse (`XM125Distance` / `XM125Presence`) nicht zur
> Firmware, äußert sich das typischerweise in `getProtocolStatus() != 0`.

## 2. Anschlüsse

| Modul-Pin | Funktion | In diesem Projekt |
|---|---|---|
| `3V3` | Versorgung 3,3 V | vom Teensy-3V3 bzw. Board-Regler |
| `GND` | Masse | gemeinsame Masse mit dem Teensy — **Pflicht** |
| `SDA` | I²C-Daten | Teensy `Wire` (Pin 18) |
| `SCL` | I²C-Takt | Teensy `Wire` (Pin 19) |
| `WAKE_UP` | Eingang: High = wach, Low = Schlaf | **unverkabelt** (bewusste Projektentscheidung, kein Pull-up) |
| `MCU_INT` | Ausgang: High = bereit / Detektion (je nach Konfiguration) | **unverkabelt** |
| `UART TX/RX` | Debug-Log des Moduls | unverkabelt (nur nötig für `ENABLE_UART_LOGS`) |

**Spannung:** Das Modul ist ein 3,3-V-Gerät. Der Teensy 4.0 ist ebenfalls
3,3 V und **nicht 5-V-tolerant** — es darf nirgends 5 V auf den Bus gelangen.

**Pull-ups:** I²C braucht Pull-up-Widerstände nach 3V3 (typisch 2,2 kΩ–4,7 kΩ,
je einer auf SDA und SCL für den **gesamten** Bus, nicht pro Gerät). Bei vier
Modulen an einem Bus mit längeren Leitungen eher 2,2 kΩ wählen — steigende
Buskapazität verschleift sonst die Flanken.

### WAKE_UP unverkabelt — warum das hier in Ordnung ist

Aus dem Header-Kommentar von [XM125.h](../XM125.h) (**✔ Code**):

- `WAKE_UP` weckt die Modul-MCU aus dem Hibernate. Der Schlafzustand entsteht
  **nur** durch expliziten Host-Eingriff (Pin aktiv Low ziehen oder
  Low-Power-Kommando).
- Diese Library sendet **nie** ein Sleep-Kommando, und `sleep()` wirkt nur bei
  verkabeltem `wakePin`. Ohne Verkabelung bleibt das Modul also dauerhaft wach.
- **Restrisiko:** Ein offener, floatender Eingang ist störanfälliger (EMI) als
  ein fest beschalteter. Für Dauerbetrieb am Netzteil wurde das bewusst in
  Kauf genommen.

Wer maximale Robustheit will, legt `WAKE_UP` per Pull-up fest auf 3V3 oder
verkabelt ihn auf einen GPIO und übergibt diesen als `wakePin`.

## 3. I²C-Adressen

| Quelle | Adresse |
|---|---|
| `XM125_I2C_ADDR_DEFAULT` (Konstante im Code) | `0x55` |
| Kommentar im Konstruktor-Doxygen | `0x52` (widersprüchlich, siehe Kapitel 05) |
| Tatsächlich verwendete Module im Projekt | `0x51`, `0x52`, `0x53`, `0x54` |

Die Adresse lässt sich **nicht per Software ändern** — sie ergibt sich aus
Hardware-/Firmwarekonfiguration des jeweiligen Moduls. Für den Aufbau mit vier
Sensoren muss jedes Modul eine eigene Adresse haben; welche das ist, sollte am
Modul beschriftet sein.

**Im Zweifel scannen:**

```cpp
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Wire.begin();
  Wire.setClock(100000);

  Serial.println("I2C-Scan...");
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print("  gefunden: 0x"); Serial.println(a, HEX);
    }
  }
  Serial.println("fertig.");
}

void loop() {}
```

## 4. Bustakt: warum 100 kHz

Alle Testskripte setzen:

```cpp
Wire.begin();
Wire.setClock(100000);   // niedriger Takt, mehr Toleranz für Clock-Stretching
```

Die Modul-MCU streckt den Takt, sobald sie nach einem Kommando mit der
Verarbeitung beginnt. Bei 400 kHz reagieren manche Host-Treiber darauf mit
Timeouts. 100 kHz kostet Buszeit (siehe
[10-Interna-und-Timing.md](10-Interna-und-Timing.md#5-zeitbudget-bei-100-khz-bustakt)),
ist dafür deutlich unempfindlicher.

## 5. Teensy-4.0-Busse

| Bus | SDA | SCL |
|---|---|---|
| `Wire` | 18 | 19 |
| `Wire1` | 17 | 16 |
| `Wire2` | 25 | 24 |

Alle vier Radarmodule hängen im Projekt an `Wire`. Wer die Abfragerate erhöhen
will, kann sie auf zwei Busse verteilen (z.B. je zwei an `Wire` und `Wire1`) —
`begin()` nimmt den gewünschten Bus als Parameter:

```cpp
sensorVorne.begin(Wire);
sensorHinten.begin(Wire1);
```

Beim Verteilen daran denken: Jeder Bus braucht eigene Pull-ups, und
`Wire1.begin()` muss ebenfalls aufgerufen werden.

## 6. Sensorring (4 Module)

Im Projekt sitzen vier Distance-Module im 90°-Raster auf einem Ring
(vorne / rechts / hinten / links). Die Positionsbibliothek geht davon aus
([lib/positioning](../../positioning/positioning.md)):

| Größe | Wert | Bedeutung |
|---|---|---|
| `mountAngleDeg[]` | `{0, 90, 180, 270}` | Montagewinkel im Uhrzeigersinn |
| `radarRingRadius_cm` | `9.0` | Abstand Sensorstirn → Roboterzentrum |
| `RADAR_MAX_PEAKS` | `10` | passend zu `XM125D_MAX_PEAKS` |

Der Ringradius wird **in der Positionsbibliothek** aufgeschlagen — die
XM125-Library liefert die Distanz ab Sensorstirn. Nicht doppelt einrechnen.

## 7. PlatformIO — Build & Tests

Konfiguration: [platformio.ini](../../../platformio.ini)

```ini
[env:teensy40]
platform  = teensy
board     = teensy40
framework = arduino
monitor_speed   = 115200
upload_protocol = teensy-gui
```

Die Library liegt unter `lib/XM125/` und wird von PlatformIO automatisch
gefunden (Library Dependency Finder). Es genügt daher:

```cpp
#include "XM125Distance.h"   // bzw. XM125Presence.h / XM125.h
```

### Beispiele ausführen

Die Beispiele liegen unter [lib/XM125/examples/](../examples/) (nicht in
`test/` — siehe [examples/README.md](../examples/README.md) für die
Übersicht). Jedes Beispiel ist eine einzelne `.cpp`-Datei mit eigenem
`setup()`/`loop()`; zum Bauen die gewünschte Datei als `src/main.cpp` des
Hauptprojekts verwenden (in `src/` darf immer nur eine Datei mit
`setup()`/`loop()` liegen):

```bash
cp lib/XM125/examples/XM125_Connectivity.cpp src/main.cpp
pio run -e teensy40 -t upload
```

Danach den seriellen Monitor öffnen:

```bash
pio device monitor -b 115200
```

### Vorgehen bei der Inbetriebnahme eines neuen Aufbaus

1. **`XM125_Connectivity`** — antwortet das Modul überhaupt? Zeigt Version,
   Protokoll- und Detektorstatus. Läuft das nicht, ist alles Weitere sinnlos.
2. I²C-Scan (Abschnitt 3), falls die Adresse unklar ist.
3. **`XM125_Distance`** — ein Sensor, Konfiguration + Kalibrierung + Messung.
4. **Kalibrierskripte in dieser Reihenfolge** — je Methode ein eigener Sketch,
   Übersicht in
   [07-Kalibrierung.md](07-Kalibrierung.md#ein-skript-je-kalibriermethode):
   `XM125_Calib_Module` (kalibriert das Modul sauber durch?) →
   `XM125_Calib_Alignment` (Sensor auf maximale Echostärke ausrichten) →
   `XM125_Calib_Sensitivity` (`RADAR_SIGNAL_QUALITY`,
   `RADAR_THRESHOLD_SENSITIVITY`) → `XM125_Calib_Crosstalk`
   (stören sich die gleichzeitig messenden Sensoren?). Ergebnisse nach
   [params.h](../../include/params.h) übertragen.
5. **`XM125_Distance_Multi`** — alle vier Sensoren, inklusive PDS-Ausgabe.
6. Erst danach die Positionsbibliothek dazunehmen.

Der Rohwert↔Distanz-Teil (Counts → mm) hat kein eigenes Skript, weil der
Umrechnungsfaktor offen ist — dafür weiterhin `XM125_WallCalibration`
(Modi `d`/`f`), siehe [07-Kalibrierung.md Teil B](07-Kalibrierung.md#teil-b--kalibrierung-der-peak-rohwerte-counts--mm).

## 8. Integration: Power Debug System (PDS)

[XM125_Distance_Multi.cpp](../examples/XM125_Distance_Multi.cpp)
schreibt den Rohwert jedes Sensors in einen PDS-Kanal, um ihn live zu
plotten:

```cpp
#include "PDS.h"
PowerDebugger PDS;

void setup() { PDS.init(); }

void loop() {
  // ... isMeasuring() == false, Ergebnis abgeholt ...
  PDS.Channel(150 + i, radar.getPeakDistanceRaw(0));   // Kanäle 150..153
  PDS.update();
}
```

Das ist der schnellste Weg, das Verhalten der Rohwerte über der Zeit zu sehen —
und damit auch das beste Werkzeug für den Sägezahn-Test aus
[07-Kalibrierung.md B.2](07-Kalibrierung.md#b2-schritt-0-erst-das-komplette-32-bit-wort-ansehen).

## 9. Integration: systemlib & params.h

Im fertigen Roboter wird die Library nicht direkt aus `loop()` bedient,
sondern über die `systemlib`. Alle Zahlen dazu stehen zentral in
[lib/include/params.h](../../include/params.h):

| Konstante | Wert | Bedeutung |
|---|---|---|
| `I2C_RADAR` | `Wire` | Bus, an dem die vier Module hängen |
| `RADAR_NUM_SENSORS` | 4 | Anzahl Module |
| `RADAR_SENSOR_ADDRS[]` | `0x51`–`0x54` | Adressen in Ringreihenfolge (Vo, Li, Hi, Re) |
| `RADAR_WAKE_PIN` / `RADAR_INT_PIN` | `-1` | beide Pins unverkabelt |
| `RADAR_START_MM` | 200 | näher liegt der Roboter selbst |
| `RADAR_END_MM` | 2700 | 200 mm Reserve, damit 2500 mm nicht auf der Bereichskante liegt |
| `RADAR_SIGNAL_QUALITY` | 25000 | mehr SNR am Bereichsende (Holzwand) |
| `RADAR_THRESHOLD_SENSITIVITY` | 750 | CFAR lässt schwache Wand-Peaks durch |
| `RADAR_MEASURE_TIMEOUT_MS` | 3000 | PROFILE5 + hohe Signalqualität brauchen länger als der 1000-ms-Default; wird per `setMeasureTimeout()` in die Library gesetzt |
| `RADAR_PEAKS_TO_FETCH` | 1 | wie viele Peaks je Messung wirklich geholt werden — `systemlib` wertet nur Peak 0 aus |
| `RADAR_REINIT_INTERVAL_MS` | 2000 | Mindestabstand zwischen zwei Wiederanlaufversuchen für einen toten Sensor (die Neuinitialisierung blockiert) |

Ablauf in der `systemlib` — **es wird nie auf ein Messergebnis gewartet**:

- `setupRadarSens(idx)` — `begin()` + Konfiguration aus `params.h` +
  `setMeasureTimeout()`/`setMaxPeaksToFetch()` + `applyConfigAndCalibrate()`
  für einen Sensor. Blockiert, läuft deshalb nur beim Start bzw. gedrosselt.
- `readRadar()` — sieht je Zyklus bei jedem laufenden Sensor **einmal** mit
  `isMeasuring()` nach (höchstens ein Registerlesezugriff), übernimmt fertige
  Ergebnisse und startet die nächste Runde, sobald alle durch sind.
- `startRadarRound()` — `startMeasurement()` bei **allen** Sensoren
  hintereinander: sie messen damit gleichzeitig, die Messdauer fällt einmal
  für alle an.
- `takeRadarResult(idx)` — `getLastResult()` in `RadarVo/Li/Hi/Re` übernehmen.
- `radarMeasureFailed(idx)` — zählt Fehlschläge und markiert den Sensor nach
  `RADAR_MAX_CONSEC_FAILURES` als nicht bereit (ohne zu blockieren).
- `retryRadarSetup(idx)` — höchstens alle `RADAR_REINIT_INTERVAL_MS` ein
  blockierender Wiederanlaufversuch für einen als tot markierten Sensor.

Zeitbedarf im Zyklus: rund ein Buszugriff je Sensor (< 1 ms), plus einmal je
Runde das Lesen von Ergebnis- und Peak-Registern.

Die Werte für `RADAR_SIGNAL_QUALITY` und `RADAR_THRESHOLD_SENSITIVITY` stammen
aus [XM125_Calib_Sensitivity.cpp](../examples/XM125_Calib_Sensitivity.cpp);
ob der Gleichzeitigbetrieb Trefferquote kostet, misst
[XM125_Calib_Crosstalk.cpp](../examples/XM125_Calib_Crosstalk.cpp) —
wird die Hardware oder die Zielumgebung geändert (andere Wand, anderes
Gehäuse), gehören sie neu gemessen.

## 10. Integration: Positionsbestimmung

Die Positionsbibliothek erwartet pro Zyklus ein `RadarReading` je Sensor in
Ringreihenfolge (vorne, rechts, hinten, links):

```cpp
Positioning::RadarReading rd[Positioning::RADAR_COUNT];

for (uint8_t i = 0; i < Positioning::RADAR_COUNT; i++) {
  rd[i].valid = radarSensor[i].hasResult();   // nach isMeasuring() == false
  if (!rd[i].valid) continue;

  uint8_t n = radarSensor[i].getNumDistances();
  if (n > Positioning::RADAR_MAX_PEAKS) n = Positioning::RADAR_MAX_PEAKS;
  rd[i].peakCount = n;

  for (uint8_t p = 0; p < n; p++) {
    rd[i].peaks[p].distance_cm = Positioning::rawCountToCm(radarSensor[i].getPeakDistanceRaw(p));
    rd[i].peaks[p].strength    = radarSensor[i].getPeakStrength(p);
  }
}

Positioning::position pos = Positioning::update(heading, camOpp, camOwn, rd);
```

Empfohlene Sensor-Konfiguration für diesen Einsatzfall (Bandenwände messen):

```cpp
radar.config.startMm              = RADAR_START_MM;              // 200
radar.config.endMm                = RADAR_END_MM;                // 2700, Reserve über 2500
radar.config.signalQuality        = RADAR_SIGNAL_QUALITY;        // 25000, Holz reflektiert schwach
radar.config.thresholdSensitivity = RADAR_THRESHOLD_SENSITIVITY; // 750
radar.config.peakSorting          = XM125D_SORT_CLOSEST;         // nächste Wand zuerst
radar.config.reflectorShape       = XM125D_REFLECTOR_PLANAR;     // flache Bande
```

Begründung jedes Werts: [04-Konfiguration.md](04-Konfiguration.md#in-diesem-projekt-verwendete-werte).

> **Solange die Rohwert-Kalibrierung offen ist**, rechnet
> `Positioning::rawCountToCm()` mit einer Annahme (`20.0 cm + raw · 0.902 cm`).
> Sobald die Messreihe aus [07-Kalibrierung.md Teil B](07-Kalibrierung.md#teil-b--kalibrierung-der-peak-rohwerte-counts--mm)
> gefahren ist, müssen **beide** Stellen konsistent nachgezogen werden.

## 11. Checkliste für eine neue Platine

- [ ] 3V3 und GND stabil, gemeinsame Masse mit dem Teensy
- [ ] Pull-ups auf SDA/SCL vorhanden (einmal pro Bus, 2,2–4,7 kΩ)
- [ ] Jedes Modul hat eine eigene, bekannte I²C-Adresse (I²C-Scan)
- [ ] `Wire.begin()` **vor** `radar.begin(Wire)` in der Anwendung
- [ ] `Wire.setClock(100000)` gesetzt
- [ ] `WAKE_UP` bewusst behandelt: unverkabelt (Projektstandard), fest auf 3V3, oder als `wakePin` übergeben
- [ ] `XM125_Connectivity` läuft und zeigt plausible Register
- [ ] Kalibrierung erfolgt in freier Umgebung (siehe [07-Kalibrierung.md A.4](07-Kalibrierung.md#a4-umgebungsbedingungen-während-der-kalibrierung))
