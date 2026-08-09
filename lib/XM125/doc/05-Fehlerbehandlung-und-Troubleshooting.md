# Fehlerbehandlung & Troubleshooting

## Allgemeine Status-Diagnose

Bei jedem Fehler lohnt sich zuerst ein Blick auf zwei Register (beide über
`XM125Base` verfügbar, also für Distance und Presence identisch nutzbar):

```cpp
uint32_t detStatus   = radar.getDetectorStatus();
uint32_t protoStatus = radar.getProtocolStatus();

bool busy  = XM125Base::statusBusy(detStatus);
bool error = XM125Base::statusError(detStatus);
```

- **`protoStatus != 0`**: Protokollfehler auf Register-/Kommandoebene —
  starkes Indiz, dass die geflashte Firmware nicht (nur) dem erwarteten
  Registerprotokoll (Distance *oder* Presence) folgt, das diese Klasse
  annimmt. Prüfen, ob die richtige Klasse (`XM125Distance` vs.
  `XM125Presence`) zur geflashten Firmware passt.
- **`error` (Bit 28, `DETECTOR_ERROR`)**: Allgemeiner Detektorfehler,
  meist behebbar durch Neustart/Neukonfiguration.
- **`busy` (Bit 31)**: Ein Kommando läuft noch — sollte nach kurzer Zeit
  von selbst auf `false` gehen; bleibt es dauerhaft `true`, deutet das auf
  einen hängenden Sensor hin (Power-Cycle nötig).

## Bekannte Fehlerquellen in diesem Projekt

### 1. I²C-Schreibfehler durch Clock-Stretching

Die interne STM32-MCU des XM125 kann den I²C-Takt strecken
("Clock-Stretching"), sobald sie unmittelbar nach Erhalt eines Kommandos
mit der Verarbeitung beginnt. Manche Host-I²C-Treiber interpretieren das
als Schreibfehler/Timeout auf der aktuellen Transaktion, **obwohl das
Kommando beim Sensor angekommen ist**.

`_executeCommand()` und `startMeasurement()` fangen das ab: Meldet der
Schreibvorgang einen Fehler, wird sofort der `DETECTOR_STATUS` gegengeprüft
— steht `BUSY` bereits, wurde das Kommando trotz gemeldetem Schreibfehler
offensichtlich angenommen, und die Funktion fährt normal fort statt einen
Fehler zurückzugeben.

**Praktische Konsequenz:** `Wire.setClock(100000)` (Standard-100kHz statt
400kHz) wird in allen Testskripten verwendet, um dem Modul mehr Toleranz
beim Clock-Stretching zu geben:

```cpp
Wire.begin();
Wire.setClock(100000); // niedriger Takt, mehr Toleranz für Clock-Stretching des XM125
```

### 2. Unkalibrierte Peak-Distanz (Distance-Detector)

Siehe ausführlich [02-Distance-Detector.md](02-Distance-Detector.md#️-unkalibrierte-peak-distanz).
Kurzfassung: `getPeakDistanceMm()` liefert aktuell **keine echten
Millimeter**, sondern denselben unkalibrierten 0–255-Rohwert wie
`getPeakDistanceRaw()`. Nicht als Bug in der Anwendung fehlinterpretieren
— die Kalibrierung des Umrechnungsfaktors ist ein offener Punkt.

Zum Aufklären ist `getPeakDistanceReg()` da: es liefert das vollständige
32-bit-Registerwort, während `getPeakDistanceRaw()` 24 dieser Bits verwirft.
Ob die Distanz überhaupt im obersten Byte steckt, ist genau die offene Frage
— deshalb bei Messreihen immer das Registerwort protokollieren. Fertiges
Werkzeug dafür: [XM125_WallCalibration.cpp](../examples/XM125_WallCalibration.cpp)
(Modus `d <cm>` / `f`).

### 2b. Radar „reagiert nicht" auf eine Wand

Erkennung (Peak vorhanden) und Distanzwert sind zwei getrennte Probleme —
zuerst prüfen, ob überhaupt ein Peak kommt (`getNumDistances() > 0`), und erst
danach über den Zahlenwert reden. Häufigste Ursachen, wenn gar kein Peak
kommt, in dieser Reihenfolge:

1. **Schräg getroffen.** Schon 10–15° Schräglage lenken die Reflexion an einer
   glatten Fläche vom Sensor weg. Senkrecht ausrichten.
2. **Ziel auf der Bereichskante.** Ein Objekt bei exakt `endMm` wird
   unzuverlässig erkannt — `endMm` mit Reserve über dem Zielabstand wählen
   (im Projekt: 2700 mm für 2500 mm Ziel).
3. **Schwelle zu hoch für einen schwachen Reflektor.** Holz reflektiert
   deutlich schwächer als Metall: `signalQuality` und `thresholdSensitivity`
   anheben, `reflectorShape = PLANAR` setzen. Siehe
   [04-Konfiguration.md](04-Konfiguration.md#in-diesem-projekt-verwendete-werte).
4. **Etwas anderes steht näher.** Bei `peakSorting = CLOSEST` ist Peak 0 das
   nächstgelegene Objekt, nicht die Wand — den Bereich vor dem Sensor
   freiräumen (Öffnungswinkel der integrierten Antenne lt. XM125-Datenblatt).

### 3. `CALIBRATION_NEEDED` beim Distance-Detector

Wenn `getCalibrationNeeded()` nach einer Messung `true` liefert, fordert
der Sensor selbst eine Neukalibrierung an (z.B. durch Temperaturdrift).
Empfohlene Reaktion, wie in
[XM125_Distance.cpp](../examples/XM125_Distance.cpp) umgesetzt:

```cpp
if (radar.getCalibrationNeeded()) {
  radar.recalibrate();
}
```

### 4. Wiederholte Messfehler / Sensor scheint "verschwunden"

Alle Testskripte zählen aufeinanderfolgende Fehlschläge und stoßen nach
`MAX_CONSECUTIVE_FAILURES` (typ. 5) eine komplette Neuverbindung an
(`begin()` + erneute Konfiguration). Empfohlenes Muster:

```cpp
uint8_t consecutiveFailures = 0;

if (!radar.isMeasuring()) {        // Runde fertig oder abgebrochen
  if (radar.hasResult()) {         // (bei Presence: !radar.update())
    consecutiveFailures = 0;
    auswerten(radar.getLastResult());
  } else if (++consecutiveFailures >= 5) {
    consecutiveFailures = 0;
    setupSensor();                 // begin() + config + apply erneut
  }
  radar.startMeasurement();        // nächste Runde
}
```

> `setupSensor()` blockiert (Kalibrierung bis ~2 s). Im Roboter wird der
> Wiederanlauf deshalb gedrosselt: `systemlib::retryRadarSetup()` versucht es
> höchstens alle `RADAR_REINIT_INTERVAL_MS`, sonst würde ein abgezogener
> Sensor jeden Zyklus ausbremsen.

### 5. Sensor bei Start nicht erreichbar (`begin()` liefert `false`)

Mögliche Ursachen (siehe [XM125_Connectivity.cpp](../examples/XM125_Connectivity.cpp)):

- SDA/SCL/GND/3V3 falsch verkabelt oder Pull-up-Widerstände fehlen.
- Modul befindet sich in einem Sleep-Zustand (nur relevant, falls
  `WAKE_UP` verkabelt ist und aktiv auf Low gezogen wurde).
- **Kein Bug, sondern bewusstes Projekt-Setup:** In diesem Projekt bleibt
  der `WAKE_UP`-Pin bewusst unverkabelt/floatend (kein Pull-up). Da nie
  ein Sleep-Kommando gesendet wird, bleibt das Modul davon unabhängig
  wach — ein floatender Pin ist daher **nicht** die erwartete
  Fehlerursache, bleibt aber ein theoretisches Restrisiko für
  Störeinkopplungen (EMI), siehe Kommentar in [XM125.h](../XM125.h).

Verbindungsaufbau mit mehreren Versuchen:

```cpp
for (uint8_t attempt = 1; attempt <= 5; attempt++) {
  if (radar.begin(Wire)) break;
  delay(500);
}
```

### 6. Widersprüchliche Angabe der Default-I²C-Adresse in Kommentaren

Im Konstruktor-Kommentar von `XM125Base` ([XM125.h](../XM125.h)) steht
*"Werkseinstellung: 0x52"*, während die tatsächlich verwendete Konstante
`XM125_I2C_ADDR_DEFAULT` auf **`0x55`** gesetzt ist. Maßgeblich für das
tatsächliche Verhalten des Codes ist die Konstante (`0x55`), nicht der
Kommentartext — bei Unklarheiten über die reale Hardware-Adresse ggf. per
I²C-Scanner verifizieren.

### 7. Mehrere Sensoren am selben Bus

Beim Betrieb mehrerer Module an einem `Wire`-Bus (siehe
[XM125_Distance_Multi.cpp](../examples/XM125_Distance_Multi.cpp))
muss jedes Modul eine eigene, hardwareseitig eingestellte I²C-Adresse
haben. Die Software ändert die Adresse nicht — sie wird nur beim
Konstruktor-Aufruf angegeben, um das jeweils richtige Modul anzusprechen.
Fällt einzelner Sensor beim Start aus, wird er in der Hauptschleife
periodisch erneut über `setupSensor(idx)` initialisiert, ohne die
anderen Sensoren zu blockieren.

### 8. Übersprechen zwischen mehreren Sensoren

Im Roboter messen **alle** Module gleichzeitig (`systemlib::readRadar()` stößt
eine Runde bei allen Sensoren zusammen an). Sie senden dabei unkoordiniert im
selben 60-GHz-Band; eine Synchronisation zwischen den Modulen gibt es nicht —
die I²C-Firmware kennt kein Sync-Signal.

**Wie sich Übersprechen äußert:** Ein fremder Sender ist nicht phasenstarr zum
eigenen Empfänger. Er wird deshalb typischerweise *nicht* als sauberer Peak
integriert, sondern hebt das Rauschniveau an. Genau das ist die unangenehme
Wirkung: die CFAR-Schwelle passt sich dem Rauschen an, steigt mit und frisst
die Empfindlichkeitsreserve, die für den schwachen Wand-Reflex bei 2,5 m
gebraucht wird. Das Symptom ist also selten ein Geisterobjekt, sondern
**sporadisch fehlende Peaks am Bereichsende** — dasselbe Bild wie bei zu
niedrig eingestellter Empfindlichkeit.

**Was dagegen hilft — in dieser Reihenfolge:**

1. **Geometrie.** Die Sensoren des Roboters blicken um je 90° versetzt.
   Direktes Einkoppeln von einem in den anderen liegt damit weitgehend
   außerhalb der Antennenkeulen — der Grund, warum der Gleichzeitigbetrieb
   überhaupt tragbar ist.
2. **Reflexionen bedenken.** Eine glatte Wand wirkt für 60 GHz wie ein
   Spiegel — genau deshalb steht `reflectorShape` auf `PLANAR`. In einer Ecke
   kann sie Energie vom Front- auf den Seitensensor umlenken, auch wenn die
   beiden sich nicht direkt „sehen". Das ist der Fall, der in der Praxis am
   ehesten stört.
3. **Empfindlichkeit nachziehen.** Kostet das Übersprechen messbar
   Trefferquote, ist der nächste Hebel `signalQuality` /
   `thresholdSensitivity`
   ([XM125_Calib_Sensitivity.cpp](../examples/XM125_Calib_Sensitivity.cpp)) —
   nicht das Auseinanderziehen der Messungen: Sensoren nacheinander messen zu
   lassen würde die Rundendauer vervielfachen.

**Nicht raten, messen:** Der Sketch
[XM125_Calib_Crosstalk.cpp](../examples/XM125_Calib_Crosstalk.cpp)
hat dafür Modus `i`. Er misst denselben Sensor zweimal — einmal allein, einmal
während alle anderen gleichzeitig senden — und vergleicht Trefferquote und
Signalstärke:

```
  allein:          20/20 erkannt, Staerke 1240, 46 ms
  mit Nachbarn:    18/20 erkannt, Staerke 1180, 46 ms
```

(Beispielausgabe, nicht gemessen.) Aus der Differenz sagt der Sketch, ob das
Übersprechen ins Gewicht fällt. Der Test gehört an den *ungünstigsten* Punkt:
Wand auf 2,5 m, weil dort die Reserve am kleinsten ist.

Zwei weitere Modi desselben Sketches grenzen den Befund ein: `p` misst jeden
Nachbarn **einzeln** als Störer (welches Paar verträgt sich nicht?), und `g`
fährt komplette Messrunden einmal nacheinander und einmal gleichzeitig wie
`systemlib::readRadar()` — dort steht die verlorene Trefferquote direkt neben
der gewonnenen Rundendauer, was die Entscheidung erst abwägbar macht.

## Nützliche Diagnosefunktion

Aus [XM125_Distance.cpp](../examples/XM125_Distance.cpp), gültig
sinngemäß auch für Presence:

```cpp
void printDiagnostics() {
  uint32_t detStatus = radar.getDetectorStatus();
  uint32_t protoStatus = radar.getProtocolStatus();
  Serial.print("Detector Status: 0x"); Serial.println(detStatus, HEX);
  Serial.print("  BUSY="); Serial.print(XM125Base::statusBusy(detStatus));
  Serial.print(" ERROR="); Serial.println(XM125Base::statusError(detStatus));
  Serial.print("Protocol Status: 0x"); Serial.println(protoStatus, HEX);
}
```
