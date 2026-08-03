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

`_executeCommand()` und `measure()` fangen das ab: Meldet der
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

### 3. `CALIBRATION_NEEDED` beim Distance-Detector

Wenn `getCalibrationNeeded()` nach einer Messung `true` liefert, fordert
der Sensor selbst eine Neukalibrierung an (z.B. durch Temperaturdrift).
Empfohlene Reaktion, wie in
[test/XM125_Distance.cpp](../../../test/XM125_Distance.cpp) umgesetzt:

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

if (!radar.measure()) {           // oder !radar.update() bei Presence
  consecutiveFailures++;
  if (consecutiveFailures >= 5) {
    consecutiveFailures = 0;
    setupSensor();               // begin() + config + apply erneut
  }
} else {
  consecutiveFailures = 0;
}
```

### 5. Sensor bei Start nicht erreichbar (`begin()` liefert `false`)

Mögliche Ursachen (siehe [test/XM125_Connectivity.cpp](../../../test/XM125_Connectivity.cpp)):

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
[src/XM125_Distance_Multi.cpp](../../../src/XM125_Distance_Multi.cpp))
muss jedes Modul eine eigene, hardwareseitig eingestellte I²C-Adresse
haben. Die Software ändert die Adresse nicht — sie wird nur beim
Konstruktor-Aufruf angegeben, um das jeweils richtige Modul anzusprechen.
Fällt einzelner Sensor beim Start aus, wird er in der Hauptschleife
periodisch erneut über `setupSensor(idx)` initialisiert, ohne die
anderen Sensoren zu blockieren.

## Nützliche Diagnosefunktion

Aus [test/XM125_Distance.cpp](../../../test/XM125_Distance.cpp), gültig
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
