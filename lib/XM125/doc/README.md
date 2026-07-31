# XM125 Radar-Bibliothek — Dokumentation

Diese Bibliothek steuert das Acconeer **XM125 (A121)** Radarsensor-Modul über I²C.
Das Modul kann wahlweise mit der **Distance-Detector**- oder der
**Presence-Detector**-Firmware geflasht sein — beide teilen sich ein
gemeinsames Basis-Protokoll (16-bit Registeradresse + 32-bit Wert, Big-Endian).

## Inhalt dieses Doku-Ordners

| Datei | Inhalt |
|---|---|
| [01-Protokoll-und-Architektur.md](01-Protokoll-und-Architektur.md) | Wie die Bibliothek aufgebaut ist, I²C-Protokoll, Ablauf einer Messung, Klassenhierarchie |
| [02-Distance-Detector.md](02-Distance-Detector.md) | API, Ablauf und Ergebniswerte des Distance-Detectors (`XM125Distance`) |
| [03-Presence-Detector.md](03-Presence-Detector.md) | API, Ablauf und Ergebniswerte des Presence-Detectors (`XM125Presence`) |
| [04-Konfiguration.md](04-Konfiguration.md) | **Alle Konfigurationsparameter** beider Detektoren mit Wertebereich, Default und Erklärung |
| [05-Fehlerbehandlung-und-Troubleshooting.md](05-Fehlerbehandlung-und-Troubleshooting.md) | Status-Bits, bekannte Probleme (I2C, WAKE_UP, unkalibrierte Peak-Distanz), Debugging-Tipps |
| [06-Beispiele.md](06-Beispiele.md) | Vollständige Codebeispiele: Connectivity-Test, Einzel- und Mehrsensor-Distanzmessung, Presence-Loop |

## Kurzeinstieg

```cpp
#include <Wire.h>
#include "XM125Distance.h"

XM125Distance radar; // Default-Adresse 0x55, kein WAKE_UP/INT-Pin verkabelt

void setup() {
  Wire.begin();
  radar.begin(Wire);

  radar.config.startMm = 200;
  radar.config.endMm   = 2500;
  radar.applyConfigAndCalibrate();
}

void loop() {
  if (radar.measure()) {
    for (uint8_t i = 0; i < radar.getNumDistances(); i++) {
      Serial.println(radar.getPeakDistanceRaw(i)); // s. Hinweis zur Kalibrierung!
    }
  }
}
```

> **Wichtiger Hinweis zur Distanz-Kalibrierung:** Die auf den Sensoren dieses
> Projekts laufende Firmware liefert Peak-Distanzen aktuell nur als
> unkalibrierten 1-Byte-Rohwert (0–255 "Counts"), **nicht** in echten
> Millimetern. Details siehe [04-Konfiguration.md](04-Konfiguration.md) und
> [05-Fehlerbehandlung-und-Troubleshooting.md](05-Fehlerbehandlung-und-Troubleshooting.md).

## Quelle

Registerprotokoll, Kommandos und Default-Werte basieren auf dem Acconeer
*"I2C Distance/Presence Detector User Guide"* (XM125-Datenblatt, Kapitel
Register Interface), ergänzt um projektspezifische Beobachtungen (siehe
Kommentare im Code sowie Kapitel 5 dieser Doku).
