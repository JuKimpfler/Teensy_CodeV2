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
| [06-Beispiele.md](06-Beispiele.md) | Vollständige Codebeispiele: Connectivity-Test, Einzel- und Mehrsensor-Distanzmessung, Presence-Loop, Wand-Kalibrierung |
| [07-Kalibrierung.md](07-Kalibrierung.md) | **Alle Kalibrierverfahren**: was das Modul intern kalibriert (Sensor, Rauschpegel, Offset, Leakage, Recorded Threshold), welches Kommando was tut, Umgebungsbedingungen, `CALIBRATION_NEEDED`-Strategie — plus vollständiges Messprotokoll für die Umrechnung Rohwert → Millimeter |
| [08-API-Referenz.md](08-API-Referenz.md) | Jede Methode einzeln: Signatur, Rückgabe, Blockierdauer, Vorbedingungen, Fehlerfälle |
| [09-Register-und-Konstanten.md](09-Register-und-Konstanten.md) | Vollständige Registerkarte und **jede** Konstante mit Zahlenwert; Spickzettel zum Dekodieren von Statuswerten |
| [10-Interna-und-Timing.md](10-Interna-und-Timing.md) | Byte-Layout auf dem Bus, Clock-Stretching-Workaround, Polling, Zeitbudget, paralleles Messen, Speicherbedarf, bekannte Eigenheiten |
| [11-Hardware-und-Integration.md](11-Hardware-und-Integration.md) | Verkabelung, I²C-Adressen, Bustakt, Teensy-Busse, Sensorring, PlatformIO-Tests, Anbindung an systemlib/params.h, PDS und Positionsbestimmung |
| [12-Glossar.md](12-Glossar.md) | Alle Fachbegriffe erklärt: Sweep, Frame, HWAAS, Profile, CFAR, Intra/Inter, Leakage, 1000×-Einheiten … |

## Wo finde ich was?

| Frage | Kapitel |
|---|---|
| "Wie fange ich an?" | Kurzeinstieg unten, dann [06](06-Beispiele.md) |
| "Was macht dieser Parameter?" | [04](04-Konfiguration.md) |
| "Was macht diese Funktion genau?" | [08](08-API-Referenz.md) |
| "Wie funktioniert die Kalibrierung?" | [07](07-Kalibrierung.md) |
| "Warum sind die Distanzen keine Millimeter?" | [07 Teil B](07-Kalibrierung.md#teil-b--kalibrierung-der-peak-rohwerte-counts--mm) |
| "Das Radar sieht die Wand nicht" | [05, Abschnitt 2b](05-Fehlerbehandlung-und-Troubleshooting.md), dann `XM125_Calib_Alignment` (Ausrichtung) und `XM125_Calib_Sensitivity` Modus `s` |
| "Welches Kalibrierskript wofür?" | [07, Skriptübersicht](07-Kalibrierung.md#ein-skript-je-kalibriermethode) |
| "Was bedeutet Status `0x1100000F`?" | [09](09-Register-und-Konstanten.md) |
| "Warum ist meine Loop so langsam?" | [10](10-Interna-und-Timing.md) |
| "Wie messen mehrere Sensoren gleichzeitig?" | [02](02-Distance-Detector.md#mehrere-sensoren-parallel-messen-lassen), [08](08-API-Referenz.md), [10](10-Interna-und-Timing.md) |
| "Sensor antwortet nicht" | [05](05-Fehlerbehandlung-und-Troubleshooting.md), [11](11-Hardware-und-Integration.md) |
| "Was heißt HWAAS / CFAR / Sweep?" | [12](12-Glossar.md) |

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
  radar.startMeasurement();          // erste Messung anstoßen
}

void loop() {
  if (radar.isMeasuring()) return;   // laeuft noch -- die Library wartet nie

  const XM125DistanceResult& res = radar.getLastResult();
  for (uint8_t i = 0; i < res.numDistances; i++) {
    Serial.println(res.peakDistanceRaw[i]); // s. Hinweis zur Kalibrierung!
  }

  radar.startMeasurement();          // naechste Runde
}
```

> **Wichtiger Hinweis zur Distanz-Kalibrierung:** Die auf den Sensoren dieses
> Projekts laufende Firmware liefert Peak-Distanzen aktuell nur als
> unkalibrierten 1-Byte-Rohwert (0–255 "Counts"), **nicht** in echten
> Millimetern. Vollständiges Messprotokoll dazu:
> [07-Kalibrierung.md, Teil B](07-Kalibrierung.md#teil-b--kalibrierung-der-peak-rohwerte-counts--mm),
> fertiges Werkzeug: `lib/XM125/examples/XM125_WallCalibration.cpp`.

## Quelle & Kennzeichnung

Registerprotokoll, Kommandos und Default-Werte basieren auf dem Acconeer
*"I2C Distance/Presence Detector User Guide"* (XM125-Datenblatt, Kapitel
Register Interface), ergänzt um projektspezifische Beobachtungen (siehe
Kommentare im Code sowie Kapitel 5, 7 und 10 dieser Doku).

In den Kapiteln 7–12 sind Aussagen gekennzeichnet mit **✔ Code** (aus diesem
Repo belegbar), **📘 Acconeer** (aus Datenblatt/User Guide) und **⚠ Annahme**
(unbestätigt, muss gemessen werden).
