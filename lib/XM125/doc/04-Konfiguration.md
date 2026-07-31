# Konfiguration — alle Parameter im Überblick

Diese Datei listet **jeden** Konfigurationsparameter beider Detektoren
(`XM125Distance::config` und `XM125Presence::config`) mit Registeradresse,
Typ, Wertebereich, Default und einer Erklärung, was der Parameter bewirkt.

Konfiguriert wird ausschließlich über das öffentliche `config`-Member der
jeweiligen Klasse, **vor** dem Aufruf von `applyConfigAndCalibrate()` /
`applyConfiguration()`:

```cpp
radar.config.startMm = 200;
radar.config.endMm   = 2500;
radar.applyConfigAndCalibrate(); // schreibt alle config-Felder + kalibriert
```

`writeConfig()` (intern von `applyConfig*()` aufgerufen) schreibt **alle**
Felder aus `config` nacheinander in die zugehörigen Register, mit 1 ms
Pause zwischen den einzelnen Schreibvorgängen. Schlägt ein einzelner
Register-Write fehl, bricht `writeConfig()` sofort ab (fail-fast) und
`getLastFailedConfigRegister()` liefert die betroffene Registeradresse.

---

## 1. Distance-Detector — `XM125DistanceConfig`

Struct-Definition: [XM125Distance.h](../XM125Distance.h) (`XM125DistanceConfig`, Zeile ~95)

| `config`-Feld | Register | Typ | Default | Bedeutung |
|---|---|---|---|---|
| `startMm` | `0x0040` | uint32 (mm) | **250** | Untere Grenze des Messbereichs. Objekte näher am Sensor werden nicht erkannt. In diesem Projekt üblich: 200 mm |
| `endMm` | `0x0041` | uint32 (mm) | **3000** | Obere Grenze des Messbereichs. Größerer Bereich = längere Messzeit. In diesem Projekt üblich: 2500–4000 mm |
| `maxStepLength` | `0x0042` | uint32 | **0** | Maximale Schrittweite zwischen Messpunkten. `0` = automatisch von der Firmware bestimmt (empfohlen) |
| `closeRangeLeakageCancellation` | `0x0043` | bool | **true** | Kompensiert Signal-"Leakage" (Übersprechen Sender→Empfänger) im Nahbereich. Verbessert Messungen nahe `startMm`; sollte i.d.R. aktiviert bleiben |
| `signalQuality` | `0x0044` | uint32 (1000×-Einheit) | **15000** | Gewünschte Signalqualität. Höhere Werte verbessern SNR auf Kosten der Messzeit/Energieverbrauch |
| `maxProfile` | `0x0045` | Enum `PROFILE1`–`PROFILE5` | **PROFILE5** | Radar-Profil, steuert Trade-off zwischen Reichweite/Auflösung und Energieverbrauch. Höhere Profile = größere Reichweite, aber längere Pulse (schlechtere Nahbereichsauflösung). PROFILE5 = maximale Reichweite |
| `thresholdMethod` | `0x0046` | Enum, s.u. | **CFAR** | Verfahren zur Schwellwertberechnung, mit dem echte Reflexionen vom Rauschen unterschieden werden (siehe Tabelle unten) |
| `peakSorting` | `0x0047` | Enum `CLOSEST` / `STRONGEST` | **STRONGEST** | Sortierreihenfolge der zurückgegebenen Peaks: nach Nähe oder nach Signalstärke |
| `numFramesRecordedThreshold` | `0x0048` | uint32 | **100** | Nur relevant bei `thresholdMethod = RECORDED`: Anzahl der Frames, die zur Aufzeichnung des Rausch-/Referenzprofils verwendet werden |
| `fixedAmplitudeThreshold` | `0x0049` | uint32 (1000×-Einheit) | **100000** | Nur relevant bei `thresholdMethod = FIXED_AMPLITUDE`: fester Amplituden-Schwellwert |
| `thresholdSensitivity` | `0x004A` | uint32, 0–1000 (1000×-Einheit) | **500** | Empfindlichkeit des Schwellwertverfahrens (v.a. bei CFAR relevant). Höhere Werte = empfindlicher, aber mehr Fehlalarme |
| `reflectorShape` | `0x004B` | Enum `GENERIC` / `PLANAR` | **GENERIC** | Erwartete Reflektorform. `PLANAR` für große flache Flächen (z.B. Wände), `GENERIC` für beliebige/unbekannte Objekte |
| `fixedStrengthThreshold` | `0x004C` | uint32 (1000×-Einheit) | **0** | Nur relevant bei `thresholdMethod = FIXED_STRENGTH`: fester Signalstärke-Schwellwert |
| `measureOnWakeup` | `0x0080` | bool | **false** | Wenn `true`, misst das Modul automatisch direkt nach dem Aufwachen (nur relevant mit verkabeltem `WAKE_UP`-Pin) |

### `thresholdMethod` — Werte im Detail

| Enum | Wert | Verwendet zusätzlich Feld | Beschreibung |
|---|---|---|---|
| `XM125D_THRESHOLD_FIXED_AMPLITUDE` | 1 | `fixedAmplitudeThreshold` | Fester Amplituden-Schwellwert, einfach aber unflexibel bei wechselnden Umgebungsbedingungen |
| `XM125D_THRESHOLD_RECORDED` | 2 | `numFramesRecordedThreshold` | Schwellwert wird aus aufgezeichneten Referenz-Frames abgeleitet (gut bei bekannter, stabiler Umgebung ohne Zielobjekt) |
| `XM125D_THRESHOLD_CFAR` | 3 | `thresholdSensitivity` | *Constant False Alarm Rate* — adaptiver, umgebungsunabhängiger Schwellwert (Standard/Default, für die meisten Anwendungsfälle empfohlen) |
| `XM125D_THRESHOLD_FIXED_STRENGTH` | 4 | `fixedStrengthThreshold` | Fester Schwellwert auf Basis der (kalibrierten) Signalstärke statt Rohamplitude |

### `maxProfile` — Werte im Detail

| Enum | Wert |
|---|---|
| `XM125D_PROFILE1` | 1 (kürzeste Pulse, beste Nahbereichsauflösung, geringste Reichweite) |
| `XM125D_PROFILE2` | 2 |
| `XM125D_PROFILE3` | 3 |
| `XM125D_PROFILE4` | 4 |
| `XM125D_PROFILE5` | 5 (längste Pulse, größte Reichweite, Default in diesem Projekt) |

### In diesem Projekt verwendete Werte

Aus den Testskripten ([test/XM125_Distance.cpp](../../../test/XM125_Distance.cpp),
[test/XM125_Distance_Multi.cpp](../../../test/XM125_Distance_Multi.cpp)):

```cpp
radar.config.startMm         = 200;
radar.config.endMm           = 2500;   // teils auch 4000
radar.config.maxProfile      = XM125D_PROFILE5;
radar.config.thresholdMethod = XM125D_THRESHOLD_CFAR;
radar.config.peakSorting     = XM125D_SORT_STRONGEST;
```

Alle übrigen Felder bleiben auf ihren Defaults.

---

## 2. Presence-Detector — `XM125PresenceConfig`

Struct-Definition: [XM125Presence.h](../XM125Presence.h) (`XM125PresenceConfig`, Zeile ~89)

| `config`-Feld | Register | Typ | Default | Bedeutung |
|---|---|---|---|---|
| `sweepsPerFrame` | `0x0040` | uint32 | **16** | Anzahl der Radar-Sweeps, die pro Frame gemittelt werden. Mehr Sweeps = besseres SNR, aber langsamere Framerate |
| `interFramePresenceTimeoutS` | `0x0041` | uint32 (s) | **3** | Zeit, für die `isPresenceDetectedSticky()` nach der letzten Erkennung noch `true` bleibt |
| `interPhaseBoostEnabled` | `0x0042` | bool | **false** | Erhöht die Empfindlichkeit der Inter-Frame-Erkennung (langsame Bewegung) kurzfristig, sobald eine mögliche Bewegung vermutet wird |
| `intraDetectionEnabled` | `0x0043` | bool | **true** | Aktiviert die Intra-Frame-Erkennung (schnelle Bewegung, z.B. Gestik, Atmung im Nahbereich) |
| `interDetectionEnabled` | `0x0044` | bool | **true** | Aktiviert die Inter-Frame-Erkennung (langsame Bewegung, z.B. eine Person, die sich langsam bewegt oder ruhig sitzt) |
| `frameRateMilliHz` | `0x0045` | uint32 (mHz) | **12000** (= 12 Hz) | Rate, mit der neue Frames aufgenommen werden |
| `intraDetectionThreshold` | `0x0046` | uint32 (1000×-Einheit) | **1300** | Schwellwert für die Intra-Frame-Erkennung. Höher = weniger empfindlich (weniger Fehlalarme, aber auch weniger sensible Erkennung) |
| `interDetectionThreshold` | `0x0047` | uint32 (1000×-Einheit) | **1000** | Schwellwert für die Inter-Frame-Erkennung, analog zu `intraDetectionThreshold` |
| `interFrameDeviationTimeConstMs` | `0x0048` | uint32 (ms) | **500** | Zeitkonstante des Filters, der die Abweichung zwischen Frames für die Inter-Erkennung glättet |
| `interFrameFastCutoffMilliHz` | `0x0049` | uint32 (mHz) | **6000** | Obere Grenzfrequenz des Bandpassfilters für die Inter-Frame-Erkennung (schnellerer Anteil langsamer Bewegung) |
| `interFrameSlowCutoffMilliHz` | `0x004A` | uint32 (mHz) | **200** | Untere Grenzfrequenz des Bandpassfilters für die Inter-Frame-Erkennung |
| `intraFrameTimeConstMs` | `0x004B` | uint32 (ms) | **150** | Zeitkonstante des internen Filters für die Intra-Frame-Erkennung |
| `intraOutputTimeConstMs` | `0x004C` | uint32 (ms) | **300** | Zeitkonstante zur Glättung des Intra-Presence-Scores am Ausgang (`getIntraPresenceScore()`) |
| `interOutputTimeConstMs` | `0x004D` | uint32 (ms) | **2000** | Zeitkonstante zur Glättung des Inter-Presence-Scores am Ausgang (`getInterPresenceScore()`) |
| `autoProfileEnabled` | `0x004E` | bool | **true** | Lässt die Firmware das Radar-Profil automatisch anhand von `startMm`/`endMm` wählen. Wenn `false`, wird `manualProfile` verwendet |
| `autoStepLengthEnabled` | `0x004F` | bool | **true** | Lässt die Firmware die Schrittweite zwischen Messpunkten automatisch wählen. Wenn `false`, wird `manualStepLength` verwendet |
| `manualProfile` | `0x0050` | Enum `PROFILE1`–`PROFILE5` | **PROFILE4** | Nur relevant wenn `autoProfileEnabled = false`: manuell gewähltes Radar-Profil |
| `manualStepLength` | `0x0051` | uint32 | **72** | Nur relevant wenn `autoStepLengthEnabled = false`: manuell gewählte Schrittweite zwischen Messpunkten |
| `startMm` | `0x0052` | uint32 (mm) | **300** | Untere Grenze des Erfassungsbereichs |
| `endMm` | `0x0053` | uint32 (mm) | **2500** | Obere Grenze des Erfassungsbereichs. In diesem Projekt üblich: 2000 mm |
| `resetFiltersOnPrepare` | `0x0054` | bool | **true** | Setzt interne Filterzustände beim Vorbereiten/(Neu-)Start zurück, statt alte Zustände weiterzuverwenden |
| `hwaas` | `0x0055` | uint32 | **32** | *Hardware Accelerated Average Samples* — Anzahl der pro Messpunkt Hardware-seitig gemittelten Samples. Höher = besseres SNR, aber höherer Energieverbrauch/Messzeit |
| `detectionOnGpio` | `0x0080` | bool | **false** | Wenn `true`, gibt das Modul den Anwesenheits-Status zusätzlich direkt über einen GPIO-Pin aus (Hardware-Verkabelung nötig, unabhängig vom I²C-Register) |

### In diesem Projekt verwendete Werte

Aus [test/XM125_Presence.cpp](../../../test/XM125_Presence.cpp):

```cpp
radar.config.startMm = 300;
radar.config.endMm   = 2000;
```

Alle übrigen Felder bleiben auf ihren Defaults (16 Sweeps/Frame, 12 Hz
Framerate, Auto-Profil, Auto-Schrittweite, beide Erkennungsarten aktiv).

---

## 3. Allgemeine Hinweise zur Konfiguration

- **Reihenfolge:** `config` wird komplett gesetzt, *bevor* `applyConfig*()`
  aufgerufen wird — einzelne Felder nachträglich zu ändern, ohne erneut
  `applyConfig*()` aufzurufen, hat **keine** Wirkung auf den Sensor.
- **Distance-Detector:** Nach jeder Änderung an sicherheitsrelevanten
  Feldern (insbesondere `startMm`/`endMm`/`maxProfile`) sollte
  `applyConfigAndCalibrate()` (nicht nur `applyConfiguration()`) verwendet
  werden, damit die Kalibrierung zum neuen Messbereich passt.
- **Presence-Detector:** Laut Header-Kommentar in
  [XM125Presence.h](../XM125Presence.h) ist `config` "vor
  `applyConfiguration()` anzupassen (danach nicht mehr änderbar!)" — d.h.
  Änderungen zur Laufzeit erfordern immer einen erneuten
  `stop()`/`applyConfiguration()`/`start()`-Zyklus.
- **1000×-Einheiten:** Mehrere Schwellwert-Felder (`signalQuality`,
  `fixedAmplitudeThreshold`, `thresholdSensitivity`,
  `fixedStrengthThreshold`, `intraDetectionThreshold`,
  `interDetectionThreshold`) sind laut Acconeer-Datenblatt in einer
  1000-fachen Einheit angegeben (z.B. entspricht `1000` intern dem
  Wert `1.0`).
- **Fehlerdiagnose beim Schreiben:** Schlägt `writeConfig()` fehl, gibt
  `getLastFailedConfigRegister()` die genaue Registeradresse zurück, bei
  der der I²C-Schreibvorgang fehlgeschlagen ist — nützlich, um einen
  I²C-Verkabelungsfehler von einem Firmware-/Registerproblem zu
  unterscheiden (siehe [05-Fehlerbehandlung-und-Troubleshooting.md](05-Fehlerbehandlung-und-Troubleshooting.md)).
