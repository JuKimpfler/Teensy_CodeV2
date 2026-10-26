# Kalibrierung — alle Verfahren im Detail

Dieses Kapitel beschreibt **beide** Dinge, die im Zusammenhang mit dem XM125
"Kalibrierung" heißen und leicht verwechselt werden:

| | Was | Wo | Status |
|---|---|---|---|
| **Teil A** | Modul-interne Kalibrierung (Sensor + Detektor) — Voraussetzung dafür, dass das Modul überhaupt messen kann | läuft **im Modul**, ausgelöst über Kommandos (`applyConfigAndCalibrate()`, `calibrate()`, `recalibrate()`) | funktioniert, wird von dieser Library vollständig bedient |
| **Teil B** | Umrechnung des gelieferten Peak-**Rohwerts** (0–255 Counts) in echte Millimeter | passiert **auf dem Teensy**, außerhalb des Moduls | **offen** — Umrechnungsfaktor unbekannt, Messreihe nötig (Anleitung s.u.) |

Ein dritter Vorgang heißt zwar nicht "Kalibrierung", gehört aber in dieselbe
Inbetriebnahme: die **Abstimmung der Empfindlichkeit** (`signalQuality`,
`thresholdSensitivity`, `reflectorShape`) darauf, dass die Holzwand bei 2,5 m
zuverlässig als Peak erkannt wird, plus die Frage, ob sich die gleichzeitig
messenden Sensoren gegenseitig stören. Die Ergebnisse und
Begründungen stehen in
[04-Konfiguration.md](04-Konfiguration.md#in-diesem-projekt-verwendete-werte)
und [05, Abschnitt 8](05-Fehlerbehandlung-und-Troubleshooting.md#8-übersprechen-zwischen-mehreren-sensoren).

**Reihenfolge bei der Inbetriebnahme:** erst Teil A (sonst misst das Modul
gar nicht), dann die Empfindlichkeitsabstimmung (sonst kommt bei 2,5 m kein
Peak, den man kalibrieren könnte), erst danach Teil B.

## Ein Skript je Kalibriermethode

Jede Methode hat einen eigenen Sketch in [test/](../../../test/). Sie bauen
aufeinander auf — in dieser Reihenfolge durchlaufen, denn jeder Schritt setzt
voraus, dass der vorherige stimmt:

| # | Skript | Methode | Ergebnis geht nach |
|---|---|---|---|
| 1 | [XM125_Calib_Module.cpp](../examples/XM125_Calib_Module.cpp) | Modul-interne Kalibrierung: läuft sie durch, wie lange dauert sie, hält sie im Warmbetrieb? (Teil A) | — (Diagnose) |
| 2 | [XM125_Calib_Alignment.cpp](../examples/XM125_Calib_Alignment.cpp) | Mechanische Ausrichtung: Sensor auf maximale Echostärke drehen | Montage |
| 3 | [XM125_Calib_Sensitivity.cpp](../examples/XM125_Calib_Sensitivity.cpp) | Empfindlichkeit: `signalQuality` × `thresholdSensitivity`, dazu `maxProfile` und `reflectorShape` | `RADAR_SIGNAL_QUALITY`, `RADAR_THRESHOLD_SENSITIVITY` in [params.h](../../include/params.h) |
| 4 | [XM125_Calib_Crosstalk.cpp](../examples/XM125_Calib_Crosstalk.cpp) | Übersprechen: stören sich die gleichzeitig messenden Sensoren? | Montage / Empfindlichkeit (kein Parameter) |

Alle vier konfigurieren mit den Werten aus `params.h` und kalibrieren wie
`systemlib::setupRadarSens()` — was hier gemessen wird, gilt damit auch für
den Roboter. Bedienung jeweils über den seriellen Monitor, `?` zeigt die Hilfe.

Ausführen: gewünschte Datei nach `src/main.cpp` des Hauptprojekts kopieren
(in `src/` darf immer nur eine Datei mit `setup()`/`loop()` liegen) und normal
bauen/hochladen:

```bash
cp lib/XM125/examples/XM125_Calib_Sensitivity.cpp src/main.cpp
pio run -e teensy40 -t upload
```

Ausführliche Beschreibung jedes Skripts, aller Modi und der Bedienung:
[examples/README.md](../examples/README.md).

Teil B (Rohwert → mm) hat bewusst **kein** eigenes Skript: dafür ist der
Umrechnungsfaktor noch offen. Wer die Messreihe fahren will, nutzt weiterhin
[XM125_WallCalibration.cpp](../examples/XM125_WallCalibration.cpp) (Modi
`d`/`f`/`c`, siehe [B.10](#b10-werkzeug-libxm125examplesxm125_wallcalibrationcpp)).

Teil A ist beim Distance-Detektor der Grund, warum `applyConfigAndCalibrate()`
mehrere Sekunden blockieren darf. Teil B ist der Grund, warum
`getPeakDistanceMm()` aktuell **keine** Millimeter liefert.

### Kennzeichnung der Quellen

Da nicht alles aus dem Code dieses Repos beweisbar ist, sind Aussagen unten
markiert:

- **✔ Code** — direkt aus dieser Library / den Testskripten belegbar.
- **📘 Acconeer** — aus dem Acconeer-Datenblatt bzw. *"I2C Distance/Presence
  Detector User Guide"*; nicht am Repo verifizierbar.
- **⚠ Annahme** — Vermutung, muss durch Messung bestätigt werden.

---

# Teil A — Modul-interne Kalibrierung

## A.1 Was das Modul intern kalibriert

Die "Kalibrierung" des Distance-Detektors ist kein einzelner Vorgang, sondern
eine Kette von Teilschritten, die die Firmware nacheinander ausführt
(**📘 Acconeer**):

| # | Teilschritt | Zweck | Abhängig von |
|---|---|---|---|
| 1 | **Sensor-Kalibrierung** (A121-Hardware) | Kalibriert die Analog-/Digitalstufen des Radarchips selbst (u.a. interne Verzögerungsketten, Verstärkung). **Zwingend nach jedem Power-On**, bevor irgendeine Messung gültig ist. | Temperatur, Betriebsspannung |
| 2 | **Rauschpegel-Schätzung** (noise level estimation) | Misst den Rauschboden, indem Frames **ohne** Sendesignal aufgenommen werden. Bildet die Basis für alle Schwellwertverfahren, insbesondere CFAR. | `maxProfile`, `signalQuality`, HWAAS-Einstellung |
| 3 | **Offset-Kompensation** (loopback measurement) | Misst intern die Signallaufzeit, um Sample-Index → Distanz korrekt zuzuordnen. Dieser Offset **driftet mit der Temperatur** — Hauptgrund für spätere Neukalibrierung. | Temperatur |
| 4 | **Close-Range-Leakage-Kalibrierung** | Nimmt die Signatur des direkten Übersprechens Sender→Empfänger auf, damit sie von jeder Messung abgezogen werden kann. Nur aktiv, wenn `config.closeRangeLeakageCancellation = true` (Default). | `startMm`, `maxProfile` |
| 5 | **Recorded Threshold** | Nur bei `thresholdMethod = XM125D_THRESHOLD_RECORDED`: nimmt `numFramesRecordedThreshold` Frames der **leeren** Szene auf und leitet daraus die Schwellwertkurve ab. | `numFramesRecordedThreshold`, gesamte Umgebung |

Wichtig an dieser Aufstellung: Die Schritte 2, 4 und 5 nehmen die **aktuelle
Umgebung** als Referenz auf. Steht während der Kalibrierung etwas vor dem
Sensor (Hand, Person, Wand im Nahbereich), wird dieses Objekt Teil der
Referenz und danach *weggerechnet* — das Objekt "verschwindet" bei späteren
Messungen. Siehe [A.4 Umgebungsbedingungen](#a4-umgebungsbedingungen-während-der-kalibrierung).

Beim **Presence**-Detektor entfällt der Detektor-Teil: Dort gibt es nur die
Sensor-Kalibrierung (Schritt 1), die als Teil von `APPLY_CONFIGURATION`
mitläuft — erkennbar am eigenen Status-Bit `XM125P_STATUS_SENSOR_CALIBRATE_OK`
(Bit 3) (**✔ Code**, [XM125Presence.h](../XM125Presence.h)).

## A.2 Welches Kommando macht was

Die Distance-Firmware kennt vier kalibrierungsrelevante Kommandos. Diese
Library bildet sie 1:1 ab (**✔ Code**, [XM125Distance.cpp](../XM125Distance.cpp))
und setzt eine fünfte Methode obendrauf, die den Modul-Reset voranstellt:

| Methode | Kommando (Reg. `0x0100`) | Schreibt `config`? | Sensor-Kal. (1) | Detektor-Kal. (2–5) | Typische Dauer |
|---|---|---|---|---|---|
| `applyConfigAndCalibrate()` | `1` | **ja** (`writeConfig()`) | ja | ja | einige 100 ms bis ~2 s |
| `applyConfiguration()` | `3` | **ja** (`writeConfig()`) | nein | nein | kurz (~10–100 ms) |
| `calibrate()` | `4` | nein | ja | ja | wie `applyConfigAndCalibrate()` |
| `recalibrate()` | `5` | nein | ja (schneller Teil) | nur temperaturabhängige Teile | deutlich kürzer als `calibrate()` |
| `hardCalibration()` | `RESET`, dann `1` | **ja** (alle 14 Register) | ja | ja | ~0,5–2,5 s |

Merksätze:

- **Nach jedem Power-On / Reset genau einmal `applyConfigAndCalibrate()`.**
  Ohne Kalibrierung liefert das Modul keine gültigen Messungen.
- **`applyConfiguration()` allein kalibriert nicht.** Es ist nur sinnvoll,
  wenn im Modul bereits eine gültige Kalibrierung liegt und sich nur
  unkritische Konfigurationsfelder geändert haben.
- **`calibrate()` = volle Neukalibrierung** ohne die `config`-Register neu zu
  schreiben (nutzt also die zuletzt angewendete Konfiguration).
- **`recalibrate()` = "kleine" Nachkalibrierung**, gedacht als Reaktion auf
  das Flag `CALIBRATION_NEEDED` (**📘 Acconeer**: erneuert vor allem die
  temperaturabhängigen Anteile, ohne die Umgebungsreferenz komplett neu
  aufzunehmen).
- **`hardCalibration()` = der große Hammer**: erst `XM125_CMD_RESET_MODULE`,
  dann alle 14 Konfigurationsregister neu schreiben, dann Kommando `1`. Für
  den Fall, dass das Modul selbst in einem kaputten Zustand steckt und ein
  `applyConfigAndCalibrate()` deshalb gar nicht mehr greift — siehe A.5
  (der Reset wirft die Kalibrierung *und* die Konfiguration weg, deshalb
  gehört beides zwingend danach). Alle Konfigurationswerte sind als Parameter
  setzbar, ohne Angabe gilt die Werksvorgabe:

  ```cpp
  radar.hardCalibration();                     // alles auf Werksvorgabe
  radar.hardCalibration(150, 2500);            // nur Messbereich anders

  XM125DistanceConfig cfg;                     // oder benannt, empfohlen
  cfg.startMm        = RADAR_START_MM;
  cfg.reflectorShape = XM125D_REFLECTOR_PLANAR;
  radar.hardCalibration(cfg);
  ```

> **Achtung — Reihenfolge:** `applyConfigAndCalibrate()` schreibt *zuerst*
> alle `config`-Register und kalibriert *danach*. Werden `startMm`, `endMm`,
> `maxProfile` oder `closeRangeLeakageCancellation` geändert, muss deshalb
> `applyConfigAndCalibrate()` verwendet werden — ein bloßes
> `applyConfiguration()` würde eine Kalibrierung stehen lassen, die zu einem
> anderen Messfenster gehört.

## A.3 Was im Code tatsächlich passiert

`applyConfigAndCalibrate()` ist eine dünne Hülle (**✔ Code**):

```
applyConfigAndCalibrate(timeoutMs)
 ├── writeConfig()                      // 14 Register, je 1 ms Pause, fail-fast
 │     └── bei Fehler: _lastFailedConfigRegister = reg;  return false
 └── _executeCommand(CMD=1, timeoutMs)
       ├── _writeReg(0x0100, 1)
       │     └── schlägt der Write fehl: DETECTOR_STATUS lesen —
       │         ist BUSY gesetzt, wurde das Kommando trotzdem angenommen
       │         (Clock-Stretching, siehe Kapitel 05) → weitermachen
       ├── delay(2)                     // der internen MCU Zeit geben, BUSY zu setzen
       ├── _waitNotBusy(timeoutMs)      // pollt 0x0003 bis BUSY = 0
       └── return !statusError(status)  // Bit 28 DETECTOR_ERROR
```

Daraus folgen zwei Dinge, die für die Fehlersuche wichtig sind:

1. **`false` von `applyConfigAndCalibrate()` hat drei mögliche Ursachen** —
   (a) I²C-Schreibfehler beim Konfigurieren → `getLastFailedConfigRegister()`
   liefert die Adresse, (b) Timeout (`BUSY` ging nicht rechtzeitig zurück),
   (c) `DETECTOR_ERROR` (Bit 28) nach Abschluss. Fall (a) unterscheidet sich
   von (b)/(c) genau daran, ob `getLastFailedConfigRegister() != 0xFFFF` ist —
   so ist es in [XM125_Distance.cpp](../examples/XM125_Distance.cpp)
   umgesetzt.
2. **Der Rückgabewert prüft nur Bit 28**, nicht die spezifischen
   Kalibrier-Fehlerbits 23/24/25. Wer genau wissen will, *welcher* Teilschritt
   fehlgeschlagen ist, muss `getDetectorStatus()` selbst auswerten — siehe
   [A.8](#a8-fehlerbits-der-kalibrierung).

## A.4 Umgebungsbedingungen während der Kalibrierung

Die Kalibrierung nimmt die Umgebung als Referenz auf. Deshalb gilt für den
Moment, in dem `applyConfigAndCalibrate()` / `calibrate()` läuft:

- ✅ Sensor ist **final montiert** (im Ring, hinter dem endgültigen
  Radom/Gehäuse). Eine Kalibrierung "in der Hand" und späterer Einbau
  verfälscht vor allem die Leakage-Kompensation.
- ✅ **Nichts bewegt sich** vor dem Sensor. Keine Hand, kein Mensch, kein
  vorbeifahrender Roboter.
- ✅ **Nahbereich frei** (mindestens bis `startMm`, besser deutlich weiter),
  solange `closeRangeLeakageCancellation = true` ist.
- ✅ Bei `thresholdMethod = RECORDED`: die **komplette Szene leer** — alles,
  was jetzt dasteht, wird später weggeschwellt.
- ✅ **Temperatur ungefähr im späteren Betriebszustand.** Direkt nach dem
  Einschalten ist die Elektronik kalt; nach ein paar Minuten Betrieb ist sie
  deutlich wärmer. Bei hohen Genauigkeitsansprüchen lieber nach kurzer
  Warmlaufzeit noch einmal `recalibrate()`.
- ✅ **Stabile Versorgung.** Kalibrierung während gleichzeitig die Motoren
  anlaufen ist keine gute Idee.
- ⚠ **Mehrere XM125 gleichzeitig:** Vier Module am selben Ring senden im
  gleichen Frequenzband. Für die Rauschpegel-Schätzung (Schritt 2) ist
  Fremdsignal störend. In
  [XM125_Distance_Multi.cpp](../examples/XM125_Distance_Multi.cpp)
  werden die Sensoren **nacheinander** initialisiert — das ist gut so, weil
  dann immer nur einer kalibriert, während die anderen noch nicht messen.
  Nachträgliches `recalibrate()` einzelner Sensoren im laufenden Betrieb
  passiert dagegen zwangsläufig, während die Nachbarn senden.

## A.5 Kalibrierung ist flüchtig

Die Kalibrierdaten liegen im RAM der Modul-MCU (**📘 Acconeer**). Konsequenzen:

- Nach **jedem** Power-Cycle, nach `XM125_CMD_RESET_MODULE` (also auch nach
  `resetModule()`) und nach einem Brown-Out muss neu kalibriert werden — und
  weil der Reset zusätzlich alle Konfigurationsregister auf Werksvorgabe
  zurücksetzt, muss auch die Konfiguration neu hoch. Genau diese Reihenfolge
  steckt in `hardCalibration()`: Reset → alle Register schreiben →
  kalibrieren.
- Die Library speichert **nichts** persistent — es gibt keine Funktion, eine
  Kalibrierung zu sichern und zurückzuspielen (**✔ Code**).
- Ein Reset des Teensys allein reicht **nicht** aus, damit das Modul seine
  Kalibrierung verliert: Wird der Teensy neu geflasht, während das XM125
  weiterläuft, ist die alte Kalibrierung im Modul noch gültig. Trotzdem ruft
  jeder Neustart der Firmware `applyConfigAndCalibrate()` erneut auf — das ist
  unschädlich und macht das Verhalten reproduzierbar.

## A.6 `CALIBRATION_NEEDED` — wann das Modul selbst nachkalibrieren will

Nach jeder Messung steht in Bit 9 des Ergebnisregisters `0x0010` das Flag
`CALIBRATION_NEEDED`, abrufbar über `getCalibrationNeeded()` (**✔ Code**).

Typische Ursache ist **Temperaturdrift**: Die Offset-Kompensation (A.1
Schritt 3) hängt an der Chiptemperatur; entfernt sich diese zu weit vom Wert
zum Kalibrierzeitpunkt, meldet der Detektor Nachkalibrierbedarf
(**📘 Acconeer**).

Empfohlenes Muster (so in
[XM125_Distance.cpp](../examples/XM125_Distance.cpp)):

```cpp
if (radar.isMeasuring()) return;

if (!radar.hasResult() && radar.getCalibrationNeeded()) {
  radar.recalibrate();       // billiger als calibrate()
}
radar.startMeasurement();
```

Robusteres Muster für den Dauerbetrieb — auch prüfen, wenn die Messung
**erfolgreich** war, und Wiederholungen begrenzen:

```cpp
static uint8_t  recalTries   = 0;
static uint32_t lastRecalMs  = 0;

if (radar.isMeasuring()) return;

if (radar.getCalibrationNeeded() && millis() - lastRecalMs > 2000) {
  lastRecalMs = millis();
  if (radar.recalibrate()) {
    recalTries = 0;
  } else if (++recalTries >= 3) {
    recalTries = 0;
    setupSensor();          // volle Neuinitialisierung: begin() + applyConfigAndCalibrate()
  }
}
```

Warum die Zeitsperre: `recalibrate()` blockiert und liefert in dieser Zeit
keine Messwerte. Ohne Bremse kann ein dauerhaft gesetztes Flag die Regelschleife
lahmlegen.

**Temperatur mitloggen** lohnt sich, um Drift überhaupt zu sehen:

```cpp
Serial.print("T="); Serial.print(radar.getTemperature()); Serial.println(" C");
```

`getTemperature()` liefert °C aus Bit 16–31 des Ergebnisregisters. Laut
Datenblatt ist die **Absolutgenauigkeit gering** — die Werte taugen also für
"hat sich um 15 K erwärmt", nicht für "es sind exakt 34 °C" (**✔ Code**-Kommentar
in [XM125Distance.h](../XM125Distance.h) + **📘 Acconeer**).

## A.7 Dauer & Timeouts

| Konstante | Wert | Verwendet von |
|---|---|---|
| `XM125_DEFAULT_APPLY_TIMEOUT_MS` | **3000 ms** | `applyConfigAndCalibrate()`, `applyConfiguration()`, `calibrate()`, `recalibrate()`, `XM125Presence::start()/stop()` |
| `XM125_DEFAULT_MEASURE_TIMEOUT_MS` | **1000 ms** | Messtimeout der Zustandsmaschine (Default), änderbar über `setMeasureTimeout()`; das Projekt setzt `RADAR_MEASURE_TIMEOUT_MS` = 3000 ms |
| `XM125_WAKEUP_TIMEOUT_MS` | **1500 ms** | `wakeUp()` / `begin()` |

Der Kommentar in [XM125.h](../XM125.h) nennt "~1–2 s" als realistische
Kalibrierdauer — die 3000 ms sind bewusst großzügig. Die tatsächliche Dauer
steigt mit `endMm`, `signalQuality`, `hwaas` und höherem `maxProfile`.

Wenn `applyConfigAndCalibrate()` in einen Timeout läuft, ist das Modul unter
Umständen **noch beschäftigt** — `_waitNotBusy()` bricht ab, das Kommando
läuft im Modul aber weiter. Deshalb: nach einem Timeout nicht sofort das
nächste Kommando hinterherschicken, sondern kurz warten und
`getDetectorStatus()` pollen, bis `BUSY = 0` ist.

Messen, wie lange es real dauert (Muster aus dem Testskript):

```cpp
uint32_t t0 = millis();
bool ok = radar.applyConfigAndCalibrate();
Serial.print("Kalibrierung: "); Serial.print(millis() - t0); Serial.println(" ms");
```

## A.8 Fehlerbits der Kalibrierung

Nach einem fehlgeschlagenen Kalibrierkommando `getDetectorStatus()` auswerten
(**✔ Code**, [XM125Distance.h](../XM125Distance.h)):

| Bit | Konstante | Bedeutung | Erste Maßnahme |
|---|---|---|---|
| 0 | `XM125D_STATUS_RSS_REGISTER_OK` | RSS-Registerzugriff ok | fehlt das Bit: falsche Firmware/Registerprotokoll → `getProtocolStatus()` prüfen |
| 1 | `XM125D_STATUS_CONFIG_CREATE_OK` | Konfigurationsobjekt angelegt | fehlt: Konfiguration unplausibel (z.B. `endMm <= startMm`) |
| 2 | `XM125D_STATUS_SENSOR_CREATE_OK` | Sensorobjekt angelegt | fehlt: Modul-internes Problem → Reset |
| 3 | `XM125D_STATUS_DETECTOR_CREATE_OK` | Detektorobjekt angelegt | fehlt: Messbereich zu groß (Speicher) → `endMm` reduzieren |
| 23 | `XM125D_STATUS_CONFIG_APPLY_ERROR` | Konfiguration abgelehnt | Wertebereich prüfen (siehe [04-Konfiguration.md](04-Konfiguration.md)) |
| 24 | `XM125D_STATUS_SENSOR_CALIBRATE_ERROR` | **Sensor-Kalibrierung (Schritt 1) fehlgeschlagen** | Versorgung/Verkabelung, Temperatur außerhalb Spezifikation, Modul-Reset |
| 25 | `XM125D_STATUS_DETECTOR_CALIBRATE_ERROR` | **Detektor-Kalibrierung (Schritte 2–5) fehlgeschlagen** | Umgebung während Kalibrierung (Bewegung, Objekt im Nahbereich), `closeRangeLeakageCancellation` testweise aus, `startMm` erhöhen |
| 28 | `XM125_STATUS_DETECTOR_ERROR` | Sammelfehler | s.o., danach `calibrate()`, sonst `hardCalibration()` (Reset + alles neu) |
| 31 | `XM125_STATUS_BUSY` | Kommando läuft noch | warten; bleibt es dauerhaft gesetzt → `hardCalibration()`, hilft das nicht: Power-Cycle |

Auswertung als Hilfsfunktion:

```cpp
void printCalibStatus(uint32_t s) {
  Serial.print("DETECTOR_STATUS = 0x"); Serial.println(s, HEX);
  Serial.print("  RSS_REGISTER_OK      "); Serial.println((s & XM125D_STATUS_RSS_REGISTER_OK)   ? "1" : "0");
  Serial.print("  CONFIG_CREATE_OK     "); Serial.println((s & XM125D_STATUS_CONFIG_CREATE_OK)  ? "1" : "0");
  Serial.print("  SENSOR_CREATE_OK     "); Serial.println((s & XM125D_STATUS_SENSOR_CREATE_OK)  ? "1" : "0");
  Serial.print("  DETECTOR_CREATE_OK   "); Serial.println((s & XM125D_STATUS_DETECTOR_CREATE_OK)? "1" : "0");
  Serial.print("  CONFIG_APPLY_ERROR   "); Serial.println((s & XM125D_STATUS_CONFIG_APPLY_ERROR)      ? "1" : "0");
  Serial.print("  SENSOR_CAL_ERROR     "); Serial.println((s & XM125D_STATUS_SENSOR_CALIBRATE_ERROR)  ? "1" : "0");
  Serial.print("  DETECTOR_CAL_ERROR   "); Serial.println((s & XM125D_STATUS_DETECTOR_CALIBRATE_ERROR)? "1" : "0");
  Serial.print("  DETECTOR_ERROR       "); Serial.println(XM125Base::statusError(s) ? "1" : "0");
  Serial.print("  BUSY                 "); Serial.println(XM125Base::statusBusy(s)  ? "1" : "0");
}
```

## A.9 Presence-Detektor: Kalibrierung und Einschwingen

Der Presence-Detektor hat **kein eigenes Kalibrierkommando**. Die
Sensor-Kalibrierung passiert innerhalb von `APPLY_CONFIGURATION` (Kommando 1);
Erfolg zeigt Bit 3 `XM125P_STATUS_SENSOR_CALIBRATE_OK` (**✔ Code**).

Praktisch relevanter ist beim Presence-Detektor das **Einschwingen der
Filter** nach `start()` — das ist keine Kalibrierung, wirkt aber ähnlich:

| Filter | Zeitkonstante (Default) | Bedeutung fürs Einschwingen |
|---|---|---|
| `intraOutputTimeConstMs` | 300 ms | Intra-Score ist nach ~1 s brauchbar |
| `interOutputTimeConstMs` | **2000 ms** | Inter-Score braucht mehrere Sekunden, bis er aussagekräftig ist |
| `interFrameDeviationTimeConstMs` | 500 ms | Glättung der Frame-zu-Frame-Abweichung |

Faustregel: **nach `start()` mindestens 2–3 s** vergehen lassen (bzw. rund das
Dreifache von `interOutputTimeConstMs`), bevor `isPresenceDetected()`
ausgewertet wird — sonst können beim Anlauf Fehldetektionen auftreten. Während
dieser Zeit sollte sich vor dem Sensor nichts bewegen.

`resetFiltersOnPrepare` (Default `true`) sorgt dafür, dass bei jedem
`applyConfiguration()`/`start()` mit frischen Filterzuständen begonnen wird —
also wieder mit voller Einschwingzeit. Auf `false` gesetzt, werden alte
Zustände weiterverwendet (schnellerer Wiederanlauf, dafür ggf. Altlasten).

Nach einem `hasDetectorError()` genügt laut
[XM125_Presence.cpp](../examples/XM125_Presence.cpp) meist
`stop()` + `start()`; ein erneutes `applyConfiguration()` (und damit eine neue
Sensor-Kalibrierung) ist nur nötig, wenn sich die Konfiguration geändert hat.

## A.10 Empfohlene Startsequenz (Distance)

Vollständig, mit Prüfung aller Zwischenschritte:

```cpp
bool setupRadar(XM125Distance& radar) {
  // 1. I2C + Erreichbarkeit
  if (!radar.begin(Wire)) {
    Serial.println("XM125: nicht erreichbar (I2C/Adresse/Verkabelung)");
    return false;
  }

  // 2. Vorzustand prüfen: darf weder BUSY noch ERROR sein
  uint32_t st = radar.getDetectorStatus();
  if (XM125Base::statusBusy(st)) {
    Serial.println("XM125: noch BUSY - warte");
    uint32_t t0 = millis();
    while (XM125Base::statusBusy(radar.getDetectorStatus()) && millis() - t0 < 3000) {}
  }

  // 3. Konfiguration setzen (VOR dem Kalibrieren!)
  radar.config.startMm         = 200;
  radar.config.endMm           = 2500;
  radar.config.maxProfile      = XM125D_PROFILE5;
  radar.config.thresholdMethod = XM125D_THRESHOLD_CFAR;
  radar.config.peakSorting     = XM125D_SORT_CLOSEST;
  radar.config.reflectorShape  = XM125D_REFLECTOR_PLANAR;   // Bandenwand = flache Fläche

  // 4. Konfiguration schreiben + kalibrieren (Umgebung frei halten!)
  uint32_t t0 = millis();
  if (!radar.applyConfigAndCalibrate()) {
    uint16_t reg = radar.getLastFailedConfigRegister();
    if (reg != 0xFFFF) {
      Serial.print("XM125: I2C-Schreibfehler bei Register 0x"); Serial.println(reg, HEX);
    } else {
      Serial.println("XM125: APPLY_AND_CALIBRATE fehlgeschlagen (Timeout oder DETECTOR_ERROR)");
      printCalibStatus(radar.getDetectorStatus());
    }
    return false;
  }
  Serial.print("XM125: kalibriert in "); Serial.print(millis() - t0); Serial.println(" ms");

  // 5. Ergebnis gegenprüfen: Erfolgsbits müssen gesetzt, Fehlerbits frei sein
  st = radar.getDetectorStatus();
  const uint32_t wanted = XM125D_STATUS_RSS_REGISTER_OK | XM125D_STATUS_CONFIG_CREATE_OK |
                          XM125D_STATUS_SENSOR_CREATE_OK | XM125D_STATUS_DETECTOR_CREATE_OK;
  if ((st & wanted) != wanted) {
    Serial.println("XM125: WARNUNG - nicht alle Create-Bits gesetzt");
    printCalibStatus(st);
  }
  return true;
}
```

## A.11 Kalibrierstrategie im Wettkampfbetrieb

| Zeitpunkt | Aktion | Begründung |
|---|---|---|
| Power-On | `begin()` + `applyConfigAndCalibrate()` je Sensor, **nacheinander** | Pflicht; nacheinander wegen gegenseitiger Störung |
| Nach ~1–2 min Warmlaufen | einmal `recalibrate()` je Sensor | Elektronik ist dann auf Betriebstemperatur, Offset stimmt besser |
| Wenn `getCalibrationNeeded()` | `recalibrate()` (mit Zeitsperre, s. A.6) | Temperaturdrift |
| Nach 5 Fehlschlägen in Folge | volle Neuinitialisierung `setupSensor()` | so in allen Testskripten umgesetzt |
| Vor dem Spiel, Roboter auf Feld | **nicht** neu kalibrieren, wenn Personen/Roboter in Reichweite stehen | die Umgebung würde in die Referenz einfließen |

---

# Teil B — Kalibrierung der Peak-Rohwerte (Counts → mm)

## B.1 Ausgangslage

**Fakten aus dem Code** (**✔ Code**, [XM125Distance.cpp](../XM125Distance.cpp)):

- Gelesen wird das Peak-Register als volles 32-Bit-Wort:
  `_readReg(XM125D_REG_PEAK0_DISTANCE + i, dist)`.
- Das ungekürzte Wort landet in `_peakDistanceReg[i]` und ist über
  **`getPeakDistanceReg(idx)`** abrufbar — genau dafür gedacht, die offene
  Frage zu klären.
- `_peakDistanceRaw[i] = (uint8_t)(dist >> 24);` behält davon **nur das
  oberste Byte** — Wertebereich 0–255. Ob die Distanz überhaupt dort steckt,
  ist unbestätigt.
- `getPeakDistanceMm()` gibt diesen Rohwert derzeit **1:1** zurück, ist also
  ein Platzhalter und liefert **keine Millimeter**.
- Die Signalstärke wird dagegen als volle 32 Bit übernommen:
  `_peakStrength[i] = (int32_t)strength;`

**Was daraus folgt:** Solange der Umrechnungsfaktor unbekannt ist, dürfen
`getPeakDistanceMm()`-Werte nirgends als Distanz interpretiert werden. Die
Positionsbibliothek rechnet aktuell mit einer **Annahme** um
([lib/positioning/positioning.h](../../positioning/positioning.h)):

```cpp
distanz_cm = rawOffset_cm + raw * rawScale_cm;   // Default: 20.0 + raw * 0.902
```

Die Defaults `20.0 cm` / `0.902 cm` entsprechen genau Hypothese **H1** unten
(255 Counts spannen `startMm = 200 … endMm = 2500` auf) — **unbestätigt**.

## B.2 Schritt 0: erst das komplette 32-Bit-Wort ansehen

Bevor irgendeine Skalierung gemessen wird, muss geklärt sein, ob die drei
weggeworfenen Bytes wirklich leer sind. Dafür gibt es `getPeakDistanceReg()` —
es liefert das vollständige Registerwort, während `getPeakDistanceRaw()` 24 der
32 Bits verwirft:

```cpp
if (radar.measureBlocking(3000) && radar.getNumDistances() > 0) {   // Kalibrierskript: warten ist hier ok
  uint32_t w = radar.getPeakDistanceReg(0);
  Serial.print("Peak0 Register: 0x"); Serial.print(w, HEX);
  Serial.print("  Bytes: ");
  Serial.print((int)((w >> 24) & 0xFF)); Serial.print(".");
  Serial.print((int)((w >> 16) & 0xFF)); Serial.print(".");
  Serial.print((int)((w >>  8) & 0xFF)); Serial.print(".");
  Serial.println((int)(w & 0xFF));
  Serial.print("  byteswapped: "); Serial.println(__builtin_bswap32(w));
}
```

[XM125_WallCalibration.cpp](../examples/XM125_WallCalibration.cpp) macht
genau das automatisch: Zu jedem Kalibrierpunkt gibt es das Registerwort in vier
Deutungen aus — als `uint32`, byte-getauscht als `uint32`, als `float` und
byte-getauscht als `float`.

Interpretation des ausgegebenen Worts:

| Beobachtung | Deutung | Konsequenz |
|---|---|---|
| `0xNN000000` (nur oberstes Byte belegt) | Firmware liefert tatsächlich nur 1 Byte | weiter mit B.3 |
| `0x000000NN` (nur unterstes Byte belegt) | Library liest das falsche Byte | `dist & 0xFF` statt `dist >> 24` |
| `0x000003E8`-artig (plausible mm-Werte, hier 1000) | Es sind **echte Millimeter**, nur wird das falsche Byte extrahiert | `_peakDistanceRaw` durch vollen `uint32_t` ersetzen, Teil B erledigt sich |
| `0xE8030000` (Bytes gespiegelt, hier 1000 little-endian) | **Byte-Order-Problem**: das Modul sendet little-endian, `_readReg()` setzt big-endian zusammen | Wort mit `__builtin_bswap32(w)` drehen — dann echte mm |
| Wert springt beim langsamen Wegfahren sägezahnartig (…253, 254, 255, 0, 1, 2…) | ebenfalls Byte-Order/Überlauf: sichtbar wird nur ein Byte eines größeren Werts | wie oben, Bytes drehen |

> ⚠ **Der Sägezahn-Test ist der wichtigste Einzeltest.** Ein echter,
> gleichmäßig skalierter 0–255-Wert wächst über den ganzen Messbereich
> **monoton**. Springt er dagegen periodisch zurück, ist die Ursache kein
> fehlender Skalierungsfaktor, sondern eine falsch zusammengesetzte
> Mehr-Byte-Zahl — dann ist jede Regression aus B.6 sinnlos.

## B.3 Hypothesen

| # | Hypothese | Formel | Prüfung |
|---|---|---|---|
| **H1** | 255 Counts spannen den konfigurierten Bereich auf | `mm = startMm + raw · (endMm − startMm)/255` | Messreihe **B** (anderer Messbereich, gleiche physikalische Distanz) → Rohwert **ändert** sich |
| **H2** | fester Faktor, unabhängig vom Messbereich | `mm = offset + raw · k` | Messreihe **B** → Rohwert **bleibt gleich** |
| **H3** | Rohwert ist der Sample-Index | `mm = startMm + raw · stepLength · 2,5 mm` (**📘 Acconeer**: Punktabstand ≈ 2,5 mm bei Step Length 1) | Messreihe **C** (`maxStepLength` fest statt `0`) → Rohwert ändert sich proportional |
| **H4** | Byte-Order/Truncation (siehe B.2) | `mm = bswap32(wort)` | Sägezahn-Test / Rohwort-Dump |

H4 zuerst ausschließen, dann H1 vs. H2 über Messreihe B trennen, H3 nur falls
weder H1 noch H2 linear passen.

## B.4 Material & Aufbau

- **Reflektor:** flache Metallplatte oder Alu-kaschierte Platte, mindestens
  ca. 30 × 30 cm, senkrecht zur Strahlrichtung. (Eine Bandenwand aus dem
  Spielfeld ist ideal, weil sie dem späteren Einsatzfall entspricht.)
- **Maßband/Laserdistanzmesser**, Klebeband zum Markieren der Positionen.
- **Freies Umfeld:** möglichst nichts anderes im Strahlkegel, insbesondere
  keine Wand direkt hinter dem Reflektor innerhalb von `endMm`.
- **Referenzebene festlegen und aufschreiben:** ab welchem Punkt wird
  gemessen — Antennenoberfläche des XM125? Vorderkante des Gehäuses? Das ist
  später der `rawOffset`-Anteil. In der Positionsbibliothek entspricht das dem
  Konfigurationswert `radarRingRadius_cm` (Sensorstirn → Roboterzentrum,
  Default 9,0 cm) — beides nicht doppelt einrechnen!
- Sensor fest fixiert, **nicht** in der Hand halten.

## B.5 Messreihen

Für jede Messung: 20 Einzelmessungen, davon den **Median** notieren, und
Messungen mit `getNumDistances() != 1` verwerfen (dann ist etwas anderes im
Bild). Genau das erledigt `d <cm>` in
[XM125_WallCalibration.cpp](../examples/XM125_WallCalibration.cpp) —
die Tabellen unten sind zum Mitschreiben gedacht, falls von Hand gemessen wird
oder das Ergebnis dokumentiert werden soll.

**Reihe A — Grundkennlinie** (`startMm = 200`, `endMm = 2500`, `PROFILE5`,
`CFAR`, `peakSorting = CLOSEST`, `reflectorShape = PLANAR`, `maxStepLength = 0`):

| Distanz [mm] | 300 | 500 | 750 | 1000 | 1250 | 1500 | 1750 | 2000 | 2250 |
|---|---|---|---|---|---|---|---|---|---|
| Rohwert (Median) | | | | | | | | | |
| Strength | | | | | | | | | |
| `numDistances` | | | | | | | | | |

**Reihe B — anderer Messbereich, gleiche Positionen** (`startMm = 200`,
`endMm = 4000`, sonst identisch). Es genügen 4 Distanzen, z.B. 500 / 1000 /
1500 / 2000 mm:

| Distanz [mm] | 500 | 1000 | 1500 | 2000 |
|---|---|---|---|---|
| Rohwert Reihe A | | | | |
| Rohwert Reihe B | | | | |

*Auswertung:* Unterschiedliche Rohwerte → **H1** (bereichsrelativ). Gleiche
Rohwerte → **H2** (absolute Skala).

**Reihe C — nur falls A/B nicht linear aufgehen** (`maxStepLength = 1`, dann
`= 4`, sonst wie Reihe A) an 3 Distanzen. Ändert sich der Rohwert
proportional zur Schrittweite → **H3**.

**Reihe D — Wiederholbarkeit/Drift** (optional, aber aufschlussreich):
Objekt bei 1000 mm stehen lassen und 10 min lang alle 30 s messen, dazu
`getTemperature()` mitschreiben. Zeigt, wie stark der Wert mit der
Erwärmung driftet und ob `recalibrate()` ihn zurückholt.

## B.6 Auswertung

Wenn die Punkte aus Reihe A auf einer Geraden liegen, Kleinste-Quadrate-Fit
über alle `n` Messpunkte (`rawᵢ`, `dᵢ` in mm):

```
raw̄ = (1/n) Σ rawᵢ                d̄ = (1/n) Σ dᵢ

        Σ (rawᵢ − raw̄)(dᵢ − d̄)
scale = ──────────────────────────      [mm pro Count]
             Σ (rawᵢ − raw̄)²

offset = d̄ − scale · raw̄               [mm]
```

Schnellvariante mit zwei Punkten (z.B. 500 mm und 2000 mm):

```
scale  = (d₂ − d₁) / (raw₂ − raw₁)
offset = d₁ − scale · raw₁
```

**Linearität prüfen:** Für jeden Messpunkt das Residuum
`rᵢ = dᵢ − (offset + scale · rawᵢ)` ausrechnen. Liegen alle `|rᵢ|` innerhalb
weniger Prozent der Distanz, ist das lineare Modell tragfähig. Wandern die
Residuen systematisch (erst negativ, dann positiv), ist die Kennlinie
gekrümmt — dann entweder abschnittsweise linear interpolieren oder eine
Lookup-Tabelle hinterlegen.

**Erwartungswerte zur Plausibilitätsprüfung unter H1:**

| `startMm` | `endMm` | mm pro Count | raw bei 1000 mm |
|---|---|---|---|
| 200 | 2500 | (2500−200)/255 = **9,02** | (1000−200)/9,02 ≈ **89** |
| 200 | 4000 | (4000−200)/255 = **14,90** | (1000−200)/14,90 ≈ **54** |
| 250 | 3000 | (3000−250)/255 = **10,78** | (1000−250)/10,78 ≈ **70** |

## B.7 Ergebnis in den Code einbauen

Sobald `scale`/`offset` bekannt sind, gehört die Umrechnung in
`XM125Distance::getPeakDistanceMm()` — dort ist sie für alle Anwender
verfügbar, und der Rohwert bleibt über `getPeakDistanceRaw()` erhalten:

```cpp
// XM125Distance.h  (Konstanten aus der Kalibriermessung vom <Datum>)
static constexpr float XM125D_RAW_OFFSET_MM = 200.0f;  // gemessen
static constexpr float XM125D_RAW_SCALE_MM  = 9.02f;   // gemessen (mm pro Count)

// XM125Distance.cpp
uint32_t XM125Distance::getPeakDistanceMm(uint8_t idx) const {
    return (uint32_t)(XM125D_RAW_OFFSET_MM + getPeakDistanceRaw(idx) * XM125D_RAW_SCALE_MM);
}
```

Gilt die bereichsabhängige Variante (H1), sollte stattdessen aus `config`
gerechnet werden, damit die Umrechnung bei geändertem Messfenster mitzieht:

```cpp
uint32_t XM125Distance::getPeakDistanceMm(uint8_t idx) const {
    const float span = (float)(config.endMm - config.startMm) / 255.0f;
    return (uint32_t)(config.startMm + getPeakDistanceRaw(idx) * span);
}
```

**Nicht vergessen — zwei Stellen müssen zusammenpassen:**

1. `getPeakDistanceMm()` in dieser Library.
2. `Positioning::Config::rawOffset_cm` / `rawScale_cm` bzw.
   `rawCountToCm()` in [lib/positioning](../../positioning/positioning.md).
   Wird die Umrechnung hier eingebaut, sollte die Positionsbibliothek
   entweder die neuen Werte übernehmen oder direkt `getPeakDistanceMm()`
   verwenden, statt selbst zu skalieren — sonst wird zweimal skaliert.
3. Die Warnungen im Header und in dieser Doku entfernen bzw. durch die
   gemessenen Werte plus Messdatum ersetzen.

## B.8 Validierung

Nach dem Einbau gegenprüfen — mit Distanzen, die **nicht** in der
Kalibrierreihe vorkamen:

| Soll [mm] | Gemessen `getPeakDistanceMm()` | Abweichung |
|---|---|---|
| 400 | | |
| 900 | | |
| 1600 | | |
| 2200 | | |

Zusätzlich einmal quer prüfen: zwei Objekte gleichzeitig (z.B. 600 mm und
1400 mm) und `peakSorting = CLOSEST` — der erste Peak muss dann dem näheren
Objekt entsprechen.

## B.9 Fallstricke

- **Mehrere Peaks:** Steht außer dem Reflektor noch etwas im Bild, gehört der
  gesuchte Wert unter Umständen nicht zu Index 0. Deshalb `numDistances`
  mitschreiben und Messungen mit mehr als einem Peak verwerfen.
- **`NEAR_START_EDGE`** (`getNearStartEdge()`): Nahe `startMm` ist die
  Messung unzuverlässig — die kalibrierte Kennlinie deshalb nicht auf den
  Bereich direkt an der unteren Grenze stützen.
- **Schräge Montage:** Ist der Sensor nicht senkrecht zur Fläche, misst er den
  schrägen Abstand (Kosinusfehler). Für die Kalibrierreihe strikt senkrecht
  aufbauen.
- **Temperaturdrift während der Messreihe:** Bei langen Reihen zwischendurch
  `getCalibrationNeeded()` prüfen und ggf. `recalibrate()` — aber dann die
  bisherigen Punkte kritisch ansehen, weil sich der Offset verschoben haben
  kann.
- **Sensor-zu-Sensor-Streuung:** Die vier Module am Ring können leicht
  unterschiedliche Offsets haben. Die Reihe im Zweifel mit **einem** Sensor
  fahren und danach die anderen drei bei zwei Distanzen gegenprüfen; weichen
  sie ab, braucht jeder Sensor einen eigenen Offset.
- **Reflektorgröße:** Kleine Objekte liefern schwächere Peaks und werden
  eventuell erst gar nicht erkannt — die Strength-Spalte in der Tabelle hilft,
  das zu erkennen (sehr kleine Werte = grenzwertige Detektion).

## B.10 Werkzeug: `lib/XM125/examples/XM125_WallCalibration.cpp`

Die Messreihe muss nicht von Hand programmiert werden — dafür gibt es bereits
einen interaktiven Sketch:

```bash
cp lib/XM125/examples/XM125_WallCalibration.cpp src/main.cpp
pio run -e teensy40 -t upload
```

Bedienung über den seriellen Monitor:

| Befehl | Wirkung |
|---|---|
| `a <0..3>` | Sensor wählen (`0x51`–`0x54`) |
| `m` | Dauermessung an/aus — zeigt Peaks, Rohwert, Registerwort, Stärke, Trefferquote |
| `s` | Parameter-Sweep über `signalQuality` × `thresholdSensitivity` bei 2,5 m |
| `i` | Übersprech-Test (allein vs. alle Sensoren gleichzeitig) |
| `d <cm>` | Kalibrierpunkt bei bekanntem Abstand aufnehmen (20 Messungen, Median) |
| `f` | Punkte auswerten: Ausgleichsgerade `mm = a · Rohwert + b` samt Restfehler |
| `c` | Punkte löschen |
| `?` | Hilfe |

Die Modi `s` und `i` sind hier nur noch aus historischen Gründen enthalten —
sie sind inzwischen als eigenständige Skripte gebaut
([XM125_Calib_Sensitivity.cpp](../examples/XM125_Calib_Sensitivity.cpp) bzw.
[XM125_Calib_Crosstalk.cpp](../examples/XM125_Calib_Crosstalk.cpp)) und dort
weiter ausgebaut. Für Teil B relevant sind `d`, `f` und `c`.

Der Sketch nimmt einem damit die Schritte B.5 (Reihe A) und B.6 (Regression)
komplett ab — inklusive Median-Bildung, Verwerfen von Messungen ohne Peak und
der Ausgabe der vier Registerdeutungen aus B.2. Er ist auf `0x51`–`0x54`,
`CAL_START_MM = 200` und `CAL_END_MM = 2700` voreingestellt; für **Reihe B**
(Test H1 vs. H2) einfach `CAL_END_MM` im Sketch ändern, neu flashen und
dieselben physikalischen Abstände erneut mit `d <cm>` aufnehmen.

Zeigt `f` einen Restfehler > 100 mm, ist der 8-Bit-Rohwert nicht linear in der
Distanz — dann gilt die Auswertung aus B.2: Welche der vier Registerdeutungen
läuft monoton mit der Distanz mit?

---

## Checkliste

**1. Modul-Kalibrierung (Teil A)** — `XM125_Calib_Module`

- [ ] Sensor final montiert, Umfeld frei, nichts bewegt sich
- [ ] `config` vollständig gesetzt **vor** `applyConfigAndCalibrate()`
- [ ] Rückgabewert geprüft, bei `false`: `getLastFailedConfigRegister()` und `getDetectorStatus()` auswerten
- [ ] Bei mehreren Sensoren: nacheinander kalibrieren (Modus `b`)
- [ ] Dauer gegen das Timeout geprüft (Modus `n`)
- [ ] `getCalibrationNeeded()` im Betrieb auswerten → `recalibrate()` mit Zeitsperre (Modus `w`)
- [ ] Nach Power-Cycle immer neu kalibrieren (Kalibrierung ist flüchtig)

**2. Ausrichtung** — `XM125_Calib_Alignment`

- [ ] Jeder Sensor auf maximale Echostärke ausgerichtet (Modus `m`)
- [ ] Bewertung bei 2,5 m bestanden: volle Trefferquote, nur ein Peak (Modus `q`)

**3. Empfindlichkeit** — `XM125_Calib_Sensitivity`

- [ ] Modus `s` bei 2,5 m gefahren
- [ ] `RADAR_SIGNAL_QUALITY` / `RADAR_THRESHOLD_SENSITIVITY` in [params.h](../../include/params.h) eingetragen
- [ ] An einem zweiten Sensor gegengeprüft (Exemplarstreuung)

**4. Übersprechen** — `XM125_Calib_Crosstalk`

- [ ] Modus `g` gefahren (nacheinander gegen gleichzeitig), Ergebnis bewertet
- [ ] Bei Auffälligkeiten Modus `p`: welches Sensorpaar stört sich?

**Rohwert-Kalibrierung (Teil B)**

- [ ] Schritt 0: volles 32-Bit-Peak-Register (`getPeakDistanceReg()`) geloggt, Byte-Order ausgeschlossen
- [ ] Sägezahn-Test beim langsamen Wegfahren durchgeführt
- [ ] Reihe A (Grundkennlinie) aufgenommen — `d <cm>` je Distanz, dann `f`
- [ ] Reihe B (anderer Messbereich, `CAL_END_MM` ändern) → H1 vs. H2 entschieden
- [ ] Regression gerechnet, Residuen geprüft
- [ ] `getPeakDistanceMm()` angepasst, Warnungen im Header entfernt
- [ ] `lib/positioning` (`rawOffset_cm`/`rawScale_cm`) konsistent nachgezogen
- [ ] Validierungsmessung an ungenutzten Distanzen bestanden
