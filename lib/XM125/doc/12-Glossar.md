# Glossar

Alle Begriffe, die im Code, in den Registernamen und in dieser Dokumentation
vorkommen — kurz erklärt. Sortiert nach Sachzusammenhang, nicht alphabetisch.

---

## Hardware & Firmware

**A121**
Der eigentliche Radarchip auf dem XM125: gepulster kohärenter 60-GHz-Radar
(*Pulsed Coherent Radar*, PCR). "Kohärent" heißt, dass neben der Laufzeit auch
die Phase des Echos ausgewertet wird — dadurch sind sehr kleine Bewegungen
(Atmung) erkennbar, worauf der Presence-Detektor aufbaut.

**XM125**
Modul aus A121 + STM32-MCU + Spannungsversorgung. Die MCU führt eine der
Acconeer-Applikationen aus und stellt sie über I²C als Registerinterface
bereit.

**RSS** — *Radar System Software*
Die Acconeer-Softwarebibliothek auf der Modul-MCU. Statusbits wie
`RSS_REGISTER_OK` beziehen sich darauf. `getVersionRaw()` liefert die
RSS-Version.

**Detector / Detektor**
Die fertige Auswertelogik oberhalb der RSS: *Distance Detector* (Peaks und
Distanzen) oder *Presence Detector* (Anwesenheit). Auf einem Modul läuft
immer nur einer davon.

---

## Messaufbau des Radars

**Sample / Point**
Ein einzelner Messpunkt entlang der Distanzachse. Der A121 tastet den
Messbereich in diskreten Punkten ab; der Abstand zwischen zwei Punkten bei
Step Length 1 liegt bei etwa **2,5 mm** (📘 Acconeer).

**Step Length** — `maxStepLength`, `manualStepLength`
Abstand zwischen benachbarten Messpunkten, in Vielfachen des Grundabstands.
Größere Schrittweite = weniger Punkte = schnellere Messung, aber gröbere
Auflösung. `0` bzw. `autoStepLengthEnabled = true` überlässt die Wahl der
Firmware.

**Sweep**
Ein einzelner Durchlauf über alle Messpunkte des konfigurierten Bereichs.

**Frame**
Eine Gruppe von Sweeps, die gemeinsam ausgewertet wird
(`sweepsPerFrame`, Presence-Detektor: Default 16). Der Presence-Detektor
vergleicht **innerhalb** eines Frames (intra) und **zwischen** Frames (inter).

**Frame Rate** — `frameRateMilliHz`
Rate, mit der Frames aufgenommen werden, in **Milli-Hertz**. `12000` = 12 Hz.

**HWAAS** — *Hardware Accelerated Average Samples*
Anzahl der Hardware-seitig gemittelten Einzelmessungen pro Messpunkt
(Presence: `hwaas`, Default 32). Mehr Mittelung = besseres Signal-Rausch-
Verhältnis, aber längere Messzeit und höherer Stromverbrauch.

**Profile** — `maxProfile`, `manualProfile`, `PROFILE1`…`PROFILE5`
Vorgegebene Kombination aus Pulslänge und Empfängereinstellungen. Höhere
Profile senden längere Pulse: mehr Energie, größere Reichweite — aber
schlechtere Auflösung im Nahbereich und größerer Blindbereich direkt vor dem
Sensor. `PROFILE5` = maximale Reichweite (Default beim Distance-Detektor
dieses Projekts), `PROFILE1` = beste Nahbereichsauflösung.

**Signal Quality** — `signalQuality`
Zielvorgabe für das Signal-Rausch-Verhältnis (1000×-Einheit, Default 15000).
Die Firmware stellt daraufhin intern Mittelung/Messzeit ein. Höher = sauberer,
aber langsamer.

---

## Auswertung beim Distance-Detektor

**Peak**
Ein lokales Maximum im Echoprofil, das die Schwellwertprüfung überstanden hat
— also ein erkannter Reflektor. Bis zu 10 Peaks je Messung
(`XM125D_MAX_PEAKS`).

**Amplitude**
Rohe Signalhöhe eines Echos vor jeder Umrechnung. Basis für
`thresholdMethod = FIXED_AMPLITUDE`.

**Strength** — `getPeakStrength()`
Signalstärke eines Peaks in einer RSS-internen, kalibrierten Einheit
(1000×-Skalierung). Nützlich, um echte Reflektoren von Grenzfällen zu
unterscheiden: sehr kleine Werte = wackelige Detektion.

**Threshold / Schwellwert**
Kurve über der Distanzachse, oberhalb derer ein Maximum als Peak gilt. Vier
Verfahren:

| Verfahren | Prinzip |
|---|---|
| `FIXED_AMPLITUDE` | konstante Amplitudenschwelle |
| `RECORDED` | Schwelle aus aufgezeichneten Referenzframes der **leeren** Szene |
| `CFAR` | adaptiv, siehe unten (Default) |
| `FIXED_STRENGTH` | konstante Schwelle auf der kalibrierten Stärkeskala |

**CFAR** — *Constant False Alarm Rate*
Adaptives Schwellwertverfahren: Für jeden Messpunkt wird die Schwelle aus dem
Rauschpegel **in seiner Nachbarschaft** gebildet. Dadurch bleibt die
Fehlalarmrate ungefähr konstant, egal ob die Umgebung stark oder schwach
reflektiert. `thresholdSensitivity` (0–1000) stellt ein, wie empfindlich
das Verfahren arbeitet.

**Peak Sorting** — `peakSorting`
Reihenfolge der zurückgegebenen Peaks: `CLOSEST` (nächster zuerst) oder
`STRONGEST` (stärkster zuerst). Bestimmt, was `getPeakDistanceRaw(0)`
bedeutet — für die Wanderkennung im Projekt ist `CLOSEST` sinnvoll.

**Reflector Shape** — `reflectorShape`
Erwartete Form des Reflektors: `PLANAR` für große flache Flächen (Wand,
Bande), `GENERIC` für beliebige Objekte. Beeinflusst, wie die Firmware aus der
Amplitude auf die Stärke schließt (die Abnahme über die Distanz ist für Fläche
und Punktziel unterschiedlich).

**Leakage / Direct Leakage**
Direktes Übersprechen vom Sender zum Empfänger — ein starkes "Echo" bei
Distanz ~0, das echte Objekte im Nahbereich überdecken kann.

**Close Range Leakage Cancellation** — `closeRangeLeakageCancellation`
Kompensation dieser Leakage: Bei der Kalibrierung wird ihre Signatur
aufgezeichnet und später von jeder Messung abgezogen. Deshalb muss der
Nahbereich während der Kalibrierung **frei** sein.

**Near Start Edge** — `getNearStartEdge()`
Flag: Es wurde ein Objekt sehr nah an der unteren Messbereichsgrenze
(`startMm`) erkannt. Dort ist der Messwert unzuverlässig; ggf. `startMm`
verkleinern oder Ergebnis verwerfen.

**Calibration Needed** — `getCalibrationNeeded()`
Flag: Der Detektor hält seine Kalibrierung für nicht mehr gültig (meist
Temperaturdrift) und bittet um `recalibrate()`.

---

## Auswertung beim Presence-Detektor

**Intra-Frame-Erkennung**
Auswertung der Veränderungen **innerhalb** eines Frames → erkennt **schnelle**
Bewegung (Gestik, Gehen). Parameter: `intraDetectionThreshold`,
`intraFrameTimeConstMs`, `intraOutputTimeConstMs`.

**Inter-Frame-Erkennung**
Auswertung der Veränderungen **zwischen** aufeinanderfolgenden Frames →
erkennt **langsame** Bewegung bis hin zur Atmung einer stillsitzenden Person.
Parameter: `interDetectionThreshold`, `interFrameDeviationTimeConstMs`, die
beiden Cutoff-Frequenzen, `interOutputTimeConstMs`.

**Fast / Slow Cutoff** — `interFrameFastCutoffMilliHz`, `interFrameSlowCutoffMilliHz`
Ober- und Untergrenze des Bandpasses, mit dem die Inter-Frame-Auswertung
arbeitet — in mHz, also `6000` = 6 Hz und `200` = 0,2 Hz. Alles außerhalb
gilt als Rauschen bzw. als statische Szene.

**Presence Score** — `getIntraPresenceScore()`, `getInterPresenceScore()`
Kontinuierliche Maßzahl der jeweiligen Erkennung. Überschreitet sie die
zugehörige Schwelle, wird Anwesenheit gemeldet. Zum Einstellen der Schwellen
die Scores mitloggen.

**Sticky** — `isPresenceDetectedSticky()`
Gehaltene Detektion: bleibt nach der letzten Erkennung noch
`interFramePresenceTimeoutS` Sekunden gesetzt. Verhindert Flackern bei
kurzzeitigem Signalverlust.

**Inter Phase Boost** — `interPhaseBoostEnabled`
Erhöht kurzzeitig die Empfindlichkeit der Inter-Erkennung, wenn eine mögliche
Bewegung vermutet wird.

**Reset Filters On Prepare** — `resetFiltersOnPrepare`
Setzt beim (Neu-)Start alle Filterzustände zurück — sauberer Start, dafür
volle Einschwingzeit.

---

## Protokoll & Ablauf

**Register**
32-Bit-Wert unter einer 16-Bit-Adresse. Alle Kommunikation läuft über
Register-Lesen/Schreiben, siehe
[09-Register-und-Konstanten.md](09-Register-und-Konstanten.md).

**Big-Endian**
Höchstwertiges Byte zuerst — so werden Adresse und Wert auf dem Bus
übertragen (`_writeReg()`/`_readReg()`).

**Repeated Start (Sr)**
I²C-Sequenz, bei der zwischen Schreib- und Lesephase **kein** STOP gesendet
wird. Nötig, damit das Modul die zuvor geschriebene Registeradresse behält.
Im Code: `endTransmission(false)`.

**Clock Stretching**
Der Slave hält SCL auf Low, um den Master zu bremsen. Das XM125 tut das
unmittelbar nach Kommandoempfang; manche Host-Treiber melden daraufhin einen
Schreibfehler, obwohl alles in Ordnung ist — siehe Workaround in
[10-Interna-und-Timing.md](10-Interna-und-Timing.md#warum-der-clock-stretching-workaround-nötig-ist).

**BUSY (Bit 31)**
Das Modul führt noch ein Kommando aus. Alle Kommando-Methoden dieser Library
warten, bis das Bit zurückgeht (`_waitNotBusy()`).

**Detector Status (Reg. `0x0003`)**
Sammelregister mit Erfolgs- und Fehlerbits; Bit-Bedeutung unterscheidet sich
zwischen Distance- und Presence-Firmware.

**Protocol Status (Reg. `0x0001`)**
Fehler auf Register-/Kommandoebene, z.B. unbekanntes Register für die
geflashte Firmware. `!= 0` ist ein starkes Indiz für die falsche Klasse
(Distance vs. Presence).

**Measure Counter (Reg. `0x0002`)**
Zähler der Messungen seit dem letzten Modul-Neustart — springt er auf 0, hat
das Modul neu gestartet (Brown-Out, Reset).

---

## Einheiten & Konventionen

| Konvention | Bedeutung |
|---|---|
| **1000×-Einheit** | Ganzzahl, die den 1000-fachen Wert trägt: `1000` ≙ 1,0. Betrifft u.a. `signalQuality`, `thresholdSensitivity`, `intraDetectionThreshold`, `interDetectionThreshold` |
| **mHz** | Milli-Hertz: `12000` mHz = 12 Hz |
| **mm** | Alle Distanzkonfigurationen (`startMm`, `endMm`) sind Millimeter |
| **ms / s** | Zeitkonstanten in Millisekunden, `interFramePresenceTimeoutS` in Sekunden |
| **"Counts"** | Projektinterne Bezeichnung für den unkalibrierten 0–255-Rohwert der Peak-Distanz — **keine** Einheit im Sinne des Datenblatts, siehe [07-Kalibrierung.md](07-Kalibrierung.md) |

Projektweite Abkürzungen (aus [test/Coding_Rules.md](../../../test/Coding_Rules.md)):
`Dir` = Direction, `Pos` = Position, `Calib` = Calibrate, `Var` = Variables,
`US` = Ultraschall, `us` = Mikrosekunden, `ms` = Millisekunden,
`DBG` = Debugger.
