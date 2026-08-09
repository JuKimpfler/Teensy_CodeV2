# XM125 Beispiele — was jedes Skript tut

Neun eigenständige Sketche für den Acconeer-XM125-Radarsensor. Jeder hat
eigenes `setup()`/`loop()` und ist damit ein vollständiges, eigenständiges
Programm — keine Bibliothek im engeren Sinn.

## Bauen und Ausführen

PlatformIO baut immer genau eine `setup()`/`loop()`-Datei aus `src/`. Um eines
dieser Beispiele zu testen:

```bash
cp lib/XM125/examples/XM125_Connectivity.cpp src/main.cpp
pio run -e teensy40 -t upload
pio device monitor -b 115200
```

Die Original-`src/main.cpp` des Roboters vorher sichern (`git stash` oder
Kopie), sonst geht sie beim Zurückkopieren verloren. Nach dem Test:

```bash
git checkout -- src/main.cpp
```

Alle Skripte außer `XM125_Connectivity` und `XM125_Presence` binden
`lib/include/params.h` ein und benutzen dieselben Konstanten
(`RADAR_SENSOR_ADDRS`, `RADAR_START_MM`, `RADAR_SIGNAL_QUALITY`, …), mit denen
auch `systemlib::setupRadarSens()` den Roboter im Wettkampf konfiguriert —
was hier gemessen wird, gilt damit auch für den echten Betrieb.

Bedienung ist bei allen interaktiven Skripten gleich: Befehle über den
seriellen Monitor eintippen, mit Enter abschicken. `?` zeigt immer die Hilfe
mit allen Befehlen und dem aktuellen Zustand (aktiver Sensor, eingestellte
Werte).

## Reihenfolge bei der Inbetriebnahme

```
1. XM125_Connectivity        -- antwortet das Modul überhaupt?
2. XM125_Distance             -- ein Sensor, Grundfunktion
3. XM125_Calib_Module         -- Modul-Kalibrierung: läuft sie durch?
4. XM125_Calib_Alignment      -- Sensor mechanisch ausrichten
5. XM125_Calib_Sensitivity    -- Empfindlichkeit auf die Wand abstimmen
6. XM125_Calib_Crosstalk      -- wie viele Sensoren gleichzeitig?
7. XM125_Distance_Multi       -- alle vier Sensoren zusammen, mit PDS
```

`XM125_Presence` und `XM125_WallCalibration` stehen daneben (siehe unten).
Ausführliche Erklärung der Kalibriermethoden und Hintergründe:
[../doc/07-Kalibrierung.md](../doc/07-Kalibrierung.md).

---

## 1. `XM125_Connectivity.cpp`

**Zweck:** Grundlegendster Test — reagiert das Modul überhaupt auf I²C? Prüft
noch **keine** Firmware-spezifische Funktion (weder Distance- noch
Presence-Detector), sondern nur, was beide gemeinsam haben: `XM125Base`.

**Was passiert:**
- `setup()` versucht bis zu 5x (500 ms Abstand), sich zu verbinden
  (`begin()`). Schlägt das fehl, werden mögliche Ursachen ausgegeben
  (Verkabelung, Sleep-Zustand, unverkabelter `WAKE_UP`-Pin).
- Bei Erfolg: Version-Register, Protocol-Status und Detector-Status werden
  einmalig ausgegeben.
- `loop()` prüft alle 2 Sekunden erneut `isConnected()` und meldet, ob die
  Verbindung noch steht oder zwischenzeitlich abgerissen ist — nützlich, um
  einen wackelnden Stecker oder einen floatenden `WAKE_UP`-Pin sichtbar zu
  machen.

**Bedienung:** keine — reiner Beobachtungssketch, keine Befehle.

**Wann benutzen:** immer als allererstes bei neuer/geänderter Hardware. Läuft
das nicht, ist jeder weitere Test sinnlos.

---

## 2. `XM125_Distance.cpp`

**Zweck:** Ein einzelner Sensor (Adresse `0x51`), volle Kette: konfigurieren,
kalibrieren, messen, Ergebnis ausgeben. Referenzbeispiel für die
Distance-Detector-API.

**Was passiert:**
- `setupSensor()`: `begin()`, dann `config.startMm/endMm/maxProfile/
  thresholdMethod/peakSorting` setzen und `applyConfigAndCalibrate()`
  aufrufen. Bei Fehlschlag wird unterschieden zwischen I²C-Schreibfehler
  (`getLastFailedConfigRegister() != 0xFFFF`) und Kommando-Fehlschlag
  (Timeout/`DETECTOR_ERROR`), dazu Protocol-/Detector-Status ausgegeben.
- `loop()` misst zyklisch **ohne zu warten**: `startMeasurement()`, dann in
  jedem Durchlauf `isMeasuring()` prüfen und erst bei `false` das Ergebnis
  über `getLastResult()` abholen. Ausgegeben werden Messdauer, Peak-Anzahl,
  Temperatur und `NEAR_START_EDGE`. Bei Fehlschlag: Diagnose, bei
  `calibrationNeeded` automatisch `recalibrate()`, nach 5
  Fehlschlägen in Folge komplette Neuinitialisierung.
- Zu jedem Peak: Rohwert (`getPeakDistanceRaw()`, 0–255 "Counts") und
  Signalstärke — **keine** echten Millimeter (siehe Warnung im Code und
  [../doc/07-Kalibrierung.md, Teil B](../doc/07-Kalibrierung.md#teil-b--kalibrierung-der-peak-rohwerte-counts--mm)).

**Bedienung:** keine — läuft automatisch, Ausgabe nur lesen.

**Wann benutzen:** nach `XM125_Connectivity`, um die Grundfunktion eines
einzelnen Sensors zu bestätigen, bevor mit mehreren Sensoren oder der
Kalibrierabstimmung weitergemacht wird.

---

## 3. `XM125_Calib_Module.cpp`

**Zweck:** Kalibriermethode 1 von 4 — testet **nur** die modul-interne
Kalibrierung (Sensor-Hardware + Detektor-Rauschpegel/Offset/Leakage, Teil A
der Kalibrier-Doku), unabhängig davon, ob die Wand überhaupt erkannt wird.
Beantwortet: läuft die Kalibrierung sauber durch, wie lange dauert sie, und
hält sie im Dauerbetrieb (Temperaturdrift)?

**Befehle:**

| Befehl | Was er tut |
|---|---|
| `a <0..3>` | Sensor wählen (Adresse `0x51`–`0x54`) und sofort kalibrieren |
| `k` | `applyConfigAndCalibrate()` — Konfiguration aus `params.h` schreiben + voll kalibrieren |
| `v` | `calibrate()` — voll neu kalibrieren, Konfiguration bleibt wie sie ist |
| `r` | `recalibrate()` — schnelle Nachkalibrierung (Reaktion auf Temperaturdrift) |
| `n <N>` | N Kalibrierungen hintereinander (max. 20): Dauer min/mittel/max, Fehlerquote, Warnung falls die Dauer nahe an das 3000-ms-Timeout heranreicht |
| `b` | Startsequenz wie beim Roboter: alle vier Sensoren **nacheinander** kalibrieren, Gesamtdauer messen |
| `s` | Alle Statusbits des aktiven Sensors einzeln ausgeben (Version, Protocol-Status, Detector-Status inkl. Create-/Error-Bits) |
| `w` | Driftbeobachtung an/aus: misst alle 5 s, protokolliert Temperatur, Peak-Rohwert und `CALIBRATION_NEEDED`; löst bei Bedarf automatisch `recalibrate()` aus (mit 2-s-Zeitsperre) |

**Warum eigenständig:** Wenn die Wand nicht erkannt wird, ist die erste
Frage, ob das an der Kalibrierung selbst liegt (Modul kaputt, Timeout zu
knapp, Umgebung während der Kalibrierung gestört) oder an der Empfindlichkeit
(Kalibrierung ok, aber die Schwelle passt nicht). Dieses Skript beantwortet
ausschließlich die erste Frage, bevor überhaupt eine Wand nötig ist.

**Messaufbau:** Sensoren final montiert, Umfeld während jeder Kalibrierung
frei, nichts bewegt sich, stabile Versorgung (Motoren aus). Für `w` zusätzlich
ein Objekt in fester, bekannter Distanz aufstellen und während der gesamten
Beobachtung nicht bewegen.

---

## 4. `XM125_Calib_Alignment.cpp`

**Zweck:** Kalibriermethode 2 von 4 — mechanische Ausrichtung des Sensors auf
die Wand. Eine glatte Wand reflektiert spiegelnd; schon 10–15° Schräglage
lenken das Echo am Sensor vorbei. Ohne saubere Ausrichtung bringt auch die
beste Empfindlichkeitseinstellung nichts.

**Befehle:**

| Befehl | Was er tut |
|---|---|
| `a <0..3>` | Sensor wählen (0=Vo, 1=Li, 2=Hi, 3=Re — wie `systemlib::radarSlot()`), Maximum zurücksetzen |
| `m` | Live-Anzeige an/aus: Balken skaliert auf das bisher beste Echo, dazu Signalstärke, Maximum, Rohwert, Peak-Anzahl. Sensor/Roboter drehen, bis der Balken wieder voll ist |
| `q` | Ausrichtung bewerten: 20 Messungen, Trefferquote, Stärke min/median/max, Streuung, Anteil Mehrfach-Peaks und `NEAR_START_EDGE` |
| `l` | Alle vier Sensoren einmal messen — zeigt auf einen Blick, welcher (falls einer) gar nichts sieht |
| `z` | Bestes Echo (Balken-Referenz) und Trefferzähler zurücksetzen — z.B. bei neuem Abstand oder neuem Sensor |
| `k` | Neu kalibrieren, z.B. nach dem Verschieben/Umbauen des Sensors |

**Warum eigenständig:** Ausrichtung ist ein rein mechanisches Problem, das
sich nicht durch Software-Parameter kompensieren lässt. Getrennt von der
Empfindlichkeitsabstimmung zu testen verhindert, dass eine schlechte
Ausrichtung durch hochgedrehte Empfindlichkeit kaschiert wird (was später im
Betrieb zu Fehlalarmen führt).

**Messaufbau:** Wand auf den größten gewünschten Abstand stellen (2,5 m) —
dort ist die Reserve am kleinsten, näher wird es nur besser. Sonst nichts im
Strahlkegel, Sensor nicht in der Hand halten.

---

## 5. `XM125_Calib_Sensitivity.cpp`

**Zweck:** Kalibriermethode 3 von 4 — findet `signalQuality` und
`thresholdSensitivity` (plus `maxProfile` und `reflectorShape`), mit denen die
Wand am äußersten Nutzabstand **zuverlässig** als Peak auftaucht, ohne
unnötig langsam oder fehleranfällig zu werden. Ergebnis wandert nach
`lib/include/params.h` (`RADAR_SIGNAL_QUALITY`, `RADAR_THRESHOLD_SENSITIVITY`).

**Befehle:**

| Befehl | Was er tut |
|---|---|
| `a <0..3>` | Sensor wählen |
| `t` | Aktuelle Einstellung mit 20 Messungen testen |
| `s` | **Hauptmodus:** Sweep `signalQuality` × `thresholdSensitivity` (4×4 = 16 Kombinationen), gibt die günstigste Kombination mit voller Trefferquote aus und übernimmt sie |
| `p` | Sweep `maxProfile` (PROFILE1..5): bringt das lange Profil wirklich mehr als ein kurzes? |
| `r` | `reflectorShape` PLANAR gegen GENERIC direkt vergleichen |
| `q` | Fertigen Werte-Block für `params.h` ausgeben |
| `d` | Zurück auf die aktuellen Werte aus `params.h` |

**Warum eigenständig:** Der Sweep über 16 Kombinationen dauert mehrere
Minuten (jede Kombination wird neu kalibriert). Als eigenständiges Skript
lässt er sich gezielt für genau diesen einen Zweck starten, ohne die anderen,
schnelleren Kalibrierschritte mitzuschleppen.

**Messaufbau:** Wand auf 2,5 m, Sensor vorher mit `XM125_Calib_Alignment`
ausgerichtet — ein Sweep auf einen schlecht getroffenen Reflektor liefert nur
zu hohe, unnötig fehleranfällige Empfindlichkeiten.

---

## 6. `XM125_Calib_Crosstalk.cpp`

**Zweck:** Kalibriermethode 4 von 4 — stören sich die Sensoren gegenseitig,
wenn sie gleichzeitig messen? Der Roboter misst immer gleichzeitig
(`systemlib::readRadar()`); Gegenmittel bei Befund sind Montage und
Empfindlichkeit, kein Parameter in `params.h`.

**Befehle:**

| Befehl | Was er tut |
|---|---|
| `a <0..3>` | Referenzsensor wählen |
| `i` | Grundtest: Referenzsensor allein vs. alle drei Nachbarn senden gleichzeitig mit — Trefferverlust und Stärkeverlust |
| `p` | Paarweise: jeder Nachbar einzeln als Störer — zeigt, **welches** Sensorpaar sich nicht verträgt |
| `g` | **Hauptmodus:** komplette Messrunden (wie `systemlib::readRadar()`) einmal nacheinander und einmal gleichzeitig — Trefferquote je Sensor UND Rundendauer nebeneinander, daraus eine Bewertung |
| `k` | Alle Sensoren nacheinander neu kalibrieren |

**Warum eigenständig:** Übersprechen lässt sich nur mit allen vier Sensoren
gleichzeitig aktiv beurteilen — das ist ein Test über das gesamte System,
nicht über einen einzelnen Sensor, und braucht deshalb eigene Logik
(Nachbarn gezielt zum Senden anstoßen, komplette Gruppen-Zyklen nachbilden).

**Messaufbau:** Alle vier Sensoren montiert und angeschlossen. Vor jedem
Sensor eine Wand in 2,5 m (Roboter mittig zwischen den Banden) — dort zeigt
sich Übersprechen zuerst. Empfindlichkeit vorher mit
`XM125_Calib_Sensitivity` abgestimmt, sonst misst dieser Test nur, dass die
Wand ohnehin nicht sicher erkannt wird.

---

## 7. `XM125_Distance_Multi.cpp`

**Zweck:** Alle vier Sensoren gleichzeitig im Dauerbetrieb, inklusive
Live-Ausgabe über das Power Debug System (PDS) — der praxisnaheste Test vor
der Integration in `systemlib`.

**Was passiert:**
- `setupSensor(idx)` für jeden der vier Sensoren (`0x51`–`0x54`):
  konfigurieren (Wand bis 2500 mm, PROFILE5, CFAR, `CLOSEST`, `PLANAR`) und
  kalibrieren.
- `loop()` misst reihum jeden Sensor, gibt Peaks/Temperatur/Rohwert/Stärke
  aus und schreibt den Rohwert zusätzlich in einen PDS-Kanal
  (`PDS.Channel(150+i, ...)`) — damit lässt sich der Rohwert über die Zeit
  live plotten (auch das beste Werkzeug für den Sägezahn-Test aus
  [../doc/07-Kalibrierung.md](../doc/07-Kalibrierung.md#b2-schritt-0-erst-das-komplette-32-bit-wort-ansehen)).
- Bei Fehlern: Fehlerzähler pro Sensor, nach 5 Fehlschlägen in Folge
  Neuinitialisierung nur dieses einen Sensors.

**Bedienung:** keine — läuft automatisch.

**Wann benutzen:** letzter Schritt vor der Integration in die
Positionsbibliothek/`systemlib` — bestätigt, dass alle vier Sensoren
zusammen im Dauerbetrieb stabil laufen.

---

## 8. `XM125_Presence.cpp`

**Zweck:** Beispiel für den **Presence-Detector** (andere Firmware als
Distance!) — Anwesenheitserkennung über schnelle/langsame Bewegung im
Bereich 300–2000 mm. Steht außerhalb der Distance-Kalibrierkette, weil der
Presence-Detector im Roboter aktuell nicht verwendet wird.

**Was passiert:**
- `setupSensor()`: `begin()`, `config.startMm/endMm` setzen,
  `applyConfiguration()` (kalibriert die Sensor-Hardware als Teil davon,
  siehe [../doc/07-Kalibrierung.md, A.9](../doc/07-Kalibrierung.md#a9-presence-detektor-kalibrierung-und-einschwingen)),
  dann `start()`.
- `loop()` ruft `update()` auf und gibt `isPresenceDetected()` (inkl.
  "sticky"-Variante), bei Erkennung die Distanz, sowie Intra-/Inter-Score
  aus. Bei `hasDetectorError()`: `stop()` + `start()`.

**Bedienung:** keine — läuft automatisch.

**Wann benutzen:** nur relevant, falls ein Modul mit Presence- statt
Distance-Firmware eingesetzt werden soll.

---

## 9. `XM125_WallCalibration.cpp`

**Zweck:** Das ursprüngliche All-in-one-Kalibrierwerkzeug. Die Modi `s`
(Empfindlichkeits-Sweep) und `i` (Übersprech-Test) sind inzwischen durch die
eigenständigen, weiter ausgebauten Skripte `XM125_Calib_Sensitivity` und
`XM125_Calib_Crosstalk` abgelöst und nur noch aus historischen Gründen
enthalten. **Aktiv gebraucht wird dieses Skript nur noch für Teil B** — die
offene Frage, wie sich der 8-Bit-Peak-Rohwert (0–255 "Counts") in echte
Millimeter umrechnet.

**Befehle (aktiv genutzt):**

| Befehl | Was er tut |
|---|---|
| `a <0..3>` | Sensor wählen |
| `d <cm>` | Kalibrierpunkt bei bekanntem, tatsächlichem Abstand aufnehmen: 20 Messungen, Median von Rohwert/Registerwort/Stärke, verwirft Messungen ohne Peak |
| `f` | Aufgenommene Punkte auswerten: Ausgleichsgerade `mm = a · Rohwert + b`, Restfehler je Punkt. Zu jedem Punkt werden zusätzlich die vier Deutungen des vollständigen 32-bit-Registerworts ausgegeben (uint32, byte-getauscht, float, float byte-getauscht) — falls der 8-Bit-Wert nicht linear ist, hilft das zu erkennen, welche Bytes wirklich die Distanz tragen |
| `c` | Aufgenommene Punkte löschen |

**Befehle (historisch, siehe oben):** `m` (Dauermessung), `s`
(Empfindlichkeits-Sweep), `i` (Übersprech-Test).

**Warum nicht aufgeteilt wie die anderen:** Teil B ist kein abgeschlossenes,
wiederholt anzuwendendes Kalibrierverfahren, sondern eine offene Messreihe
zur Bestimmung eines bisher unbekannten Faktors. Sobald der Umrechnungsfaktor
feststeht und in `XM125Distance::getPeakDistanceMm()` eingebaut ist, verliert
dieses Skript seinen Zweck komplett — eine weitere Aufspaltung würde sich
nicht amortisieren. Details: [../doc/07-Kalibrierung.md, Teil B](../doc/07-Kalibrierung.md#teil-b--kalibrierung-der-peak-rohwerte-counts--mm).

**Messaufbau:** Ein einzelnes, flaches Objekt (Holzwand) auf exakt bekannter
Distanz, sonst nichts im Messkegel. Referenzebene (ab wo gemessen wird)
vorher festlegen und nicht doppelt mit `radarRingRadius_cm` in der
Positionsbibliothek verrechnen.

---

## Übersichtstabelle

| Skript | Kategorie | Ergebnis / Zweck | Dauer |
|---|---|---|---|
| `XM125_Connectivity` | Diagnose | Modul erreichbar? | Sekunden |
| `XM125_Distance` | Beispiel | Grundfunktion, ein Sensor | läuft dauerhaft |
| `XM125_Calib_Module` | Kalibrierung 1/4 | Modul-Kalibrierung läuft sauber? | je nach Modus |
| `XM125_Calib_Alignment` | Kalibrierung 2/4 | Montage | läuft dauerhaft |
| `XM125_Calib_Sensitivity` | Kalibrierung 3/4 | `RADAR_SIGNAL_QUALITY`, `RADAR_THRESHOLD_SENSITIVITY` | Sweep: mehrere Minuten |
| `XM125_Calib_Crosstalk` | Kalibrierung 4/4 | Übersprechen beim gleichzeitigen Messen | unter einer Minute |
| `XM125_Distance_Multi` | Integrationstest | alle 4 Sensoren + PDS | läuft dauerhaft |
| `XM125_Presence` | Beispiel | Presence-Detector-API | läuft dauerhaft |
| `XM125_WallCalibration` | Kalibrierung (offen) | Rohwert → mm (Teil B) | Messreihe, mehrere Punkte |

Hintergrund zu allen vier Distance-Kalibriermethoden, Statusbits und
Fehlerursachen: [../doc/07-Kalibrierung.md](../doc/07-Kalibrierung.md).
Allgemeine API-Referenz: [../doc/08-API-Referenz.md](../doc/08-API-Referenz.md).
