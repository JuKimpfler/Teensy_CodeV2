# API-Referenz — jede Funktion einzeln

Vollständige Referenz aller öffentlichen (und der geschützten, für
Erweiterungen relevanten) Methoden der drei Klassen. Konfigurationsfelder sind
in [04-Konfiguration.md](04-Konfiguration.md) beschrieben, Registerwerte in
[09-Register-und-Konstanten.md](09-Register-und-Konstanten.md).

Legende der Spalte *Blockiert*:
**nein** = einige I²C-Transaktionen (µs–ms) ·
**kurz** = bis ~50 ms ·
**lang** = bis zum angegebenen Timeout (Sekunden).

---

# 1. `XM125Base` — Basisklasse

Datei: [XM125.h](../XM125.h) / [XM125.cpp](../XM125.cpp)

Enthält alles, was Distance- und Presence-Firmware gemeinsam haben:
I²C-Zugriff, Wake/Ready-Handling, gemeinsame Register. Kann auch **direkt**
instanziiert werden — genau das macht
[XM125_Connectivity.cpp](../examples/XM125_Connectivity.cpp), um ein
Modul zu prüfen, ohne zu wissen, welche Firmware darauf läuft.

## `XM125Base(uint8_t i2cAddr, int8_t wakePin, int8_t intPin)`

```cpp
explicit XM125Base(uint8_t i2cAddr = XM125_I2C_ADDR_DEFAULT,
                   int8_t  wakePin = -1,
                   int8_t  intPin  = -1);
```

| Parameter | Default | Bedeutung |
|---|---|---|
| `i2cAddr` | `XM125_I2C_ADDR_DEFAULT` (`0x55`) | 7-Bit-I²C-Adresse des Moduls. Wird **nicht** von der Software gesetzt, sondern muss zur Hardware/Firmware des Moduls passen (im Projekt: `0x51`–`0x54`) |
| `wakePin` | `-1` | MCU-Pin am `WAKE_UP`-Eingang des Moduls. `-1` = nicht verkabelt |
| `intPin` | `-1` | MCU-Pin am `MCU_INT`-Ausgang des Moduls. `-1` = nicht verkabelt |

- Der Konstruktor macht **keinen** I²C-Zugriff und setzt **keine** Pin-Modi —
  er speichert nur die Parameter. Objekte dürfen deshalb global/statisch
  angelegt werden (so in allen Testskripten).
- `_wire` ist bis zum ersten `begin()` `nullptr`; alle Registerzugriffe
  liefern bis dahin `false` bzw. `0`.

## `bool begin(TwoWire& wire = Wire)`

Initialisiert die Pins und prüft die Erreichbarkeit.

| | |
|---|---|
| **Rückgabe** | `true`, wenn das Modul auf seiner Adresse antwortet |
| **Blockiert** | kurz bis lang (bis `XM125_WAKEUP_TIMEOUT_MS` = 1500 ms, falls nichts antwortet) |
| **Vorbedingung** | `wire.begin()` muss **vorher** aufgerufen sein — die Library ruft es nicht selbst auf |
| **Nebenwirkungen** | `wakePin` → `OUTPUT` + `HIGH`; `intPin` → `INPUT`; speichert den Bus-Zeiger |

Ablauf (**✔ Code**): Pins setzen → `wakeUp()` → `isConnected()`. Schlägt
`wakeUp()` fehl, wird sofort `false` zurückgegeben.

```cpp
Wire.begin();
Wire.setClock(100000);      // siehe Kapitel 05: Clock-Stretching
if (!radar.begin(Wire)) { /* Verkabelung/Adresse prüfen */ }
```

Mehrfach aufrufbar — wird in den Testskripten als Wiederverbindungsversuch
nach mehreren Fehlschlägen genutzt.

## `bool isConnected()`

Sendet eine leere I²C-Transaktion an die Adresse (Address-only-Probe).

| | |
|---|---|
| **Rückgabe** | `true`, wenn das Modul mit ACK antwortet |
| **Blockiert** | nein |
| **Hinweis** | Prüft **nur** die Adressierbarkeit, nicht ob die richtige Firmware läuft oder der Detektor konfiguriert ist |

## `bool wakeUp(uint32_t timeoutMs = XM125_WAKEUP_TIMEOUT_MS)`

Drei Verhaltensvarianten, je nach Verkabelung (**✔ Code**):

| Verkabelung | Verhalten |
|---|---|
| `wakePin >= 0` | Setzt `WAKE_UP` auf `HIGH` |
| `intPin >= 0` | Wartet zusätzlich, bis `MCU_INT` auf `HIGH` geht (Busy-Wait, kein `delay`) |
| beide `-1` | Pollt `isConnected()` alle 20 ms bis zum Timeout |

| | |
|---|---|
| **Rückgabe** | `true`, wenn der Sensor bereit ist bzw. antwortet |
| **Blockiert** | lang (bis `timeoutMs`) |

Im Projekt sind beide Pins unverkabelt — der Aufruf ist dann eine reine
Anlaufabsicherung nach Power-On, **kein** echtes Aufwecken (das Modul schläft
ohne expliziten Host-Eingriff nie ein, siehe Header-Kommentar in
[XM125.h](../XM125.h)).

## `void sleep()`

Zieht `WAKE_UP` auf `LOW`. **Wirkungslos**, wenn `wakePin == -1`.

> ⚠ Auf Hardware ohne verkabelten `wakePin` niemals einen eigenen
> Sleep-Mechanismus (Low-Power-Kommando) ergänzen — ohne Rückweg über den Pin
> wäre das Modul danach nur per Power-Cycle erreichbar. Diese Library sendet
> deshalb bewusst **kein** Sleep-Kommando.

## Register-Getter

```cpp
uint32_t getVersionRaw();       // Reg 0x0000
uint32_t getProtocolStatus();   // Reg 0x0001 — 0 = kein Protokollfehler
uint32_t getMeasureCounter();   // Reg 0x0002 — Messungen seit Modul-Neustart
uint32_t getDetectorStatus();   // Reg 0x0003 — Status-/Fehlerbits
```

| | |
|---|---|
| **Blockiert** | nein (je eine I²C-Lesetransaktion) |
| **Fehlerfall** | ⚠ **Alle vier liefern bei einem I²C-Fehler `0`** — ein fehlgeschlagener Lesevorgang ist vom echten Wert `0` nicht unterscheidbar. Wer das braucht, prüft vorher `isConnected()` oder verwendet `_readReg()` in einer abgeleiteten Klasse |

`getMeasureCounter()` ist nützlich, um zu erkennen, ob das Modul zwischendurch
neu gestartet ist (Zähler springt auf 0) — ein Indiz für Brown-Out/Reset.

## `static bool statusBusy(uint32_t status)` / `statusError(uint32_t status)`

Statische Helfer, prüfen Bit 31 (`BUSY`) bzw. Bit 28 (`DETECTOR_ERROR`) eines
zuvor gelesenen Statuswerts. Klassenunabhängig aufrufbar:

```cpp
uint32_t st = radar.getDetectorStatus();
if (XM125Base::statusBusy(st))  { /* Kommando läuft noch */ }
if (XM125Base::statusError(st)) { /* Detektorfehler */ }
```

## `uint8_t getAddress() const`

Liefert die im Konstruktor gesetzte I²C-Adresse — praktisch für Log-Ausgaben
bei mehreren Sensoren.

## Geschützte Methoden (für eigene Erweiterungen)

| Methode | Beschreibung |
|---|---|
| `bool _writeReg(uint16_t reg, uint32_t value)` | Ein 32-Bit-Register schreiben. 6 Bytes in **einer** Transaktion, Big-Endian |
| `bool _readReg(uint16_t reg, uint32_t& value)` | Ein 32-Bit-Register lesen (Repeated Start, dann 4 Bytes). Bei Fehler bleibt `value` **unverändert** |
| `bool _executeCommand(uint32_t cmd, uint32_t timeoutMs)` | Kommando nach `0x0100` schreiben, 2 ms warten, auf `BUSY = 0` pollen, `DETECTOR_ERROR` prüfen. Enthält den Clock-Stretching-Workaround |
| `bool _waitNotBusy(uint32_t timeoutMs, uint32_t& lastStatus)` | Pollt `DETECTOR_STATUS`, bis `BUSY = 0` oder Timeout. `lastStatus` enthält danach den zuletzt gelesenen Wert |

Details zum Ablauf: [10-Interna-und-Timing.md](10-Interna-und-Timing.md).

---

# 2. `XM125Distance` — Distance-Detector

Datei: [XM125Distance.h](../XM125Distance.h) / [XM125Distance.cpp](../XM125Distance.cpp)
· erbt alles aus `XM125Base`

## Konstruktor

```cpp
explicit XM125Distance(uint8_t i2cAddr = XM125_I2C_ADDR_DEFAULT,
                       int8_t  wakePin = -1,
                       int8_t  intPin  = -1);
```

Identische Parameter wie `XM125Base`; reicht sie nur durch.

## `XM125DistanceConfig config`

Öffentliches Member. Wird **vor** `applyConfig*()` gesetzt; spätere Änderungen
wirken erst nach einem erneuten `applyConfig*()`. Alle Felder:
[04-Konfiguration.md](04-Konfiguration.md).

## `bool writeConfig()`

Schreibt alle 14 Konfigurationsfelder nacheinander in ihre Register, mit 1 ms
Pause dazwischen.

| | |
|---|---|
| **Rückgabe** | `true`, wenn **alle** Writes bestätigt wurden |
| **Blockiert** | kurz (~14 × (Transaktion + 1 ms) ≈ 20–30 ms) |
| **Fehlerverhalten** | fail-fast: bricht beim ersten Fehler ab, merkt sich die Registeradresse |
| **Nebenwirkung** | setzt `_lastFailedConfigRegister` (zu Beginn auf `0xFFFF` zurück) |

Muss normalerweise nicht direkt aufgerufen werden — `applyConfigAndCalibrate()`
und `applyConfiguration()` tun es selbst. Direkt sinnvoll nur, um das
Schreiben von der Kommandoausführung zu trennen (Debugging).

## `uint16_t getLastFailedConfigRegister() const`

Registeradresse des letzten fehlgeschlagenen `writeConfig()`-Writes,
`0xFFFF` = kein Fehler.

**Das ist das entscheidende Unterscheidungsmerkmal bei einem fehlgeschlagenen
`applyConfigAndCalibrate()`:**

| Wert | Deutung |
|---|---|
| `!= 0xFFFF` | I²C-Problem beim Schreiben (Verkabelung, Adresse, Bus) |
| `== 0xFFFF` | Alle Register geschrieben — das Kommando selbst ist fehlgeschlagen (Timeout oder `DETECTOR_ERROR`) |

## `bool applyConfigAndCalibrate(uint32_t timeoutMs = 3000)`

`writeConfig()` + Kommando `1` (`APPLY_CONFIG_AND_CALIBRATE`).

| | |
|---|---|
| **Rückgabe** | `true`, wenn Konfiguration geschrieben, Kommando abgeschlossen und `DETECTOR_ERROR` nicht gesetzt ist |
| **Blockiert** | **lang** — typisch einige 100 ms bis ~2 s |
| **Wann** | einmal nach `begin()`, und nach jeder Änderung an `config` |

Vollständige Beschreibung der internen Vorgänge: [07-Kalibrierung.md](07-Kalibrierung.md).

## `bool applyConfiguration(uint32_t timeoutMs = 3000)`

`writeConfig()` + Kommando `3` (`APPLY_CONFIGURATION`) — **ohne** Kalibrierung.

Nur verwenden, wenn im Modul bereits eine gültige, zum Messfenster passende
Kalibrierung liegt. Nach Änderungen an `startMm`/`endMm`/`maxProfile`
stattdessen `applyConfigAndCalibrate()` nehmen.

## `bool calibrate(uint32_t timeoutMs = 3000)`

Kommando `4` (`CALIBRATE`) — volle Neukalibrierung, **ohne** die
`config`-Register neu zu schreiben. Blockiert lang.

## `bool recalibrate(uint32_t timeoutMs = 3000)`

Kommando `5` (`RECALIBRATE`) — leichtgewichtige Nachkalibrierung, gedacht als
Reaktion auf `getCalibrationNeeded()`. Blockiert lang, aber deutlich kürzer
als `calibrate()`.

## `bool measure(uint32_t timeoutMs = XM125_DEFAULT_MEASURE_TIMEOUT_MS)`

Führt genau **eine** Messung aus und liest alle Ergebnisse ein.

| | |
|---|---|
| **Rückgabe** | `true`, wenn die Messung ohne `MEASURE_ERROR` durchlief |
| **Blockiert** | **lang** — bis `timeoutMs` (Default 1000 ms); die Testskripte verwenden 3000 ms, weil PROFILE5 + CFAR länger brauchen kann |
| **Vorbedingung** | erfolgreiches `applyConfigAndCalibrate()` |

Ablauf (**✔ Code**):

1. Kommando `2` (`MEASURE_DISTANCE`) schreiben (mit Clock-Stretching-Absicherung).
2. Auf `BUSY = 0` pollen; bei Timeout → `false`.
3. `DETECTOR_ERROR` prüfen; gesetzt → `false`.
4. Ergebnisregister `0x0010` lesen → `numDistances`, `nearStartEdge`,
   `calibrationNeeded`, `measureError`, `temperature`.
5. `numDistances` auf maximal 10 begrenzen.
6. **5 ms Pause** (die Firmware braucht noch Zeit, bis die Peak-Register
   gültig sind — ohne die Pause wurden unplausible Werte beobachtet).
7. Je Peak: Distanz- und Stärkeregister lesen, dazwischen je 1 ms.

> ⚠ **Wichtige Feinheiten:** Wenn `measure()` `false` liefert, weil das
> Ergebnisregister nicht gelesen werden konnte, behalten die Getter ihre
> **alten** Werte aus der vorherigen Messung. Und: Lesefehler in Schritt 7
> werden **nicht** geprüft — schlägt ein Peak-Register-Read fehl, bleibt der
> betreffende Wert bei 0. Details: [10-Interna-und-Timing.md](10-Interna-und-Timing.md).

## Zerlegte Messung: `startMeasurement()` / `pollMeasurement()` / `fetchResult()`

`measure()` ist intern genau aus diesen drei Bausteinen zusammengesetzt. Wer
mehrere Sensoren **gleichzeitig** messen lassen will, ruft sie einzeln auf —
dann fällt die Wartezeit einmal für die ganze Gruppe an statt einmal pro
Sensor.

| Methode | Rückgabe | Blockiert | Beschreibung |
|---|---|---|---|
| `bool startMeasurement()` | `true`, wenn das Kommando angekommen ist (inkl. Clock-Stretching-Probe) | **nein** | Schreibt `MEASURE_DISTANCE` und kehrt sofort zurück |
| `bool pollMeasurement(bool& done)` | `false` bei I²C-Fehler **oder** gesetztem `DETECTOR_ERROR` | **nein** | Liest einmal `DETECTOR_STATUS`. Setzt `done = true`, sobald `BUSY` zurückgegangen ist. Läuft die Messung noch, ist die Rückgabe `true` und `done` bleibt `false` |
| `bool fetchResult()` | wie `measure()` (`!measureError`) | kurz (~5 ms + 2 ms je Peak) | Liest Ergebnisregister und Peaks ein. Erst aufrufen, wenn `pollMeasurement()` `done` gemeldet hat |

```cpp
for (auto& r : sensoren) r.startMeasurement();   // alle anstoßen

bool anyPending = true;                          // gemeinsam warten
while (anyPending) {
  anyPending = false;
  for (auto& r : sensoren) {
    bool done = false;
    if (r.pollMeasurement(done) && !done) anyPending = true;
  }
}

for (auto& r : sensoren) r.fetchResult();        // Ergebnisse abholen
```

**Eigenes Timeout nötig:** Die drei Bausteine haben keinen Timeout-Parameter —
die Warteschleife muss selbst abbrechen (im Projekt über
`RADAR_MEASURE_TIMEOUT_MS`). Vorbild:
`systemlib::readRadarGroup()` in [systemlib.cpp](../../systemlib/systemlib.cpp).

**Nebenwirkung:** Sensoren einer Gruppe senden dabei gleichzeitig →
Übersprechen möglich, siehe
[05, Abschnitt 8](05-Fehlerbehandlung-und-Troubleshooting.md#8-übersprechen-zwischen-mehreren-sensoren)
und `RADAR_PARALLEL_GROUP` in [params.h](../../include/params.h).

## Ergebnis-Getter (gültig nach erfolgreichem `measure()`)

| Methode | Typ | Quelle | Bedeutung |
|---|---|---|---|
| `getNumDistances()` | `uint8_t` | Reg `0x0010` Bit 0–3 | Anzahl gefundener Peaks, 0–10 (Bibliothek begrenzt auf 10) |
| `getNearStartEdge()` | `bool` | Bit 8 | Objekt sehr nah an `config.startMm` — Messwert dort unzuverlässig |
| `getCalibrationNeeded()` | `bool` | Bit 9 | Modul fordert Nachkalibrierung an → `recalibrate()` |
| `getMeasureError()` | `bool` | Bit 10 | Messung fehlerhaft |
| `getTemperature()` | `int16_t` | Bit 16–31 | Sensortemperatur in °C, **nur relative Genauigkeit** |
| `getPeakDistanceRaw(idx)` | `uint8_t` | Reg `0x0011 + idx`, oberstes Byte | Roh-"Counts" 0–255. `0`, wenn `idx >= getNumDistances()` |
| `getPeakDistanceMm(idx)` | `uint32_t` | s.o. | ⚠ **aktuell identisch zum Rohwert — keine echten Millimeter**, siehe [07-Kalibrierung.md Teil B](07-Kalibrierung.md#teil-b--kalibrierung-der-peak-rohwerte-counts--mm) |
| `getPeakDistanceReg(idx)` | `uint32_t` | Reg `0x0011 + idx`, **ungekürzt** | Das vollständige 32-Bit-Registerwort. Für die Kalibriermessung protokollieren, **nicht** den 8-Bit-Wert — welche Bits die Distanz tragen, ist die offene Frage. `0`, wenn `idx >= getNumDistances()` |
| `getPeakStrength(idx)` | `int32_t` | Reg `0x001B + idx` | Signalstärke (1000×-Einheit, RSS-intern). `0`, wenn `idx >= getNumDistances()` |

Alle Getter sind `const`, machen **keinen** I²C-Zugriff und blockieren nicht —
sie liefern den Zustand der letzten `measure()`.

Die Peaks sind gemäß `config.peakSorting` sortiert: bei `CLOSEST` ist Index 0
der nächstgelegene, bei `STRONGEST` der stärkste Reflektor.

---

# 3. `XM125Presence` — Presence-Detector

Datei: [XM125Presence.h](../XM125Presence.h) / [XM125Presence.cpp](../XM125Presence.cpp)
· erbt alles aus `XM125Base`

## Konstruktor & `config`

Wie bei `XM125Distance`, mit `XM125PresenceConfig config`. Achtung: Beim
Presence-Detektor ist `config` **nach** `applyConfiguration()` nicht mehr
wirksam änderbar — Laufzeitänderungen erfordern
`stop()` → `applyConfiguration()` → `start()`.

## `bool writeConfig()` / `uint16_t getLastFailedConfigRegister() const`

Identisches Verhalten wie beim Distance-Detektor, nur mit 23 statt 14
Registern (**✔ Code**) — entsprechend ~35–50 ms Laufzeit.

## `bool applyConfiguration(uint32_t timeoutMs = 3000)`

`writeConfig()` + Kommando `1` (`APPLY_CONFIGURATION`). Schließt die
**Sensor-Kalibrierung** mit ein (Statusbit `SENSOR_CALIBRATE_OK`, Bit 3).
Blockiert lang.

## `bool start(uint32_t timeoutMs = 3000)`

Kommando `2` (`START_DETECTOR`). Danach misst das Modul **kontinuierlich
selbstständig**; `update()` holt nur noch Ergebnisse ab.

> Nach `start()` mindestens 2–3 s Einschwingzeit einplanen, bevor die
> Detektionsflags ausgewertet werden (Filterzeitkonstanten, siehe
> [07-Kalibrierung.md A.9](07-Kalibrierung.md#a9-presence-detektor-kalibrierung-und-einschwingen)).

## `bool stop(uint32_t timeoutMs = 3000)`

Kommando `3` (`STOP_DETECTOR`). Stoppt die laufende Messung.

## `bool update()`

Liest die vier Ergebnisregister aus.

| | |
|---|---|
| **Rückgabe** | `true`, wenn **Register `0x0010`** gelesen werden konnte |
| **Blockiert** | nein — sendet **kein** Kommando, nur 4 Leseoperationen |
| **Vorbedingung** | vorheriges `start()` |

> ⚠ Der Rückgabewert bezieht sich nur auf das erste Register. Schlagen die
> Lesevorgänge für Distanz/Intra-Score/Inter-Score fehl, liefert `update()`
> trotzdem `true` und die betroffenen Werte behalten ihren **vorherigen**
> Stand (**✔ Code**, [XM125Presence.cpp](../XM125Presence.cpp)).

Aufrufrate: sinnvoll etwa im Takt von `config.frameRateMilliHz` (Default
12000 mHz = 12 Hz → alle ~83 ms). Häufigeres Pollen liefert dieselben Werte
mehrfach und belastet nur den Bus; das Testskript verwendet `delay(100)`.

## Ergebnis-Getter (gültig nach `update()`)

| Methode | Typ | Quelle | Bedeutung |
|---|---|---|---|
| `isPresenceDetected()` | `bool` | Reg `0x0010` Bit 0 | Anwesenheit im aktuellen Frame |
| `isPresenceDetectedSticky()` | `bool` | Bit 1 | bleibt nach der letzten Erkennung noch `interFramePresenceTimeoutS` Sekunden gesetzt — für Anwendungen, die nicht bei jedem Frame flackern sollen |
| `hasDetectorError()` | `bool` | Bit 15 | Detektorfehler → `stop()` + `start()` |
| `getTemperature()` | `int16_t` | Bit 16–31 | °C, nur relative Genauigkeit |
| `getPresenceDistanceMm()` | `uint32_t` | Reg `0x0011` | Distanz der Detektion in mm — hier liefert die Firmware **echte Millimeter** (anders als beim Distance-Detektor). Nur aussagekräftig, wenn `isPresenceDetected()` |
| `getIntraPresenceScore()` | `uint32_t` | Reg `0x0012` | Score der schnellen Bewegungserkennung; über `intraDetectionThreshold` entsteht daraus die Detektion |
| `getInterPresenceScore()` | `uint32_t` | Reg `0x0013` | Score der langsamen Bewegungserkennung; Schwelle `interDetectionThreshold` |

Die Scores sind auch dann nützlich, wenn (noch) keine Detektion vorliegt: Sie
zeigen, wie weit der aktuelle Wert von der Schwelle entfernt ist, und sind
damit die Grundlage, um `intraDetectionThreshold` /
`interDetectionThreshold` sinnvoll einzustellen.

---

# 4. Übersicht: was blockiert wie lange

| Methode | Typische Dauer | Worst Case |
|---|---|---|
| `isConnected()` | < 1 ms | < 1 ms |
| `begin()` | wenige ms | 1500 ms (Wake-Timeout) |
| `writeConfig()` (Distance) | ~20–30 ms | ~30 ms |
| `writeConfig()` (Presence) | ~35–50 ms | ~50 ms |
| `applyConfigAndCalibrate()` | 300–2000 ms | 3000 ms + Config-Write |
| `calibrate()` | 300–2000 ms | 3000 ms |
| `recalibrate()` | < 500 ms | 3000 ms |
| `measure()` | 20–200 ms (je nach `endMm`, Profil, `signalQuality`) | `timeoutMs` + ~5 ms + 2 ms je Peak |
| `startMeasurement()` | < 1 ms | < 1 ms |
| `pollMeasurement()` | < 1 ms | < 1 ms |
| `fetchResult()` | ~5 ms + 2 ms je Peak | dito |
| `start()` / `stop()` (Presence) | wenige ms | 3000 ms |
| `update()` (Presence) | < 2 ms | < 2 ms |
| alle `get*()`-Ergebnis-Getter | 0 (nur Speicherzugriff) | 0 |

**Für Regelschleifen wichtig:** Alle Kommando-Methoden mit `timeoutMs`-Parameter
blockieren bis zum Ergebnis. Nicht blockierend ist nur die zerlegte Messung
(`startMeasurement()` / `pollMeasurement()` / `fetchResult()`) — und auch die
verlangt, dass die Anwendung die Wartezeit selbst überbrückt und ein eigenes
Timeout mitbringt. Eine vollständige State-Machine wie bei
[SRF08](../../SRF08/SRF08.md) gibt es nicht; die Kalibrierkommandos bleiben
in jedem Fall blockierend und gehören deshalb in `setup()` bzw. in klar
definierte Pausen.
