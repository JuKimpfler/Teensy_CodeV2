# Interna, Timing & Blockierverhalten

Dieses Kapitel beschreibt, was die Library **tatsächlich** auf dem Bus tut —
nötig, wenn man Timing-Probleme sucht, den Code erweitert oder das Verhalten
mit einem Logikanalysator nachvollziehen will. Alle Angaben hier sind aus
[XM125.cpp](../XM125.cpp), [XM125Distance.cpp](../XM125Distance.cpp) und
[XM125Presence.cpp](../XM125Presence.cpp) belegt.

---

## 1. Byte-Layout auf dem I²C-Bus

### Schreiben — `_writeReg(reg, value)`

Eine einzige Transaktion mit 6 Datenbytes, alles Big-Endian (MSB zuerst):

```
S | ADDR+W | reg[15:8] | reg[7:0] | val[31:24] | val[23:16] | val[15:8] | val[7:0] | P
```

Beispiel `_writeReg(0x0040, 200)` (Messbereichsstart 200 mm):

```
S | 0xA2 | 0x00 0x40 | 0x00 0x00 0x00 0xC8 | P        (ADDR 0x51 << 1 = 0xA2)
```

Rückgabe ist `true`, wenn `Wire.endTransmission() == 0` — also alle Bytes
mit ACK quittiert wurden.

### Lesen — `_readReg(reg, value)`

Zwei Phasen, verbunden über **Repeated Start** (kein STOP dazwischen):

```
S | ADDR+W | reg[15:8] | reg[7:0] | Sr | ADDR+R | b3 | b2 | b1 | b0 | P
```

```cpp
value = (b3 << 24) | (b2 << 16) | (b1 << 8) | b0;   // Big-Endian
```

Die erste Phase endet mit `endTransmission(false)` — genau das erzeugt den
Repeated Start. Ein STOP dazwischen würde die Registeradresse verwerfen.

**Fehlerfälle** (jeweils `return false`):

| Bedingung | Bedeutung |
|---|---|
| `_wire == nullptr` | `begin()` wurde nie aufgerufen |
| `endTransmission(false) != 0` | Modul hat die Adresse/Registeradresse nicht quittiert |
| `requestFrom(...) != 4` | Modul hat weniger als 4 Bytes geliefert |
| `available() < 4` | dito, Absicherung |

> Wichtig: Bei einem Fehler bleibt der übergebene `value` **unverändert**.
> Die Getter in `XM125Base` initialisieren ihn deshalb vorher mit `0` — sie
> liefern im Fehlerfall also `0` und **nicht** den alten Wert.

## 2. Kommandoausführung — `_executeCommand()`

```
_writeReg(0x0100, cmd)
   └── false? → DETECTOR_STATUS lesen
                 ├── BUSY gesetzt → Kommando kam an, weiter
                 └── sonst        → return false
delay(2)                              // der Modul-MCU Zeit geben, BUSY zu setzen
_waitNotBusy(timeoutMs, status)
   └── false? → return false          // Timeout oder Lesefehler
return !statusError(status)           // Bit 28
```

### Warum der Clock-Stretching-Workaround nötig ist

Die interne STM32-MCU des XM125 beginnt unmittelbar nach Empfang eines
Kommandos mit der Verarbeitung und streckt dabei den I²C-Takt. Manche
Host-Treiber melden das als Fehler/Timeout auf der **laufenden** Transaktion —
obwohl das Kommando angekommen ist. Deshalb wird nach einem gemeldeten
Schreibfehler sofort der Status gegengeprüft: Steht `BUSY`, wurde das Kommando
offensichtlich angenommen.

Derselbe Workaround steckt auch direkt in `XM125Distance::measure()`, weil
`measure()` das Kommando selbst schreibt statt `_executeCommand()` zu nutzen
(es braucht danach noch die Ergebnisregister).

### `_waitNotBusy()` — Polling-Verhalten

```cpp
do {
    if (!_readReg(0x0003, lastStatus)) return false;   // Lesefehler → sofort false
    if (!statusBusy(lastStatus))       return true;
} while (millis() - start < timeoutMs);
return false;                                          // Timeout
```

Zu beachten:

- **Kein `delay()` zwischen den Abfragen.** Es wird so schnell gepollt, wie
  der I²C-Bus es zulässt — bei 100 kHz rund **1400 Abfragen pro Sekunde**.
  Während einer 2-Sekunden-Kalibrierung sind das ~2800 Lesetransaktionen.
  Der Bus ist in dieser Zeit vollständig belegt; andere I²C-Teilnehmer am
  selben Bus kommen nicht dran.
- **Ein einzelner Lesefehler bricht sofort ab** — es gibt keinen Retry. Ein
  kurzer Störimpuls auf dem Bus kann deshalb ein an sich erfolgreiches
  Kommando als Fehlschlag erscheinen lassen.
- Die `do`-`while`-Konstruktion prüft **mindestens einmal**, auch bei
  `timeoutMs = 0`.
- Die Zeitprüfung `millis() - start < timeoutMs` ist mit vorzeichenloser
  Arithmetik formuliert und daher **überlauffest** (nach 49,7 Tagen Laufzeit
  korrekt).

## 3. `measure()` im Detail

`measure()` ist aus drei einzeln aufrufbaren Bausteinen zusammengesetzt
(**✔ Code**):

```cpp
bool XM125Distance::measure(uint32_t timeoutMs) {
    if (!startMeasurement()) return false;          // Schritt 1
    uint32_t status = 0;
    if (!_waitNotBusy(timeoutMs, status)) return false;   // Schritt 2
    if (statusError(status)) return false;                // Schritt 3
    return fetchResult();                                 // Schritte 4-7
}
```

| Schritt | Aktion | Zeit | Baustein |
|---|---|---|---|
| 1 | Kommando `MEASURE_DISTANCE` schreiben (+ Clock-Stretching-Probe) | ~0,7 ms | `startMeasurement()` |
| 2 | `_waitNotBusy(timeoutMs)` — bzw. wiederholtes `pollMeasurement()` | Messdauer des Moduls | — |
| 3 | `DETECTOR_ERROR` prüfen | — | (in `pollMeasurement()` enthalten) |
| 4 | Ergebnisregister `0x0010` lesen | ~0,7 ms | `fetchResult()` |
| 5 | Flags/Temperatur dekodieren, `numDistances` auf 10 begrenzen | — | `fetchResult()` |
| 6 | **`delay(5)`** | 5 ms | `fetchResult()` |
| 7 | je Peak: Distanz lesen, `delay(1)`, Stärke lesen, `delay(1)` | ~3,4 ms je Peak | `fetchResult()` |

`pollMeasurement(done)` ersetzt Schritt 2+3 durch **eine einzelne**
Statusabfrage: Rückgabe `false` bei I²C-Fehler oder `DETECTOR_ERROR`,
`done = true`, sobald `BUSY` zurückgegangen ist. Damit lässt sich die
Wartezeit über mehrere Sensoren überlappen — der Grund, warum es die
Zerlegung überhaupt gibt (siehe Abschnitt 5).

In Schritt 7 wird seit der Erweiterung um `getPeakDistanceReg()` **beides**
gespeichert: das ungekürzte Registerwort in `_peakDistanceReg[i]` und das
oberste Byte in `_peakDistanceRaw[i]`.

**Zu Schritt 6:** Die Pause ist empirisch nötig. Das Ergebnisregister ist
bereits gültig (Peak-Anzahl, Temperatur, Flags), während die Firmware die
Peak-Detailregister offenbar noch schreibt — ohne die Pause wurden
unplausible Werte beobachtet (**✔ Code**-Kommentar). Wer die Messrate
optimieren will, sollte diese 5 ms nicht blind entfernen, sondern gegen die
Plausibilität der Peaks testen.

**Zu Schritt 7 — bekannte Schwäche:** Die Rückgabewerte von `_readReg()`
werden hier **nicht** ausgewertet:

```cpp
uint32_t dist = 0, strength = 0;
_readReg(XM125D_REG_PEAK0_DISTANCE + i, dist);   // Rückgabe ignoriert
_readReg(XM125D_REG_PEAK0_STRENGTH + i, strength);
```

Schlägt ein Lesevorgang fehl, bleiben `dist`/`strength` bei `0` und der Peak
erscheint als "Distanz 0, Stärke 0" — `measure()` liefert trotzdem `true`.
Anwendungen sollten Peaks mit Stärke `0` deshalb als verdächtig behandeln.

**`numDistances`-Begrenzung:** Die Maske `0x0F` lässt Werte bis 15 zu,
`XM125D_MAX_PEAKS` ist aber 10 — deshalb die Klemmung
`if (_numDistances > XM125D_MAX_PEAKS) _numDistances = XM125D_MAX_PEAKS;`.
Das schützt die Arrays vor Überlauf, falls die Firmware einen unerwarteten
Wert liefert.

## 4. `XM125Presence::update()` im Detail

```cpp
if (!_readReg(0x0010, result)) return false;     // nur DIESER Fehler wird gemeldet
// Flags + Temperatur dekodieren
if (_readReg(0x0011, v)) _presenceDistanceMm = v;   // Fehler → alter Wert bleibt
if (_readReg(0x0012, v)) _intraScore = v;
if (_readReg(0x0013, v)) _interScore = v;
return true;
```

Vier Lesetransaktionen, keine `delay()`s, kein Kommando — der Aufruf kostet
bei 100 kHz rund **3 ms**. Die drei Detailregister werden "best effort"
gelesen: Bei einem Fehler behalten die Getter den **vorherigen** Wert, ohne
dass das nach außen sichtbar wird.

## 5. Zeitbudget bei 100 kHz Bustakt

Grundlage: 9 Bit je Byte (8 Datenbits + ACK), 100 kbit/s → **90 µs pro Byte**.

| Vorgang | Bytes auf dem Bus | Reine Buszeit |
|---|---|---|
| Register schreiben | 1 Adresse + 6 Daten = 7 | ~0,63 ms |
| Register lesen | 3 (Schreibphase) + 5 (Lesephase) = 8 | ~0,72 ms |
| `isConnected()` | 1 | ~0,09 ms |
| `writeConfig()` Distance (14 Register + 14 × `delay(1)`) | 98 | **~23 ms** |
| `writeConfig()` Presence (23 Register + 23 × `delay(1)`) | 161 | **~38 ms** |
| `update()` Presence (4 Reads) | 32 | **~3 ms** |
| `measure()` Nachlauf: 5 ms + 4 Peaks × (2 Reads + 2 ms) | | **~19 ms** bei 4 Peaks |

Bei **400 kHz** wären alle Buszeiten viertelt — die Testskripte verwenden
aber bewusst 100 kHz, weil das dem Modul mehr Toleranz beim Clock-Stretching
lässt (siehe [05-Fehlerbehandlung](05-Fehlerbehandlung-und-Troubleshooting.md#1-i²c-schreibfehler-durch-clock-stretching)).

### Konsequenz für mehrere Sensoren

Vier Distance-Sensoren **nacheinander** per `measure()` abzufragen kostet
`4 × (Messdauer + Nachlauf)`. Bei ~50 ms Messdauer je Sensor sind das ~280 ms
pro Runde, also gut 3 Hz Positionsrate.

Die Messdauer besteht fast vollständig aus Warten — deshalb überlappt
`systemlib::readRadarGroup()` sie über die zerlegte Mess-API:

| `RADAR_PARALLEL_GROUP` | Gruppen | Wartezeit pro Runde | Übersprechen |
|---|---|---|---|
| 1 | 4 × einzeln | 4 × Messdauer | ausgeschlossen |
| **2** (Projektstandard) | {Vo, Hi}, {Li, Re} | 2 × Messdauer | gering — die Paare blicken entgegengesetzt |
| 4 | alle zusammen | 1 × Messdauer | am größten |

Die Gruppen entstehen über die Schrittweite `numGroups`: Bei 2er-Gruppen
messen jeweils die **entgegengesetzt** blickenden Sensoren zusammen, nicht die
benachbarten. Welcher Wert tragbar ist, misst
[test/XM125_WallCalibration.cpp](../../../test/XM125_WallCalibration.cpp)
im Modus `i`.

Weitere Stellschrauben, wenn es noch schneller sein muss:

- `endMm` reduzieren (kürzerer Messbereich = kürzere Messung),
- `signalQuality` senken — kostet aber Reichweitenreserve am Bereichsende,
- niedrigeres `maxProfile`,
- Sensoren auf **verschiedene** `Wire`-Busse verteilen (Teensy 4.0 hat
  `Wire`, `Wire1`, `Wire2`) und abwechselnd abfragen.

## 6. Blockierverhalten & Echtzeit

- **Alles ist blockierend.** `delay()` und Polling-Schleifen laufen ohne
  `yield()`-Aufrufe; während `applyConfigAndCalibrate()` steht die
  Hauptschleife bis zu 3 s still. Für eine Roboter-Regelschleife heißt das:
  **Kalibrierung nur in `setup()` bzw. in klar definierten Pausen.**
- **Nicht ISR-tauglich.** Die Methoden nutzen `Wire` und `millis()`/`delay()`
  — Aufrufe aus einem Interrupt-Handler sind nicht zulässig.
- **Keine Bus-Arbitrierung zwischen Instanzen.** Mehrere `XM125*`-Objekte am
  selben `TwoWire` teilen sich den Bus ohne Sperre; da alle Aufrufe
  synchron aus der Hauptschleife kommen, genügt das — nebenläufige Nutzung
  (z.B. aus einem Timer-Callback) würde jedoch kollidieren.
- **Kein Retry, keine Prüfsumme.** Das Registerprotokoll hat keine CRC; die
  Library wiederholt fehlgeschlagene Transaktionen nicht. Robustheit wird
  über die Fehlerzähler in der Anwendung erreicht
  (`MAX_CONSECUTIVE_FAILURES`-Muster in allen Testskripten).

## 7. Speicherbedarf

| Element | Größe |
|---|---|
| `XM125Base` (Bus-Zeiger, Adresse, 2 Pins) | ~8 Byte |
| `XM125DistanceConfig` (12 × `uint32_t` + 2 × `bool`) | ~52 Byte |
| Peak-Arrays (10 × `uint8_t` + 10 × `uint32_t` + 10 × `int32_t`) | 90 Byte |
| Ergebnis-Flags/Temperatur/`_lastFailedConfigRegister` | ~8 Byte |
| **`XM125Distance` gesamt** | **≈ 160 Byte je Instanz** |
| `XM125PresenceConfig` (16 × `uint32_t` + 7 × `bool`) | ~72 Byte |
| **`XM125Presence` gesamt** | **≈ 105 Byte je Instanz** |

(Genaue Werte hängen vom Padding des Compilers ab.) Vier
`XM125Distance`-Instanzen belegen also rund **0,6 kB** RAM — auf einem Teensy
4.0 mit 1 MB RAM unkritisch. Es wird **kein** dynamischer Speicher
(`new`/`malloc`) verwendet, alle Objekte sind statisch anlegbar.

## 8. Bekannte Eigenheiten in Stichpunkten

| # | Eigenheit | Auswirkung |
|---|---|---|
| 1 | Register-Getter liefern bei I²C-Fehler `0` | Fehler nicht von echtem `0` unterscheidbar |
| 2 | Peak-Lesefehler in `measure()` werden ignoriert | Peak erscheint als `0`/`0` |
| 3 | `update()` meldet nur Fehler des ersten Registers | Alte Distanz-/Score-Werte bleiben unbemerkt stehen |
| 4 | `_waitNotBusy()` bricht bei einem einzelnen Lesefehler ab | Störimpuls → Kommando gilt als fehlgeschlagen, obwohl es lief |
| 5 | Kein `delay` im Busy-Polling | maximale Buslast während Kalibrierung |
| 6 | `wakeUp()` mit `intPin` ist ein reiner Busy-Wait | blockiert bis 1500 ms ohne `yield()` |
| 7 | `getPeakDistanceMm()` ist ein Platzhalter | **keine echten Millimeter**, siehe [07-Kalibrierung.md](07-Kalibrierung.md) |
| 8 | `sleep()` wirkt nur mit verkabeltem `wakePin` | ohne Pin folgenlos (bewusst so) |
| 9 | `begin()` ruft `Wire.begin()` **nicht** auf | muss die Anwendung tun |
| 10 | Firmwaretyp wird nicht erkannt | falsche Klasse → `PROTOCOL_STATUS != 0` |
| 11 | `pollMeasurement()` hat **keinen** Timeout | die Warteschleife der Anwendung muss selbst abbrechen (Vorbild: `RADAR_MEASURE_TIMEOUT_MS` in `readRadarGroup()`) |
| 12 | `fetchResult()` prüft nicht, ob überhaupt eine Messung lief | zu früh aufgerufen liefert es die Register der **vorherigen** Messung |

Punkte 1–4 sind bewusste Vereinfachungen, keine Bugs im Sinne von "falsches
Verhalten" — sie sind hier dokumentiert, damit man sie beim Debuggen nicht
für Sensorfehler hält.
