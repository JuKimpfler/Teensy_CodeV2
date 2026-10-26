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

Derselbe Workaround steckt auch direkt in `XM125Distance::startMeasurement()`,
weil das Messkommando nicht über `_executeCommand()` läuft (dort würde auf das
Ergebnis gewartet — genau das soll der Messbetrieb nicht).

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

## 3. Der Messbetrieb im Detail — die Zustandsmaschine

Der Distance-Detektor misst über eine nicht blockierende Zustandsmaschine.
**Nirgends steht ein `delay()`**; die zwei nötigen Pausen werden über
`micros()`-Zeitstempel ausgesessen, nicht verschlafen.

```
IDLE ──startMeasurement()──> RUNNING ──BUSY weg, Result gelesen──> SETTLING
                                │                                     │
                                │ Fehler/Timeout          Pause um,   │
                                ▼                         Peaks da    ▼
                             FAILED <────Fehler────────────────────  READY
```

Weitergetrieben wird sie ausschließlich von `isMeasuring()` — ein Aufruf macht
genau **einen** Schritt und höchstens **einen** Buszugriff:

| Zustand | Was `isMeasuring()` tut | Buszeit | Ergebnis |
|---|---|---|---|
| `IDLE`/`READY`/`FAILED` | nichts | 0 | `false` |
| `RUNNING`, < 2 ms seit Start | nichts (siehe unten) | 0 | `true` |
| `RUNNING`, BUSY gesetzt | `DETECTOR_STATUS` lesen, Timeout prüfen | ~0,7 ms | `true` |
| `RUNNING`, BUSY weg | Status lesen **+** Ergebnisregister `0x0010` lesen | ~1,4 ms | `true` (→ `SETTLING`) bzw. `false` bei 0 Peaks |
| `SETTLING`, < 5 ms | nichts | 0 | `true` |
| `SETTLING`, Pause um | alle zu holenden Peaks lesen (2 Register je Peak) | ~1,4 ms je Peak | `false` (→ `READY`) |

### Die beiden Pausen

**`XM125D_BUSY_SETTLE_US` = 2000 µs — vor der ersten BUSY-Abfrage.**
Die interne MCU setzt `BUSY` nicht in derselben Mikrosekunde, in der das
Kommando ankommt. Wer sofort fragt, sieht `BUSY = 0` und würde das
Ergebnisregister der **vorherigen** Messung als neu ausgeben. (In der früheren
blockierenden Fassung war das ein `delay(2)` in `_executeCommand()`.)

**`XM125D_PEAK_SETTLE_US` = 5000 µs — zwischen Ergebnis- und Peak-Registern.**
Empirisch nötig: Das Ergebnisregister ist gültig, sobald `BUSY` zurückgeht
(Peak-Anzahl, Temperatur, Flags), während die Firmware die Peak-Detailregister
offenbar noch schreibt — ohne die Pause wurden unplausible Werte beobachtet.
Wer die Messrate optimieren will, sollte diese 5 ms nicht blind kürzen,
sondern gegen die Plausibilität der Peaks testen.

Der frühere zusätzliche `delay(1)` zwischen den einzelnen Peak-Lesevorgängen
ist entfallen: Die Peak-Register werden jetzt in einem Rutsch gelesen, nachdem
die Settle-Pause abgelaufen ist. Im Mehrsensorbetrieb liegt zwischen
Ergebnisregister und Peak-Lesen ohnehin mindestens ein kompletter
Anwendungszyklus.

### Peaks: nur holen, was gebraucht wird

`setMaxPeaksToFetch(n)` begrenzt, wie viele Peaks überhaupt vom Sensor geholt
werden (jeder kostet zwei Registerlesezugriffe ≈ 1,4 ms). Das Projekt setzt
`RADAR_PEAKS_TO_FETCH = 1`, weil `systemlib` nur Peak 0 auswertet — statt bis
zu 20 Lesezugriffen bleiben zwei. `numDistances` bleibt die echte Anzahl; die
Werte der übersprungenen Peaks werden auf `0` gesetzt, damit dort nicht die
Werte der vorherigen Messung stehen bleiben. Meldet die Messung `0` Peaks,
entfällt sowohl das Peak-Lesen als auch die 5-ms-Pause komplett.

Gespeichert wird je Peak **beides**: das ungekürzte Registerwort in
`peakDistanceReg[i]` und das oberste Byte in `peakDistanceRaw[i]`.

### Fehlerbehandlung

Jeder Lesevorgang wird ausgewertet — schlägt einer fehl, geht die Maschine auf
`FAILED` und `result.valid` wird `false`. Die Zahlenwerte bleiben dabei auf dem
letzten guten Stand stehen (der Aufrufer entscheidet über `valid` bzw.
`getResultAgeMs()`, ob er sie noch verwenden will). Ein Peak, der als
"Distanz 0, Stärke 0" durchrutscht, ohne dass `valid` fällt, ist damit nicht
mehr möglich.

**`numDistances`-Begrenzung:** Die Maske `0x0F` lässt Werte bis 15 zu,
`XM125D_MAX_PEAKS` ist aber 10 — deshalb die Klemmung auf 10. Das schützt die
Arrays vor Überlauf, falls die Firmware einen unerwarteten Wert liefert.

**Timeout:** `setMeasureTimeout(ms)` (Default 1000 ms, Projekt: 3000 ms) wird
in `RUNNING` gegen die seit `startMeasurement()` vergangene Zeit geprüft. Die
Anwendung braucht **keine** eigene Abbruchlogik mehr.

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
| `startMeasurement()` (1 Write) | 7 | ~0,63 ms |
| `isMeasuring()` in `RUNNING` (1 Read) | 8 | ~0,72 ms |
| Peaks holen (2 Reads je Peak) | 16 je Peak | ~1,4 ms je Peak |

Bei **400 kHz** wären alle Buszeiten viertelt — die Testskripte verwenden
aber bewusst 100 kHz, weil das dem Modul mehr Toleranz beim Clock-Stretching
lässt (siehe [05-Fehlerbehandlung](05-Fehlerbehandlung-und-Troubleshooting.md#1-i²c-schreibfehler-durch-clock-stretching)).

### Konsequenz für mehrere Sensoren

Vier Distance-Sensoren **nacheinander** blockierend abzufragen kostet
`4 × (Messdauer + Nachlauf)`. Bei ~50 ms Messdauer je Sensor sind das ~280 ms
pro Runde, also gut 3 Hz Positionsrate — und die Hauptschleife steht in dieser
Zeit still.

Der Messbetrieb wartet deshalb überhaupt nicht mehr:
`systemlib::readRadar()` stößt eine Runde bei **allen** Sensoren gleichzeitig
an und sieht in den folgenden Zyklen nur noch nach.

| Variante | Zeit im Zyklus | Rundendauer | Übersprechen |
|---|---|---|---|
| nacheinander blockierend (früher) | volle Messdauer × Sensoranzahl | 4 × Messdauer | ausgeschlossen |
| gleichzeitig blockierend | volle Messdauer | 1 × Messdauer | möglich |
| **gleichzeitig, nicht blockierend (heute)** | **~1 Buszugriff je Sensor (< 1 ms)** | 1 × Messdauer | möglich |

Die Rundendauer ändert sich zwischen den beiden Gleichzeitig-Varianten nicht —
was sich ändert, ist die Zeit, die der Zyklus dafür **stillsteht**: statt der
vollen Messdauer nur noch ein Statusregister je Sensor. Bei drei Sensoren sind
das rund 2 ms pro Zyklus statt ~50 ms.

Ob der Gleichzeitigbetrieb Trefferquote kostet, misst
[XM125_Calib_Crosstalk.cpp](../examples/XM125_Calib_Crosstalk.cpp) im Modus `g`
(nacheinander gegen gleichzeitig).

Weitere Stellschrauben, wenn es noch schneller sein muss:

- `setMaxPeaksToFetch(n)` — nur die Peaks holen, die ausgewertet werden
  (Projekt: 1 statt 10, spart bis zu 18 Registerlesezugriffe je Messung),

- `endMm` reduzieren (kürzerer Messbereich = kürzere Messung),
- `signalQuality` senken — kostet aber Reichweitenreserve am Bereichsende,
- niedrigeres `maxProfile`,
- Sensoren auf **verschiedene** `Wire`-Busse verteilen (Teensy 4.0 hat
  `Wire`, `Wire1`, `Wire2`) und abwechselnd abfragen.

## 6. Blockierverhalten & Echtzeit

- **Der Messbetrieb blockiert nicht.** `startMeasurement()`, `isMeasuring()`
  und `getLastResult()` kehren immer sofort zurück; die Zustandsmaschine
  enthält kein `delay()` und keine Warteschleife.
- **Die Einrichtung blockiert weiterhin.** `begin()`, `writeConfig()`,
  `applyConfig*()`, `calibrate()`, `recalibrate()` — und das ausdrücklich für
  Testskripte gedachte `measureBlocking()`. Während
  `applyConfigAndCalibrate()` steht die Hauptschleife bis zu 3 s still. Für
  eine Roboter-Regelschleife heißt das: **Kalibrierung nur in `setup()` bzw.
  in klar definierten Pausen** — und ein Wiederanlaufversuch für einen toten
  Sensor gedrosselt, siehe `RADAR_REINIT_INTERVAL_MS` in
  [params.h](../../include/params.h).
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
| `XM125DistanceResult`: Peak-Arrays (10 × `uint8_t` + 10 × `uint32_t` + 10 × `int32_t`) | 90 Byte |
| `XM125DistanceResult`: Flags, Temperatur, Zeitstempel, Dauer | ~14 Byte |
| Zustandsmaschine (Zustand, Timeout, Zeitstempel, Peak-Zähler) | ~16 Byte |
| **`XM125Distance` gesamt** | **≈ 185 Byte je Instanz** |
| `XM125PresenceConfig` (16 × `uint32_t` + 7 × `bool`) | ~72 Byte |
| **`XM125Presence` gesamt** | **≈ 105 Byte je Instanz** |

(Genaue Werte hängen vom Padding des Compilers ab.) Vier
`XM125Distance`-Instanzen belegen also rund **0,6 kB** RAM — auf einem Teensy
4.0 mit 1 MB RAM unkritisch. Es wird **kein** dynamischer Speicher
(`new`/`malloc`) verwendet, alle Objekte sind statisch anlegbar.

## 8. Bekannte Eigenheiten in Stichpunkten

| # | Eigenheit | Auswirkung |
|---|---|---|
| 1 | Register-Getter in `XM125Base` liefern bei I²C-Fehler `0` | Fehler nicht von echtem `0` unterscheidbar |
| 2 | Nach einem Fehlversuch bleiben die Zahlenwerte im Ergebnis stehen | nur `valid`/`timestampMs` zeigen, dass sie alt sind |
| 3 | `update()` (Presence) meldet nur Fehler des ersten Registers | Alte Distanz-/Score-Werte bleiben unbemerkt stehen |
| 4 | `_waitNotBusy()` bricht bei einem einzelnen Lesefehler ab | Störimpuls → Kalibrierkommando gilt als fehlgeschlagen, obwohl es lief |
| 5 | Kein `delay` im Busy-Polling der Kalibrierkommandos | maximale Buslast während Kalibrierung |
| 6 | `wakeUp()` mit `intPin` ist ein reiner Busy-Wait | blockiert bis 1500 ms ohne `yield()` |
| 7 | `getPeakDistanceMm()` ist ein Platzhalter | **keine echten Millimeter**, siehe [07-Kalibrierung.md](07-Kalibrierung.md) |
| 8 | `sleep()` wirkt nur mit verkabeltem `wakePin` | ohne Pin folgenlos (bewusst so) |
| 9 | `begin()` ruft `Wire.begin()` **nicht** auf | muss die Anwendung tun |
| 10 | Firmwaretyp wird nicht erkannt | falsche Klasse → `PROTOCOL_STATUS != 0` |
| 11 | Nach einem Timeout misst der Sensor evtl. noch weiter | das nächste `startMeasurement()` kann auf einen `BUSY`-Sensor treffen; die Anwendung zählt Fehlschläge (`RADAR_MAX_CONSEC_FAILURES`) und initialisiert dann neu |
| 12 | `getLastResult()` prüft nicht, ob überhaupt schon gemessen wurde | vor der ersten Messung ist alles `0` und `valid == false` |

Punkte 1–5 sind bewusste Vereinfachungen, keine Bugs im Sinne von "falsches
Verhalten" — sie sind hier dokumentiert, damit man sie beim Debuggen nicht
für Sensorfehler hält.
