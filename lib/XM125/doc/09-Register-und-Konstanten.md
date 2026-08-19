# Registerkarte & alle Konstanten

Vollständige Auflistung **jeder** in der Library definierten Konstante, mit
Zahlenwert und Fundstelle. Gedacht zum Nachschlagen beim Debuggen — z.B. wenn
im Serial-Monitor ein Statuswert `0x90000004` auftaucht und man wissen will,
welche Bits das sind.

---

## 1. Adressraum-Überblick

| Bereich | Inhalt | Zugriff |
|---|---|---|
| `0x0000` – `0x0003` | Gemeinsame Register (Version, Protokollstatus, Messzähler, Detektorstatus) | R |
| `0x0010` – `0x0024` | Ergebnisregister (Bedeutung je nach Firmware unterschiedlich!) | R |
| `0x0040` – `0x0055` | Konfigurationsregister | R/W |
| `0x0080` | Applikationsspezifische Zusatzkonfiguration | R/W |
| `0x0100` | Kommandoregister | W |

Jedes Register ist **32 Bit** breit, die Adresse **16 Bit**, Übertragung
**Big-Endian**. Siehe [10-Interna-und-Timing.md](10-Interna-und-Timing.md) für
die Byte-Reihenfolge auf dem Bus.

> **Achtung:** `0x0010` bedeutet bei Distance- und Presence-Firmware etwas
> völlig Verschiedenes. Die Adressen überlappen, die Interpretation nicht.
> Deshalb muss man wissen, welche Firmware auf dem Modul läuft — automatisch
> erkannt wird das **nicht**.

---

## 2. Gemeinsame Konstanten — [XM125.h](../XM125.h)

### 2.1 Adresse

| Konstante | Wert | Anmerkung |
|---|---|---|
| `XM125_I2C_ADDR_DEFAULT` | `0x55` | Als Default im Konstruktor verwendet. ⚠ Der Kommentar im Header nennt an einer Stelle `0x52` als Werksadresse — maßgeblich ist die Konstante. Die Module dieses Projekts laufen auf `0x51`–`0x54` |

### 2.2 Gemeinsame Register

| Konstante | Adresse | R/W | Inhalt |
|---|---|---|---|
| `XM125_REG_VERSION` | `0x0000` | R | RSS-Firmware-Version (Rohwert, Bit-Layout lt. Datenblatt) |
| `XM125_REG_PROTOCOL_STATUS` | `0x0001` | R | Protokoll-Fehlerflags; `0` = kein Fehler |
| `XM125_REG_MEASURE_COUNTER` | `0x0002` | R | Anzahl Messungen seit Modul-Neustart |
| `XM125_REG_DETECTOR_STATUS` | `0x0003` | R | Status-/Fehlerbits des Detektors |
| `XM125_REG_COMMAND` | `0x0100` | W | Kommandoregister |

### 2.3 Gemeinsame Statusbits

| Konstante | Bit | Maske | Bedeutung |
|---|---|---|---|
| `XM125_STATUS_DETECTOR_ERROR` | 28 | `0x10000000` | Detektorfehler, Neustart/Neukonfiguration nötig |
| `XM125_STATUS_BUSY` | 31 | `0x80000000` | Kommando wird noch ausgeführt |

### 2.4 Gemeinsame Kommandos

| Konstante | Dezimal | Hex | Bedeutung |
|---|---|---|---|
| `XM125_CMD_ENABLE_UART_LOGS` | 32 | `0x20` | UART-Logging des Moduls einschalten |
| `XM125_CMD_DISABLE_UART_LOGS` | 33 | `0x21` | UART-Logging ausschalten |
| `XM125_CMD_LOG_CONFIGURATION` | 34 | `0x22` | Aktuelle Konfiguration ins UART-Log schreiben |
| `XM125_CMD_RESET_MODULE` | 1381192737 | `0x52535421` | Modul-Reset ("Magic Number"; die vier Bytes sind ASCII `R` `S` `T` `!`) |

Die drei Log-Kommandos haben **keine** eigene Methode in der Library — sie
lassen sich über eine abgeleitete Klasse mit
`_executeCommand(XM125_CMD_ENABLE_UART_LOGS, 1000)` senden. Die Ausgabe
erscheint dann auf dem UART des Moduls, nicht auf I²C; dafür muss der
UART-Pin des XM125 verkabelt sein.

### 2.5 Timeouts

| Konstante | Wert | Verwendet in |
|---|---|---|
| `XM125_DEFAULT_APPLY_TIMEOUT_MS` | `3000` | alle Apply-/Kalibrier-/Start-/Stop-Kommandos |
| `XM125_DEFAULT_MEASURE_TIMEOUT_MS` | `1000` | Default-Messtimeout der Zustandsmaschine, änderbar über `setMeasureTimeout()` |
| `XM125_WAKEUP_TIMEOUT_MS` | `1500` | `wakeUp()`, indirekt `begin()` |

---

## 3. Distance-Detector — [XM125Distance.h](../XM125Distance.h)

### 3.1 Ergebnisregister

| Konstante | Adresse | Inhalt |
|---|---|---|
| `XM125D_REG_DISTANCE_RESULT` | `0x0010` | Flags + Peak-Anzahl + Temperatur |
| `XM125D_REG_PEAK0_DISTANCE` | `0x0011` | Distanz Peak 0 (Peak `i` → `0x0011 + i`) |
| `XM125D_REG_PEAK0_STRENGTH` | `0x001B` | Stärke Peak 0 (Peak `i` → `0x001B + i`) |
| `XM125D_MAX_PEAKS` | `10` | Maximale Anzahl Peaks |

Ausgeschrieben:

| Peak | Distanzregister | Stärkeregister |
|---|---|---|
| 0 | `0x0011` | `0x001B` |
| 1 | `0x0012` | `0x001C` |
| 2 | `0x0013` | `0x001D` |
| 3 | `0x0014` | `0x001E` |
| 4 | `0x0015` | `0x001F` |
| 5 | `0x0016` | `0x0020` |
| 6 | `0x0017` | `0x0021` |
| 7 | `0x0018` | `0x0022` |
| 8 | `0x0019` | `0x0023` |
| 9 | `0x001A` | `0x0024` |

### 3.2 Bitfelder des Ergebnisregisters `0x0010`

| Konstante | Bits | Maske / Shift | Bedeutung |
|---|---|---|---|
| `XM125D_RESULT_NUM_DISTANCES_MASK` | 0–3 | `0x0000000F` | Anzahl gefundener Distanzen |
| `XM125D_RESULT_NEAR_START_EDGE` | 8 | `0x00000100` | Objekt nahe der unteren Messbereichsgrenze |
| `XM125D_RESULT_CALIBRATION_NEEDED` | 9 | `0x00000200` | Nachkalibrierung angefordert |
| `XM125D_RESULT_MEASURE_ERROR` | 10 | `0x00000400` | Messfehler |
| `XM125D_RESULT_TEMPERATURE_SHIFT` | 16–31 | Shift `16` | Temperatur in °C (signed, `int16_t`) |

Beispiel: `result = 0x00190002` → 2 Peaks, keine Flags, Temperatur `0x0019` = 25 °C.

### 3.3 Konfigurationsregister

| Konstante | Adresse | `config`-Feld | Default |
|---|---|---|---|
| `XM125D_REG_START` | `0x0040` | `startMm` | 250 |
| `XM125D_REG_END` | `0x0041` | `endMm` | 3000 |
| `XM125D_REG_MAX_STEP_LENGTH` | `0x0042` | `maxStepLength` | 0 (auto) |
| `XM125D_REG_CLOSE_RANGE_LEAKAGE_CANCEL` | `0x0043` | `closeRangeLeakageCancellation` | true |
| `XM125D_REG_SIGNAL_QUALITY` | `0x0044` | `signalQuality` | 15000 |
| `XM125D_REG_MAX_PROFILE` | `0x0045` | `maxProfile` | PROFILE5 |
| `XM125D_REG_THRESHOLD_METHOD` | `0x0046` | `thresholdMethod` | CFAR |
| `XM125D_REG_PEAK_SORTING` | `0x0047` | `peakSorting` | STRONGEST |
| `XM125D_REG_NUM_FRAMES_RECORDED_THRESHOLD` | `0x0048` | `numFramesRecordedThreshold` | 100 |
| `XM125D_REG_FIXED_AMPLITUDE_THRESHOLD` | `0x0049` | `fixedAmplitudeThreshold` | 100000 |
| `XM125D_REG_THRESHOLD_SENSITIVITY` | `0x004A` | `thresholdSensitivity` | 500 |
| `XM125D_REG_REFLECTOR_SHAPE` | `0x004B` | `reflectorShape` | GENERIC |
| `XM125D_REG_FIXED_STRENGTH_THRESHOLD` | `0x004C` | `fixedStrengthThreshold` | 0 |
| `XM125D_REG_MEASURE_ON_WAKEUP` | `0x0080` | `measureOnWakeup` | false |

Die Reihenfolge in der Tabelle entspricht exakt der Schreibreihenfolge in
`writeConfig()` (**✔ Code**) — hilfreich, wenn
`getLastFailedConfigRegister()` eine Adresse meldet: alle davor stehenden
Register wurden bereits erfolgreich geschrieben.

### 3.4 Kommandos

| Konstante | Wert | Methode |
|---|---|---|
| `XM125D_CMD_APPLY_CONFIG_AND_CALIBRATE` | 1 | `applyConfigAndCalibrate()` |
| `XM125D_CMD_MEASURE_DISTANCE` | 2 | `startMeasurement()` |
| `XM125D_CMD_APPLY_CONFIGURATION` | 3 | `applyConfiguration()` |
| `XM125D_CMD_CALIBRATE` | 4 | `calibrate()` |
| `XM125D_CMD_RECALIBRATE` | 5 | `recalibrate()` |

### 3.5 Enums

```cpp
enum XM125DMaxProfile     : uint32_t { XM125D_PROFILE1 = 1, ... XM125D_PROFILE5 = 5 };
enum XM125DThresholdMethod: uint32_t { XM125D_THRESHOLD_FIXED_AMPLITUDE = 1,
                                       XM125D_THRESHOLD_RECORDED        = 2,
                                       XM125D_THRESHOLD_CFAR            = 3,
                                       XM125D_THRESHOLD_FIXED_STRENGTH  = 4 };
enum XM125DPeakSorting    : uint32_t { XM125D_SORT_CLOSEST = 1, XM125D_SORT_STRONGEST = 2 };
enum XM125DReflectorShape : uint32_t { XM125D_REFLECTOR_GENERIC = 1, XM125D_REFLECTOR_PLANAR = 2 };
```

### 3.6 Statusbits (Register `0x0003`)

| Konstante | Bit | Maske | Bedeutung |
|---|---|---|---|
| `XM125D_STATUS_RSS_REGISTER_OK` | 0 | `0x00000001` | RSS-Registerzugriff ok |
| `XM125D_STATUS_CONFIG_CREATE_OK` | 1 | `0x00000002` | Konfigurationsobjekt erstellt |
| `XM125D_STATUS_SENSOR_CREATE_OK` | 2 | `0x00000004` | Sensorobjekt erstellt |
| `XM125D_STATUS_DETECTOR_CREATE_OK` | 3 | `0x00000008` | Detektorobjekt erstellt |
| `XM125D_STATUS_CONFIG_APPLY_ERROR` | 23 | `0x00800000` | Konfiguration abgelehnt |
| `XM125D_STATUS_SENSOR_CALIBRATE_ERROR` | 24 | `0x01000000` | Sensor-Kalibrierung fehlgeschlagen |
| `XM125D_STATUS_DETECTOR_CALIBRATE_ERROR` | 25 | `0x02000000` | Detektor-Kalibrierung fehlgeschlagen |
| `XM125_STATUS_DETECTOR_ERROR` | 28 | `0x10000000` | Sammelfehler (gemeinsam) |
| `XM125_STATUS_BUSY` | 31 | `0x80000000` | Kommando läuft (gemeinsam) |

**Beispiel-Dekodierung** `0x0000000F`: Bits 0–3 gesetzt → alle vier
Create-/Register-Schritte ok, keine Fehler, nicht busy → Sensor ist bereit.
`0x8000000F` → dasselbe, aber gerade `BUSY`. `0x1100000F` → Bits 24 und 28:
Sensor-Kalibrierung fehlgeschlagen.

---

## 4. Presence-Detector — [XM125Presence.h](../XM125Presence.h)

### 4.1 Ergebnisregister

| Konstante | Adresse | Inhalt |
|---|---|---|
| `XM125P_REG_PRESENCE_RESULT` | `0x0010` | Detektions-Flags + Temperatur |
| `XM125P_REG_PRESENCE_DISTANCE` | `0x0011` | Distanz der Detektion in mm |
| `XM125P_REG_INTRA_PRESENCE_SCORE` | `0x0012` | Score schnelle Bewegung |
| `XM125P_REG_INTER_PRESENCE_SCORE` | `0x0013` | Score langsame Bewegung |

### 4.2 Bitfelder des Ergebnisregisters `0x0010`

| Konstante | Bit | Maske / Shift | Bedeutung |
|---|---|---|---|
| `XM125P_RESULT_PRESENCE_DETECTED` | 0 | `0x00000001` | Anwesenheit im aktuellen Frame |
| `XM125P_RESULT_PRESENCE_DETECTED_STICKY` | 1 | `0x00000002` | "klebrige" Detektion |
| `XM125P_RESULT_DETECTOR_ERROR` | 15 | `0x00008000` | Detektorfehler |
| `XM125P_RESULT_TEMPERATURE_SHIFT` | 16–31 | Shift `16` | Temperatur °C (signed) |

### 4.3 Konfigurationsregister

| Konstante | Adresse | `config`-Feld | Default |
|---|---|---|---|
| `XM125P_REG_SWEEPS_PER_FRAME` | `0x0040` | `sweepsPerFrame` | 16 |
| `XM125P_REG_INTER_FRAME_PRESENCE_TIMEOUT` | `0x0041` | `interFramePresenceTimeoutS` | 3 s |
| `XM125P_REG_INTER_PHASE_BOOST_ENABLED` | `0x0042` | `interPhaseBoostEnabled` | false |
| `XM125P_REG_INTRA_DETECTION_ENABLED` | `0x0043` | `intraDetectionEnabled` | true |
| `XM125P_REG_INTER_DETECTION_ENABLED` | `0x0044` | `interDetectionEnabled` | true |
| `XM125P_REG_FRAME_RATE` | `0x0045` | `frameRateMilliHz` | 12000 mHz |
| `XM125P_REG_INTRA_DETECTION_THRESHOLD` | `0x0046` | `intraDetectionThreshold` | 1300 |
| `XM125P_REG_INTER_DETECTION_THRESHOLD` | `0x0047` | `interDetectionThreshold` | 1000 |
| `XM125P_REG_INTER_FRAME_DEVIATION_TC` | `0x0048` | `interFrameDeviationTimeConstMs` | 500 ms |
| `XM125P_REG_INTER_FRAME_FAST_CUTOFF` | `0x0049` | `interFrameFastCutoffMilliHz` | 6000 mHz |
| `XM125P_REG_INTER_FRAME_SLOW_CUTOFF` | `0x004A` | `interFrameSlowCutoffMilliHz` | 200 mHz |
| `XM125P_REG_INTRA_FRAME_TIME_CONST` | `0x004B` | `intraFrameTimeConstMs` | 150 ms |
| `XM125P_REG_INTRA_OUTPUT_TIME_CONST` | `0x004C` | `intraOutputTimeConstMs` | 300 ms |
| `XM125P_REG_INTER_OUTPUT_TIME_CONST` | `0x004D` | `interOutputTimeConstMs` | 2000 ms |
| `XM125P_REG_AUTO_PROFILE_ENABLED` | `0x004E` | `autoProfileEnabled` | true |
| `XM125P_REG_AUTO_STEP_LENGTH_ENABLED` | `0x004F` | `autoStepLengthEnabled` | true |
| `XM125P_REG_MANUAL_PROFILE` | `0x0050` | `manualProfile` | PROFILE4 |
| `XM125P_REG_MANUAL_STEP_LENGTH` | `0x0051` | `manualStepLength` | 72 |
| `XM125P_REG_START` | `0x0052` | `startMm` | 300 mm |
| `XM125P_REG_END` | `0x0053` | `endMm` | 2500 mm |
| `XM125P_REG_RESET_FILTERS_ON_PREPARE` | `0x0054` | `resetFiltersOnPrepare` | true |
| `XM125P_REG_HWAAS` | `0x0055` | `hwaas` | 32 |
| `XM125P_REG_DETECTION_ON_GPIO` | `0x0080` | `detectionOnGpio` | false |

> Beachten: Beim Presence-Detektor liegen `startMm`/`endMm` auf `0x0052`/`0x0053`,
> beim Distance-Detektor auf `0x0040`/`0x0041`. Wer Register direkt anspricht,
> darf das nicht verwechseln.

### 4.4 Kommandos

| Konstante | Wert | Methode |
|---|---|---|
| `XM125P_CMD_APPLY_CONFIGURATION` | 1 | `applyConfiguration()` |
| `XM125P_CMD_START_DETECTOR` | 2 | `start()` |
| `XM125P_CMD_STOP_DETECTOR` | 3 | `stop()` |

### 4.5 Enum

```cpp
enum XM125PProfile : uint32_t { XM125P_PROFILE1 = 1, ... XM125P_PROFILE5 = 5 };
```

### 4.6 Statusbits (Register `0x0003`)

| Konstante | Bit | Maske | Bedeutung |
|---|---|---|---|
| `XM125P_STATUS_RSS_REGISTER_OK` | 0 | `0x00000001` | RSS-Registerzugriff ok |
| `XM125P_STATUS_CONFIG_CREATE_OK` | 1 | `0x00000002` | Konfigurationsobjekt erstellt |
| `XM125P_STATUS_SENSOR_CREATE_OK` | 2 | `0x00000004` | Sensorobjekt erstellt |
| `XM125P_STATUS_SENSOR_CALIBRATE_OK` | 3 | `0x00000008` | **Sensor-Kalibrierung erfolgreich** |
| `XM125P_STATUS_DETECTOR_CREATE_OK` | 4 | `0x00000010` | Detektorobjekt erstellt |
| `XM125P_STATUS_DETECTOR_BUFFER_OK` | 5 | `0x00000020` | Detektorpuffer alloziert |
| `XM125P_STATUS_SENSOR_BUFFER_OK` | 6 | `0x00000040` | Sensorpuffer alloziert |
| `XM125P_STATUS_CONFIG_APPLY_OK` | 7 | `0x00000080` | Konfiguration angewendet |
| *(keine Konstante)* | 16–23 | `0x00010000`+ | Fehlerbits, spiegeln die Erfolgsbits 0–7 (Bit 16 = `RSS_REGISTER_ERROR`, Bit 17 = `CONFIG_CREATE_ERROR`, …) |
| `XM125_STATUS_DETECTOR_ERROR` | 28 | `0x10000000` | Sammelfehler |
| `XM125_STATUS_BUSY` | 31 | `0x80000000` | Kommando läuft |

Ein vollständig hochgefahrener Presence-Detektor zeigt also `0x000000FF`
(Bits 0–7 gesetzt), solange keine Fehler vorliegen.

Praktische Prüfung der gespiegelten Fehlerbits:

```cpp
uint32_t st = radar.getDetectorStatus();
uint32_t errBits = (st >> 16) & 0xFF;      // Fehlerbits auf Positionen 0..7 schieben
if (errBits) {
  Serial.print("Presence-Fehlerbits: 0x"); Serial.println(errBits, HEX);
}
```

---

## 5. Konstanten in Kurzform (Spickzettel)

```
gemeinsam:   0x0000 VERSION   0x0001 PROTO_STATUS  0x0002 MEAS_COUNTER
             0x0003 DET_STATUS                     0x0100 COMMAND
             Bit31 BUSY   Bit28 DETECTOR_ERROR
             RESET = 0x52535421

Distance:    Result 0x0010 | Peak-Dist 0x0011+i | Peak-Str 0x001B+i | max 10
             Config 0x0040..0x004C, 0x0080
             CMD 1=ApplyCal 2=Measure 3=Apply 4=Cal 5=Recal

Presence:    Result 0x0010 | Dist 0x0011 | Intra 0x0012 | Inter 0x0013
             Config 0x0040..0x0055, 0x0080
             CMD 1=Apply 2=Start 3=Stop

Timeouts:    Apply/Cal 3000 ms | Measure 1000 ms | Wakeup 1500 ms
```
