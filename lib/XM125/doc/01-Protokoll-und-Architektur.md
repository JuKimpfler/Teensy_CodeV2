# Protokoll & Architektur

## Klassenhierarchie

```
XM125Base            (XM125.h / XM125.cpp)
 ├── XM125Distance    (XM125Distance.h / XM125Distance.cpp)
 └── XM125Presence    (XM125Presence.h / XM125Presence.cpp)
```

`XM125Base` implementiert alles, was beiden Applikationen gemeinsam ist:
I²C-Low-Level-Zugriff, Wake/Ready-Handling und die gemeinsamen Register
(Version, Protocol Status, Measure Counter, Detector Status). Die beiden
abgeleiteten Klassen fügen die jeweils applikationsspezifischen
Konfigurations-/Ergebnisregister und Kommandos hinzu.

Welche Firmware auf dem Modul läuft (Distance oder Presence), wird **nicht**
automatisch erkannt — das muss beim Programmieren durch Wahl der passenden
Klasse (`XM125Distance` bzw. `XM125Presence`) bekannt sein.

## I²C-Register-Protokoll

Jedes Register ist 32-bit breit und wird über eine 16-bit Registeradresse
angesprochen. Die Übertragung ist **Big-Endian**:

- **Schreiben:** `[Reg-High][Reg-Low][Val-Byte3][Val-Byte2][Val-Byte1][Val-Byte0]` in einer einzigen I²C-Transaktion.
- **Lesen:** Registeradresse schreiben (mit *Repeated Start*, kein STOP), dann 4 Bytes anfordern (`requestFrom`).

Implementiert in `XM125Base::_writeReg()` / `XM125Base::_readReg()`
([XM125.cpp](../XM125.cpp)).

## Gemeinsame Register (0x0000–0x01FF)

| Register | Adresse | R/W | Bedeutung |
|---|---|---|---|
| `VERSION` | `0x0000` | R | RSS-Firmware-Version (Rohwert) |
| `PROTOCOL_STATUS` | `0x0001` | R | Protokoll-Fehlerflags; `0` = kein Fehler. Ungleich 0 deutet z.B. auf falsches Register/Kommando für die geflashte Firmware hin |
| `MEASURE_COUNTER` | `0x0002` | R | Anzahl Messungen seit Neustart des Moduls |
| `DETECTOR_STATUS` | `0x0003` | R | Status-/Fehlerflags des Detektors (Bit-Layout je nach Firmware, siehe unten) |
| `COMMAND` | `0x0100` | W | Kommandoregister — hier werden Aktionen (Konfigurieren, Kalibrieren, Messen, ...) ausgelöst |

### Gemeinsame Status-Bits (Register `DETECTOR_STATUS`)

| Bit | Konstante | Bedeutung |
|---|---|---|
| 31 | `XM125_STATUS_BUSY` | Ein Kommando wird noch ausgeführt |
| 28 | `XM125_STATUS_DETECTOR_ERROR` | Detektor-Fehler, Neustart/Neukonfiguration nötig |

Abfrage über die statischen Helfer `XM125Base::statusBusy(status)` und
`XM125Base::statusError(status)`.

### Gemeinsame Kommandos

| Kommando | Wert | Bedeutung |
|---|---|---|
| `XM125_CMD_ENABLE_UART_LOGS` | 32 | UART-Logging aktivieren |
| `XM125_CMD_DISABLE_UART_LOGS` | 33 | UART-Logging deaktivieren |
| `XM125_CMD_LOG_CONFIGURATION` | 34 | Aktuelle Konfiguration ins Log schreiben |
| `XM125_CMD_RESET_MODULE` | 1381192737 (0x52535421, "Magic Number") | Modul-Reset laut Datenblatt — `XM125Base::resetModule()` |

Die drei Log-Kommandos haben keine eigene Methode. `resetModule()` schon, denn
nach dem Reset ist Nacharbeit nötig: Konfigurationsregister und Kalibrierung
sind weg. `XM125Distance::hardCalibration()` erledigt Reset, Registerschreiben
und Kalibrierung in einem Aufruf.

## Ablauf einer Messung (laut Acconeer User Guide)

1. `WAKE_UP`-Pin high setzen (falls verkabelt) und auf `MCU_INT` high warten.
2. `DETECTOR_STATUS` lesen, prüfen dass `BUSY` und `DETECTOR_ERROR` nicht gesetzt sind.
3. Konfigurationsregister schreiben (`writeConfig()`).
4. `APPLY_CONFIGURATION(_AND_CALIBRATE)`-Kommando senden, auf `BUSY=0` pollen.
5. Start-/Measure-Kommando senden, auf `BUSY=0` pollen.
6. Ergebnisregister lesen.

Diese Bibliothek bildet den Ablauf über folgende Methoden ab:

| Schritt | Distance | Presence |
|---|---|---|
| 1+2 | `begin()` | `begin()` |
| 3+4 | `applyConfigAndCalibrate()` / `applyConfiguration()` | `applyConfiguration()` |
| 5+6 | `startMeasurement()`, dann `isMeasuring()` bis `false`, dann `getLastResult()` | `start()` einmalig, danach laufend `update()` |

**Wichtiger Unterschied zwischen den beiden Detektoren:** Der
Distance-Detector arbeitet **request/response** (jedes `startMeasurement()`
löst genau eine Messung aus; gewartet wird dabei nicht — `isMeasuring()`
meldet, wann das Ergebnis vorliegt). Der
Presence-Detector arbeitet nach `start()` **kontinuierlich selbstständig**
im Hintergrund; `update()` liest nur das zuletzt berechnete Ergebnis aus,
ohne selbst ein Kommando zu senden (siehe [03-Presence-Detector.md](03-Presence-Detector.md)).

## WAKE_UP / MCU_INT Pins

- `wakePin` (Konstruktor-Parameter, Default `-1` = nicht verkabelt): Steuert
  den `WAKE_UP`-Eingang des Moduls. High = wach, Low = Schlaf. Der
  Schlafzustand wird **nur** durch expliziten Host-Eingriff ausgelöst — das
  Modul schläft nie von selbst. Diese Bibliothek sendet **kein**
  Sleep-Kommando; `sleep()` wirkt nur, wenn tatsächlich ein `wakePin`
  verkabelt ist.
- `intPin` (Konstruktor-Parameter, Default `-1` = nicht verkabelt): Liegt an
  `MCU_INT` des Moduls. Ist er verkabelt, wartet `wakeUp()` auf dessen
  HIGH-Flanke statt auf I²C-Polling.

Sind beide Pins nicht verkabelt (Projekt-Standard in diesem Repo, siehe
`WAKE_PIN = -1` / `INT_PIN = -1` in den Testdateien), bleibt das Modul
dauerhaft wach; `wakeUp()` pollt in diesem Fall stattdessen bis zum Timeout
per I²C, ob der Sensor bereits antwortet (reine Boot-Absicherung, kein
echtes "Aufwecken").

> **Sicherheitshinweis (Code-Kommentar in [XM125.h](../XM125.h)):** Auf
> Hardware ohne verkabelten `wakePin` niemals einen eigenen
> Sleep-Mechanismus ergänzen — ohne Rückweg über den Pin wäre das Modul
> dann nur noch per Power-Cycle erreichbar.

## Mehrere Sensoren an einem Bus

Da die I²C-Adresse per Konstruktor-Parameter gesetzt wird, können mehrere
XM125-Module am selben `Wire`-Bus betrieben werden, sofern sie
unterschiedliche Adressen haben (Werkseinstellung ist bei allen Modulen
gleich — Adressänderung erfolgt über Hardware/Jumper, nicht per Software).
Siehe [06-Beispiele.md](06-Beispiele.md) für ein 4-Sensor-Beispiel.
