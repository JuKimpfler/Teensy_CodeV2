# Presence-Detector (`XM125Presence`)

Header: [XM125Presence.h](../XM125Presence.h) · Implementierung: [XM125Presence.cpp](../XM125Presence.cpp)

Der Presence-Detector erkennt Anwesenheit über **schnelle (Intra-Frame)**
und **langsame (Inter-Frame)** Bewegungen innerhalb eines konfigurierbaren
Distanzfensters. Im Gegensatz zum Distance-Detector misst er nach dem
Start **kontinuierlich selbstständig** im Hintergrund.

## Typischer Ablauf

```cpp
XM125Presence radar;
radar.begin(Wire);

radar.config.startMm = 300;
radar.config.endMm   = 2000;
radar.applyConfiguration();  // Konfiguration ist danach nicht mehr änderbar!
radar.start();

while (true) {
  if (radar.update() && radar.isPresenceDetected()) {
    Serial.println(radar.getPresenceDistanceMm());
  }
}
```

Alle Konfigurationsfelder (`radar.config.*`) sind in
[04-Konfiguration.md](04-Konfiguration.md) im Detail beschrieben.

## API-Überblick

| Methode | Beschreibung |
|---|---|
| `writeConfig()` | Schreibt alle Werte aus `config` in die Konfigurationsregister. Bricht beim ersten fehlgeschlagenen Register-Write ab (fail-fast) |
| `getLastFailedConfigRegister()` | Registeradresse des zuletzt fehlgeschlagenen `writeConfig()`-Schreibvorgangs (`0xFFFF` = keiner) |
| `applyConfiguration(timeoutMs = 3000)` | `writeConfig()` + Kommando `APPLY_CONFIGURATION`, blockiert bis fertig. **Danach ist `config` nicht mehr wirksam änderbar, ohne erneut `applyConfiguration()` aufzurufen** |
| `start(timeoutMs = 3000)` | Startet den Detektor — danach liefert `update()` laufend neue Ergebnisse |
| `stop(timeoutMs = 3000)` | Stoppt den Detektor |
| `update()` | Liest das aktuelle Ergebnisregister ein (**non-blocking**, sendet kein Kommando — der Detektor misst nach `start()` selbstständig weiter) |
| `isPresenceDetected()` | `true`, wenn im aktuellen Frame Anwesenheit erkannt wurde |
| `isPresenceDetectedSticky()` | "Klebrige" Variante — bleibt nach Erkennung noch für `interFramePresenceTimeoutS` Sekunden `true` |
| `hasDetectorError()` | `true` bei Detektorfehler → `stop()` + `start()` zum Neustart |
| `getTemperature()` | Sensortemperatur in °C (nur relative Genauigkeit) |
| `getPresenceDistanceMm()` | Distanz der Detektion in mm (nur gültig wenn `isPresenceDetected()`) |
| `getIntraPresenceScore()` | Score der schnellen Bewegungserkennung |
| `getInterPresenceScore()` | Score der langsamen Bewegungserkennung |

## Ergebnisregister im Detail (Reg. `0x0010`, `PRESENCE_RESULT`)

| Bits | Bedeutung |
|---|---|
| 0 | `PRESENCE_DETECTED` — Anwesenheit im aktuellen Frame erkannt |
| 1 | `PRESENCE_DETECTED_STICKY` — Anwesenheit erkannt (bleibt länger `true`, siehe `interFramePresenceTimeoutS`) |
| 15 | `DETECTOR_ERROR` — Detektorfehler |
| 16–31 | Temperatur (signed, °C) |

Weitere Ergebnisregister: `PRESENCE_DISTANCE` (`0x0011`),
`INTRA_PRESENCE_SCORE` (`0x0012`), `INTER_PRESENCE_SCORE` (`0x0013`).

## Presence-spezifische Status-Bits (Reg. `DETECTOR_STATUS`, `0x0003`)

| Bit | Konstante | Bedeutung |
|---|---|---|
| 0 | `RSS_REGISTER_OK` | RSS-Registerzugriff erfolgreich |
| 1 | `CONFIG_CREATE_OK` | Konfigurationsobjekt erfolgreich erstellt |
| 2 | `SENSOR_CREATE_OK` | Sensorobjekt erfolgreich erstellt |
| 3 | `SENSOR_CALIBRATE_OK` | Sensor-Kalibrierung erfolgreich |
| 4 | `DETECTOR_CREATE_OK` | Detektorobjekt erfolgreich erstellt |
| 5 | `DETECTOR_BUFFER_OK` | Detektor-Puffer erfolgreich alloziert |
| 6 | `SENSOR_BUFFER_OK` | Sensor-Puffer erfolgreich alloziert |
| 7 | `CONFIG_APPLY_OK` | Konfiguration erfolgreich angewendet |
| 16+ | *(entspr. Fehler-Bit)* | Fehler-Bits spiegeln die Success-Bits ab Bit 16 (z.B. `RSS_REGISTER_ERROR` = Bit 16) |
| 28 | `XM125_STATUS_DETECTOR_ERROR` (gemeinsam) | Allgemeiner Detektorfehler |
| 31 | `XM125_STATUS_BUSY` (gemeinsam) | Kommando läuft noch |

## Kommandos (Register `COMMAND`, `0x0100`)

| Kommando | Wert | Ausgelöst durch |
|---|---|---|
| `APPLY_CONFIGURATION` | 1 | `applyConfiguration()` |
| `START_DETECTOR` | 2 | `start()` |
| `STOP_DETECTOR` | 3 | `stop()` |

## Hinweis zur Fehlerbehandlung

Meldet `hasDetectorError()` einen Fehler, hilft laut Testcode
([XM125_Presence.cpp](../examples/XM125_Presence.cpp)) meist ein
einfaches `stop()` gefolgt von `start()`, um den Detektor neu zu
initialisieren — ein komplettes `applyConfiguration()` ist dafür in der
Regel nicht nötig, solange sich die Konfiguration nicht geändert hat.
