# RadarArray

Alle XM125-Radarsensoren des Roboters in **einem** Objekt. Kapselt Einrichtung,
gleichzeitige Messrunden, Fehlerzählung und Wiederanlauf toter Sensoren, damit
in der [systemlib](../systemlib/systemlib.md) nur noch drei Aufrufe übrig
bleiben.

Der eigentliche Treiber liegt in [XM125](../XM125/XM125.md), alle Zahlenwerte
in [params_radar.h](../include/include.md).

---

## Benutzung

```cpp
sys.initialize();          // ruft intern Radar.begin() auf

void loop() {
    sys.readSensors();     // ruft intern Radar.update() auf

    uint16_t d = sys.Radar[RadarVo].Dist;   // Rohwert Peak 0
    if (sys.Radar[RadarLi].isCon) { ... }   // antwortet der Sensor?
}
```

| Position | Index | Einbau |
|---|---|---|
| `RadarVo` | 0 | vorne |
| `RadarRe` | 1 | rechts |
| `RadarHi` | 2 | hinten |
| `RadarLi` | 3 | links |

Die Reihenfolge entspricht `RADAR_SENSOR_ADDRS` in `params.h`.

```cpp
struct RadarData
{
    uint16_t Dist;      // Rohwert von Peak 0 ("Counts", NICHT mm — siehe XM125.md), alpha-beta-gefiltert
    uint16_t DistRaw;   // derselbe Wert ungefiltert (für die Positionsfusion)
    int32_t  Strenght;  // Signalstärke von Peak 0
    uint8_t  Peaks;     // Anzahl erkannter Peaks (0 = nichts gesehen)
    bool     isCon;     // true = Sensor antwortet und ist konfiguriert
    uint32_t Time;      // millis() der letzten übernommenen Messung (0 = noch nie)
};
```

`Peaks == 0` liefert `Dist`/`Strenght` als `0`, damit kein alter Wert stehen
bleibt. Ausgewertet wird nur Peak 0, deshalb holt die Library auch nur diesen
einen aus dem Sensor (`RADAR_PEAKS_TO_FETCH`).

---

## API

| Funktion | Blockiert | Beschreibung |
|---|---|---|
| `begin(wire = I2C_RADAR)` | ja, bis ~2 s je Sensor | richtet alle Sensoren ein und startet die erste Messrunde; gibt die Anzahl erreichbarer Sensoren zurück |
| `update()` | **nie** | einmal je Zyklus: fertige Ergebnisse übernehmen, nächste Runde starten |
| `calib()` | ja, ~1 s je Sensor | schreibt `config` in alle Sensoren und kalibriert neu |
| `operator[](pos)` | – | `RadarData` der Einbauposition |
| `giveCon(pos)` | – | antwortet der Sensor? |
| `giveAge(pos)` | – | Alter des Messwerts in ms (`millis() - Time`); ein Sensor ohne Messung liefert `millis()` |
| `giveCount()` | – | `RADAR_NUM_SENSORS` |
| `giveSensor(pos)` | – | Direktzugriff auf `XM125Distance` (Temperatur, Diagnose) |
| `config` | – | `XM125DistanceConfig` für **alle** Sensoren, vorbelegt aus `params.h` |

`Wire.begin()` muss vor `begin()` gelaufen sein — in der `systemlib` erledigt
das `initialize()`.

### Abstimmung ändern

Dauerhaft: Werte in [params.h](../include/include.md) anpassen.
Zur Laufzeit (z.B. per Debugger-Parameter):

```cpp
sys.Radar.config.endMm = 4000;
sys.calibRadar();          // schreibt config in alle Sensoren + kalibriert
```

---

## Ablauf intern

Alle Sensoren messen **gleichzeitig** und es wird **nie** auf ein Ergebnis
gewartet:

| Schritt | Aufgabe | Zeit im Zyklus |
|---|---|---|
| `_startRound()` | `startMeasurement()` bei allen Sensoren — sie messen dadurch zeitgleich | 1 Registerschreibzugriff je Sensor |
| `update()` | erst ab `RADAR_POLL_START_FRACTION` (70 %) der zuletzt beobachteten Rundendauer je Zyklus einmal `isMeasuring()` je laufendem Sensor, davor kein Buszugriff; fertige Ergebnisse übernehmen; ist die Runde komplett, sofort die nächste starten | höchstens 1 Registerlesezugriff je Sensor, in den ersten ~70 % der Runde keiner |
| `_takeResult(idx)` | `getLastResult()` → `RadarData` | 0 |
| `_measureFailed(idx)` | Fehlschläge zählen, nach `RADAR_MAX_CONSEC_FAILURES` (5) den Sensor als tot markieren (`isCon = false`) | 0 |
| `_retrySetup(idx)` | Wiederanlauf eines toten Sensors — **blockiert** bis ~2 s, deshalb höchstens alle `RADAR_REINIT_INTERVAL_MS` | selten |

Damit kostet das Radar rund 1 ms pro Zyklus statt der vollen Messdauer
(20–200 ms). Die Messrate ändert sich dadurch nicht — nur wartet die
Hauptschleife nicht mehr darauf.
