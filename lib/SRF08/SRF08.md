# SRF08 – Ultraschall-Entfernungssensor (I²C, non-blocking)

Treiber für den Devantech **SRF08**. Vollständig non-blocking (State Machine)
mit mehrstufiger Filterpipeline gegen Geisterechos.

Dateien: `SRF08.h`, `SRF08.cpp`

Herkunft: KI-generiert (Julius Kimpfler), `SRF08Manager` entfernt von
Tobias Schauer.

---

## Konfiguration

### Instanziierung

```cpp
SRF08Sensor SRF08_vo(SRF08_ADDR(0xE2), SRF08_RANGE_2M, SRF08_GAIN_MID);
```

| Parameter | Default | Bedeutung |
|---|---|---|
| `addr7bitOr8bit` | – | Adresse; 7-Bit (`0x70`…`0x7F`) oder 8-Bit (`0xE0`…`0xFE`) werden beide akzeptiert |
| `rangeReg` | `SRF08_RANGE_2M` | Reichweitenregister |
| `gainReg` | `SRF08_GAIN_MID` | Verstärkungsregister |

Das Makro `SRF08_ADDR(0xE0)` rechnet die aufgedruckte 8-Bit-Adresse in die
7-Bit-Adresse für `Wire` um (`>> 1`).

**`Wire.begin()` und `Wire.setClock(400000)` müssen vor `begin()` erfolgen.**

### Range-Presets

`range_mm = (reg × 43) + 43`, Wartezeit `≈ 2 × range_mm / 343 + 2 ms`

| Konstante | Register | Reichweite | Messzeit |
|---|---|---|---|
| `SRF08_RANGE_1M` | 24 | ~1075 mm | ~8 ms |
| `SRF08_RANGE_2M` | 46 | ~2021 mm | ~14 ms |
| `SRF08_RANGE_3M` | 69 | ~3010 mm | ~20 ms |
| `SRF08_RANGE_4M` | 92 | ~3999 mm | ~25 ms |
| `SRF08_RANGE_6M` | 139 | ~6020 mm | ~37 ms |

Kürzere Reichweite = höhere Abfragerate. Für schnelle Regelung möglichst
knapp über der tatsächlich benötigten Distanz wählen. Alternativ
`setRangeMeters(float)` verwenden.

### Gain-Presets

| Konstante | Register | Gain | Einsatz |
|---|---|---|---|
| `SRF08_GAIN_MIN` | 0 | 94 | sehr kurze Zyklen |
| `SRF08_GAIN_LOW` | 10 | 133 | stark reflektierende Umgebung |
| `SRF08_GAIN_MID` | 18 | 199 | **Default**, guter Kompromiss |
| `SRF08_GAIN_HIGH` | 25 | 352 | normale Umgebung |
| `SRF08_GAIN_MAX` | 31 | 1025 | Werkseinstellung |

Zu hoher Gain → mehr Geisterechos von Seitenwänden. Bei vielen Sensoren dicht
beieinander eher niedrig wählen.

### Filter- und Timing-Konstanten (Compile-Zeit, `SRF08.h`)

| Konstante | Default | Bedeutung |
|---|---|---|
| `SRF08_MEDIAN_WINDOW` | `5` | Fenstergröße Medianfilter (muss ungerade sein) |
| `SRF08_EMA_ALPHA_DEF` | `0.30f` | EMA-Glättung: 0 = träge, 1 = Filter aus |
| `SRF08_JUMP_DEF` | `40` | max. plausible Änderung pro Messung [cm] |
| `SRF08_MIN_DIST_CM` | `3` | untere Plausibilitätsgrenze |
| `SRF08_MAX_DIST_CM` | `600` | obere Plausibilitätsgrenze |
| `SRF08_POLL_TIMEOUT_MS` | `20` | Polling-Fenster nach der Messzeit |

Zur Laufzeit anpassbar: `setEMAAlpha(float)` und `setJumpThreshold(uint16_t)`.

Im Projekt werden alle vier Sensoren auf `alpha = 0.25` und
`jumpThreshold = 50 cm` gesetzt – also etwas stärker geglättet und toleranter
gegen schnelle Distanzänderungen als der Default.

---

## Filterpipeline

```
raw → [1. Bereichscheck] → [2. Kaltstart] → [3. Sprungfilter]
     → [4. Median] → [5. EMA] → gefilterter Wert
```

1. **Bereichscheck:** `0` (kein Echo) oder außerhalb `MIN`/`MAX` → letzter
   gültiger Wert wird gehalten, der Median wird *nicht* befüllt.
2. **Kaltstart:** erste gültige Messung initialisiert Median und EMA direkt.
3. **Sprungfilter:** Änderung > `jumpThreshold`? Wert kommt in den Median;
   bestätigt der Median den Sprung ebenfalls, ist es eine echte Bewegung → EMA
   wird hart nachgezogen. Sonst Geisterecho → Ausgabe wird eingefroren.
4. **Median** (5 Samples, Insertion-Sort auf Stack-Kopie) unterdrückt
   Einzelausreißer.
5. **EMA** glättet das Restrauschen.

Faustregel beim Tunen: erst `jumpThreshold` an die reale Robotergeschwindigkeit
anpassen, dann `alpha`. Zu kleines `alpha` macht den Wert träge und damit für
eine Regelung unbrauchbar.

---

## Ablauf / State Machine

`SRF08State`: `UNINITIALIZED` → `IDLE` ⇄ `RANGING`

```cpp
sensor.begin(Wire);        // → IDLE
sensor.startRanging();     // → RANGING, Ping abgesetzt
// ... in loop():
if (sensor.update())       // true = neue gefilterte Messung
    sensor.startRanging(); // nächste Messung anstoßen
```

`update()` wartet zuerst timer-basiert die berechnete Messzeit ab (kein
I²C-Verkehr), pollt danach bis zu `SRF08_POLL_TIMEOUT_MS` das Statusregister
(`0xFF` = busy) und liest anschließend Lichtsensor und Echo. Nach Ablauf des
Polling-Fensters wird gelesen, auch wenn der Sensor nicht sauber
zurückgemeldet hat – ein I²C-Fehler blockiert den Sensor also nicht dauerhaft.

---

## API

| Funktion | Beschreibung |
|---|---|
| `bool begin(TwoWire& wire = Wire)` | 15 ms Settling, bis zu 3 Verbindungsversuche, schreibt Gain und Range. `false` = nicht erreichbar |
| `void setRange(uint8_t reg)` / `setRangeMeters(float m)` | Reichweite ändern (Wartezeit wird neu berechnet) |
| `void setGain(uint8_t reg)` | Gain ändern (0…31) |
| `void startRanging()` | Messung starten |
| `bool update()` | zyklisch aufrufen; `true` = einmalig neue Daten |
| `uint16_t getDistance()` | **gefilterte** Distanz [cm] |
| `uint16_t getRaw()` | letzter Rohwert [cm] |
| `uint8_t getLightLevel()` | Lichtsensor 0…255 |
| `uint32_t getCycleTimeMs()` | Dauer der letzten Messung |
| `SRF08State getState()`, `uint8_t getAddress()`, `bool hasValidData()` | Status |
| `void resetFilters()` | Filterzustand verwerfen |
| `bool changeI2CAddress(uint8_t newAddr)` | Adresse dauerhaft ändern |

### Adresse ändern

```cpp
sensor.changeI2CAddress(0xE4);
```

**Nur mit genau einem Sensor am Bus ausführen!** Zulässig sind gerade
8-Bit-Adressen `0xE0`…`0xFE`. Die Sequenz `A0 → AA → A5 → <Adresse>` wird auf
Register 0 geschrieben; danach prüft die Funktion die Erreichbarkeit unter der
neuen Adresse. Hilfsskript: `test/utilityScriptsSRF08/ChangeAddress.cpp`,
Bus-Scanner: `test/utilityScriptsSRF08/scanner.cpp`.

---

## Verwendung im Projekt

Vier Sensoren an `I2C_US` (= `Wire`):

| Instanz | 8-Bit-Adresse | Richtung |
|---|---|---|
| `SRF08_vo` | `0xE2` | vorne |
| `SRF08_re` | `0xE4` | rechts |
| `SRF08_hi` | `0xE6` | hinten |
| `SRF08_li` | `0xE8` | links |

Abholen über `sys.giveUS_vo()` / `_re()` / `_hi()` / `_li()`.

> `systemlib::readUS()` ist in `readSensors()` derzeit **auskommentiert** –
> die Ultraschallwerte werden also nicht aktualisiert.

---

## Bekannte Baustellen

- Alle vier Sensoren pingen gleichzeitig (`startRanging()` direkt
  nacheinander in `initialize()`). Bei benachbarten Sensoren kann das
  Übersprechen erzeugen; ein Round-Robin wäre sauberer (dafür gab es
  ursprünglich den entfernten `SRF08Manager`).
- Das Register `SRF08_REG_RANGE` (0x02) ist beim Lesen das Echo-High-Byte –
  deshalb wird in `begin()` Gain **vor** Range geschrieben.
