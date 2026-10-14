# Utils – Kleine Hilfsfunktionen

Sammelstelle für projektweite Hilfsfunktionen. Die Bibliothek stellt eine
**globale Instanz `Util`** bereit, die überall genutzt werden kann, wo
`Utils.h` eingebunden ist.

Dateien: `Utils.h`, `Utils.cpp`

---

## Konfiguration

Keine. Die globale Instanz wird in `Utils.cpp` definiert:

```cpp
Utils Util;
```

und im Header als `extern Utils Util;` bekanntgemacht. Es muss also nichts
instanziiert oder initialisiert werden – nur `#include <Utils.h>`.

---

## API

### `bool inRange(float Value, float min, float max)`

Bereichsprüfung, Grenzen **inklusive**:

```cpp
return (Value >= min && Value <= max);
```

Verwendet z.B. in `systemlib::setDBGChannel()`, um die freien Debug-Kanäle
0–20 abzusichern.

### `float wrapAngle(float Angle)`

Bildet einen Winkel auf **±180°** ab:

```cpp
return fmod(Angle + 180.0f, 360.0f) - 180.0f;
```

Damit werden im ganzen Projekt Winkeldifferenzen normalisiert, z.B.:

```cpp
BNO_eulerZ   = Util.wrapAngle(BNO.eulHeading());              // 0..360 → ±180
BNO_OrbitDir = Util.wrapAngle(BNO_eulerZ - BNO_calibration);  // Differenz normalisieren
IR_Angle     = Util.wrapAngle(IR_rawAngle - BNO_OrbitDir);    // Ball ins Roboter-KS
```

> **Achtung – funktioniert nur für `Angle >= -180`:** `fmod` in C übernimmt
> das Vorzeichen des Dividenden. Für `Angle < -180` bleibt der Wert deshalb
> unverändert stehen, statt gewrappt zu werden:
>
> | Eingabe | `wrapAngle()` liefert | korrekt wäre |
> |---|---|---|
> | `340` | `-20` | `-20` ✓ |
> | `200` | `-160` | `-160` ✓ |
> | `-200` | `-200` | `160` ✗ |
> | `-340` | `-340` | `20` ✗ |
>
> Das trifft genau den Fall „Differenz zweier ±180°-Winkel", z.B.
> `-170 - 170 = -340`. Korrekte Variante:
>
> ```cpp
> float r = fmod(Angle + 180.0f, 360.0f);
> if (r < 0.0f) r += 360.0f;
> return r - 180.0f;
> ```

---

## Beispiel

```cpp
#include <Utils.h>

void loop() {
    float diff = Util.wrapAngle(soll - ist);      // kürzester Drehweg, ±180°

    if (Util.inRange(diff, -5.0f, 5.0f)) {
        // ausgerichtet
    }
}
```

---

## Erweiterung

Neue Hilfsfunktionen gehören hierher, wenn sie in mehr als einer Bibliothek
gebraucht werden. Konventionen aus [`lib/README`](../README):

- genau zwei Dateien (`.h` und `.cpp`)
- eindeutige Funktionsnamen
- `give`- und `set`-Funktionen für Zugriffe

Für Winkelrechnungen im Bogenmaß gibt es zusätzlich `PID::wrapPi()` bzw.
`ADRC::wrapPi()` in den [Regler-Bibliotheken](../Regler/PID/README.md).
