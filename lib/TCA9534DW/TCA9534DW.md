# TCA9534DW – 8-Bit I²C-Portexpander

Treiber für den TI **TCA9534(DW)**: 8 frei als Ein- oder Ausgang
konfigurierbare Pins über I²C. Im Roboter für Motor-Richtungs-/Enable-Signale
sowie für Schalter und Taster im Einsatz.

Dateien: `TCA9534DW.h`, `TCA9534DW.cpp`

---

## Konfiguration

### Instanziierung

```cpp
TCA9534DW Motor_EXP(&Wire1, 0x20);
```

| Parameter | Bedeutung |
|---|---|
| `i2c` | Zeiger auf die `TwoWire`-Instanz |
| `address` | I²C-Adresse `0x20`…`0x27`, per A0/A1/A2-Pins eingestellt |

`Wire.begin()` muss vor `setup()` erfolgt sein.

### Richtung festlegen

```cpp
expander.setup(true);   // alle 8 Pins als Eingang
expander.setup(false);  // alle 8 Pins als Ausgang

expander.setup(true, true, false, false, false, false, false, false);
//             pin0  pin1  pin2 ...   → true = Eingang, false = Ausgang
```

Die Richtung wird ins Configuration-Register `0x03` geschrieben
(`1` = Eingang, `0` = Ausgang).

### Register

| Register | Adresse | Verwendung im Code |
|---|---|---|
| Input Port | `0x00` | `read()`, `readPin()` |
| Output Port | `0x01` | `write()`, `writePin()` |
| Polarity Inversion | `0x02` | *nicht genutzt* |
| Configuration | `0x03` | `setup()` |

---

## API

| Funktion | Beschreibung |
|---|---|
| `void setup(bool Input)` | Alle Pins gemeinsam als Ein- (`true`) oder Ausgang (`false`) |
| `void setup(bool pin0 … bool pin7)` | Richtung je Pin einzeln |
| `void write(uint8_t value)` | Ganzes Ausgangsbyte schreiben, `_state` wird mitgeführt |
| `void writePin(uint8_t pin, bool value)` | Einzelnes Bit in `_state` setzen und das ganze Byte schreiben |
| `uint8_t read()` | Eingangsbyte lesen und in `_state` ablegen. `0xFF` (−1) wenn keine Daten |
| `uint8_t readPin(uint8_t pin)` | Liest das Byte und gibt Bit `pin` zurück |
| `bool getPinState(uint8_t pin) const` | Bit aus dem **zwischengespeicherten** `_state`, ohne I²C-Zugriff |

**Wichtig:** `_state` wird sowohl von `write()` als auch von `read()`
beschrieben. Ein Expander sollte deshalb entweder als reiner Ausgang oder als
reiner Eingang betrieben werden – bei gemischtem Betrieb überschreiben sich
Ausgabezustand und Eingangsbild gegenseitig.

**Performance:** `readPin()` löst pro Aufruf eine vollständige I²C-Transaktion
aus. Für mehrere Pins besser einmal `read()` und danach `getPinState()`
verwenden.

---

## Verwendung im Projekt

Fünf Expander (siehe `systemlib.cpp`, Adressen in `params.h`):

| Instanz | Bus | Adresse | Richtung | Zweck |
|---|---|---|---|---|
| `Motor12_EXP` | `I2C_MAIN` (`Wire1`) | `0x22` | Ausgang | Main IC9: ESCON MO1, MO2, Dribbler |
| `Motor34_EXP` | `I2C_MAIN` (`Wire1`) | `0x20` | Ausgang, P6/P7 Eingang | Main IC7: ESCON MO3, MO4, SWITCH1/BUTTON1 |
| `Power_EXP` | `I2C_MAIN` (`Wire1`) | `0x23` | Ausgang | Main IC8: VMOT, VDRIB, Linien-LEDs, Lichtschranke, Intercon |
| `SW_EXP` | `I2C_SW` (`Wire`) | `0x26` | Eingang | SwitchBoard IC1: SW1–5, Start, RCJ |
| `Button_EXP` | `I2C_SW` (`Wire`) | `0x27` | Eingang | SwitchBoard IC2: Taster → [fancyBTN](../fancyBTN/fancyBTN.md), RCJ-Out |

Die ESCON-Bits der beiden Motor-Expander kommen aus `enum.h`
([Doku](../include/include.md#rad--escon-modul)) als Bitnummer 0…15:
Bit 0…7 gehen an `Motor12_EXP`, Bit 8…15 an `Motor34_EXP`. Je Modul liegen
Freigabe (ESCON DI2), Richtung (DI3) und Stopp (DI4) direkt hintereinander,
z.B. MO1 = Bit 0/1/2. `systemlib::updateMotors()` schreibt beide Bytes ohne
Spiegelung.

---

## Beispiel

```cpp
#include <Wire.h>
#include <TCA9534DW.h>

TCA9534DW out(&Wire, 0x20);
TCA9534DW in (&Wire, 0x23);

void setup() {
    Wire.begin();
    Wire.setClock(400000);

    out.setup(false);   // alle Ausgang
    in.setup(true);     // alle Eingang
}

void loop() {
    out.write(0b10101010);
    out.writePin(0, true);

    in.read();                      // ein I²C-Zugriff
    bool b0 = in.getPinState(0);    // danach ohne Bus-Zugriff
    bool b1 = in.getPinState(1);
}
```

Testprogramm: `test/Test-TCA9534DW.cpp`

---

## Bekannte Baustellen

- `read()` gibt bei Fehler `-1` zurück, der Rückgabetyp ist aber `uint8_t` →
  daraus wird `0xFF`, was von einem gültigen „alle Pins high" nicht zu
  unterscheiden ist.
- Schlägt das Lesen fehl, setzt `readPin()` `_state` auf `0xFF`. Ein
  anschließendes `getPinState()` meldet dann für alle Pins `true`.
- Das Polarity-Inversion-Register (`0x02`) wird nicht unterstützt.
