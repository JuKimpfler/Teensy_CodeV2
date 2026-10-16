# TCA9534DW – 8-Bit I²C-Portexpander

Treiber für den TI **TCA9534(DW)**: 8 frei als Ein- oder Ausgang
konfigurierbare Pins über I²C. Im Roboter für Motor-Richtungs-/Enable-Signale
sowie für Schalter und Taster im Einsatz.

Dateien: `TCA9534DW.h`, `TCA9534DW.cpp`

---

## Konfiguration

### Instanziierung

```cpp
TCA9534DW Exp_MotorB(&Wire1, 0x20);
```

| Parameter | Bedeutung |
|---|---|
| `i2c` | Zeiger auf die `TwoWire`-Instanz |
| `address` | I²C-Adresse `0x20`…`0x27`, per A0/A1/A2-Pins eingestellt |

`Wire.begin()` muss vor `begin()` erfolgt sein.

### Richtung festlegen

```cpp
expander.begin(true);   // alle 8 Pins als Eingang
expander.begin(false);  // alle 8 Pins als Ausgang

expander.begin(true, true, false, false, false, false, false, false);
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
| Configuration | `0x03` | `begin()` |

---

## API

| Funktion | Beschreibung |
|---|---|
| `void begin(bool Input)` | Alle Pins gemeinsam als Ein- (`true`) oder Ausgang (`false`) |
| `void begin(bool pin0 … bool pin7)` | Richtung je Pin einzeln |
| `void write(uint8_t value)` | Ganzes Ausgangsbyte schreiben, `_state` wird mitgeführt |
| `void writePin(uint8_t pin, bool value)` | Einzelnes Bit in `_state` setzen und das ganze Byte schreiben |
| `uint8_t read()` | Eingangsbyte lesen und in `_state` ablegen. `0xFF` (−1) wenn keine Daten |
| `uint8_t readPin(uint8_t pin)` | Liest das Byte und gibt Bit `pin` zurück |
| `void update()` | Eingangsport **einmal** lesen und merken (`_state = read()`). Bei Lesefehler gelten alle Pins als 1, wie bisher bei `readPin()` |
| `bool givePinState(uint8_t pin) const` | Bit aus dem **zwischengespeicherten** `_state`, ohne I²C-Zugriff |

**Wichtig:** `_state` wird sowohl von `write()` als auch von `read()`
beschrieben. Ein Expander sollte deshalb entweder als reiner Ausgang oder als
reiner Eingang betrieben werden – bei gemischtem Betrieb überschreiben sich
Ausgabezustand und Eingangsbild gegenseitig.

**Performance:** `readPin()` löst pro Aufruf eine vollständige I²C-Transaktion
aus (bei 100 kHz ≈ 0,4 ms). Für mehrere Pins immer einmal `update()` und danach
`givePinState()` verwenden – so macht es `systemlib::readExpanders()`, siehe
[Laufzeit_Optimierung.md](../../docs/Laufzeit_Optimierung.md).

---

## Verwendung im Projekt

Drei Expander am Teensy-Bus `Wire1` (siehe `systemlib.cpp`, Schaltplan Blatt 4):

| Instanz | Adresse | Eingänge | Ausgänge |
|---|---|---|---|
| `Exp_MotorA` | `0x22` | – | Richtung, Enable und Bremse von Motor 1 (VL) und 2 (HL); Enable und Bremse des Dribblers |
| `Exp_MotorB` | `0x20` | `SWITCH1` (P6), `BUTTON1` (P7) | Richtung, Enable und Bremse von Motor 3 (VR) und 4 (HR) |
| `Exp_System` | `0x23` | `UART_DIO1`, `LINE_EN_T` (solange nicht getrieben) | `VMOT_EN`, `VDRIB_EN`, `LIGHT_BARRIER_EN`, `I2C_INTERCON_3V3_EN` |

Die Bitzuordnung steht in `enum.h` ([Tabelle](../include/include.md)); Bit n ist Pin Pn, es wird nicht gespiegelt.
`write()`/`writePin()` und `update()`/`read()` benutzen getrennte Register (`_outState` und `_state`): Ein Schreiben
überschreibt nie den zwischengespeicherten Eingangszustand, den `givePinState()` liefert.

---

## Beispiel

```cpp
#include <Wire.h>
#include <TCA9534DW.h>

TCA9534DW out(&Wire, 0x20);
TCA9534DW in (&Wire, 0x23);

void begin() {
    Wire.begin();
    Wire.setClock(400000);

    out.begin(false);   // alle Ausgang
    in.begin(true);     // alle Eingang
}

void loop() {
    out.write(0b10101010);
    out.writePin(0, true);

    in.read();                      // ein I²C-Zugriff
    bool b0 = in.givePinState(0);    // danach ohne Bus-Zugriff
    bool b1 = in.givePinState(1);
}
```

Testprogramm: `test/Test-TCA9534DW.cpp`

---

## Bekannte Baustellen

- `read()` gibt bei Fehler `-1` zurück, der Rückgabetyp ist aber `uint8_t` →
  daraus wird `0xFF`, was von einem gültigen „alle Pins high" nicht zu
  unterscheiden ist.
- Schlägt das Lesen fehl, setzt `readPin()` `_state` auf `0xFF`. Ein
  anschließendes `givePinState()` meldet dann für alle Pins `true`.
- Das Polarity-Inversion-Register (`0x02`) wird nicht unterstützt.
