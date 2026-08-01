# PDS – Power Debug System (Telemetrie + Parameter-Downlink)

Zweiwege-Debugschnittstelle über UART:

- **Uplink (Teensy → RPi):** 200 Float-Kanäle, ~100 Hz, Binärpaket
- **Downlink (RPi/GUI → Teensy):** 50 Floats + 50 Bools (langsam) sowie
  5 Floats (schnell, z.B. Joystick)

Da UART vollduplex ist, laufen beide Richtungen unabhängig über dieselbe
Instanz `UART_DBG`.

Dateien: `PDS.h`, `PDS.cpp`, `Channel.md` (Kanalbelegung)

Die Kanalzuordnung steht in **[`Channel.md`](Channel.md)** – dort nachschlagen,
bevor ein Kanal belegt wird.

---

## Konfiguration

Alle Parameter liegen in [`lib/include/params.h`](../include/include.md) bzw.
`enum.h`:

| Konstante | Wert | Bedeutung |
|---|---|---|
| `UART_DBG` | `Serial3` | UART-Instanz für Uplink **und** Downlink |
| `UART_DBG_BAUD` | `1 000 000` | Baudrate (1 Mbps) |
| `ACTIVE_CHANNELS` | `200` | Build-Flag aus `platformio.ini`, Anzahl aktiver Kanäle |

Weitere Werte fest in `PDS.cpp`:

| Konstante | Wert | Bedeutung |
|---|---|---|
| `HEADER_MAGIC` | `0xDEADBEEF` | Paketkennung Uplink |
| `MAX_FLOATS` | `200` | Größe des Debug-Arrays |
| `PACKET_BYTES` | `808` | `8 + 200*4` |
| `SAMPLE_PERIOD_US` | `10` | Sendeintervall – trotz des Namens in **Millisekunden** (`elapsedMillis`), also 100 Hz |
| TX-Puffer | `4096` Byte | via `addMemoryForWrite()` |

> Der Kopfkommentar in `PDS.h` beschreibt noch 400 Kanäle / 1608 Byte und
> 4 Mbps. Maßgeblich ist der Code: **200 Kanäle, 808 Byte, 1 Mbps.**

### Downlink-Parameter (`params.h`)

| Konstante | Wert |
|---|---|
| `PARAM_SLOW_MAGIC` | `0xCAFEFEED` |
| `PARAM_SLOW_FLOAT_COUNT` | `50` |
| `PARAM_SLOW_BOOL_COUNT` | `50` |
| `PARAM_SLOW_PACKET_BYTES` | `258` (= 8 + 50·4 + 50) |
| `PARAM_SLOW_TIMEOUT_MS` | `1000` |
| `PARAM_FAST_MAGIC` | `0xFA57DA7A` |
| `PARAM_FAST_FLOAT_COUNT` | `5` |
| `PARAM_FAST_PACKET_BYTES` | `28` (= 8 + 5·4) |
| `PARAM_FAST_TIMEOUT_MS` | `150` |
| `PARAM_HEADER_BYTES` | `8` (magic 4 + seq 4) |

Die Timeouts bestimmen, ab wann `paramsAreFresh()` / `fastParamsAreFresh()`
`false` liefern. Slow ist auf 2 verpasste Zyklen (2 Hz → 1000 ms) ausgelegt,
Fast bewusst großzügiger als 2 Zyklen (100 Hz → 150 ms statt 20 ms), weil über
WLAN sonst dauernd Fehlalarm entsteht. **Am Feld ggf. nachjustieren.**

---

## Paketformate

### Uplink (Teensy → RPi), Little-Endian

| Offset | Typ | Inhalt |
|---|---|---|
| 0…3 | `uint32` | `0xDEADBEEF` |
| 4…7 | `uint32` | `micros()` |
| 8…807 | `float[200]` | Kanaldaten |

### Downlink Slow (258 Byte)

| Offset | Typ | Inhalt |
|---|---|---|
| 0…3 | `uint32` | `0xCAFEFEED` |
| 4…7 | `uint32` | Sequenznummer (wird nicht ausgewertet) |
| 8…207 | `float[50]` | Parameter |
| 208…257 | `uint8[50]` | Bools (≠0 = true) |

### Downlink Fast (28 Byte)

| Offset | Typ | Inhalt |
|---|---|---|
| 0…3 | `uint32` | `0xFA57DA7A` |
| 4…7 | `uint32` | Sequenznummer |
| 8…27 | `float[5]` | schnelle Parameter |

Der Empfangsparser schiebt ein 4-Byte-Fenster über den Datenstrom, bis eines
der beiden Magics passt; danach steht die erwartete Paketlänge fest. Dadurch
resynchronisiert er sich nach Störungen selbst.

---

## Verdrahtung

```
Teensy TX (Serial3) ──→ RPi UART RX
Teensy RX (Serial3) ←── RPi UART TX
GND                 ───  GND
```

---

## API

| Funktion | Beschreibung |
|---|---|
| `void init()` | Debug-Array nullen, TX-Puffer erweitern, `UART_DBG` mit `8N1` öffnen, Downlink-Arrays initialisieren |
| `void update()` | **Zyklisch aufrufen.** Sendet alle 10 ms ein Paket und pollt nicht-blockierend den Empfang |
| `void Channel(uint8_t chn, float val)` | Kanalwert setzen (0…199) |
| `float getParam(uint8_t index)` | Slow-Float, Index 0…49; außerhalb → `0.0f` |
| `bool getParamBool(uint8_t index)` | Slow-Bool, Index 0…49; außerhalb → `false` |
| `float getFastParam(uint8_t index)` | Fast-Float, Index 0…4; außerhalb → `0.0f` |
| `bool paramsAreFresh()` | Slow-Kanal aktuell? |
| `bool fastParamsAreFresh()` | Fast-Kanal aktuell? |

**Empfangene Werte werden nicht persistiert** (RAM-only). Nach einem Reset
sind alle Parameter `0.0f`/`false`, bis das nächste Paket eintrifft – deshalb
vor der Benutzung immer `paramsAreFresh()` prüfen.

`Channel()` prüft den Index **nicht**. Werte ≥ 200 schreiben über das Array
hinaus. Über [`systemlib::setDBGChannel()`](../systemlib/systemlib.md) sind nur
die Kanäle 0–20 freigegeben.

---

## Beispiel

```cpp
#include <PDS.h>

PowerDebugger PDS;

void setup() {
    PDS.init();
}

void loop() {
    PDS.Channel(0, someValue);
    PDS.Channel(1, anotherValue);

    if (PDS.paramsAreFresh()) {
        float Kp = PDS.getParam(0);       // Live-Tuning aus der GUI
        bool  on = PDS.getParamBool(3);
    }

    PDS.update();   // sendet + empfängt
}
```

---

## Bekannte Baustellen

- `SAMPLE_PERIOD_US` heißt „US", wird aber gegen einen `elapsedMillis`-Timer
  verglichen – der Wert `10` bedeutet 10 ms.
- Das Kommentar-Header in `PDS.h` (400 Kanäle, 1608 Byte, 4 Mbps, Serial1 auf
  Pin 0/1) beschreibt einen älteren Stand.
- `init()` setzt `pinMode(10, INPUT)` – Pin 10 ist der `START_SWITCH` aus
  `enum.h`; das gehört eigentlich nicht in diese Bibliothek.
- Der Dummy-Wert `9898.0f` für inaktive Kanäle ist im Code auskommentiert; es
  wird mit `0` initialisiert.
