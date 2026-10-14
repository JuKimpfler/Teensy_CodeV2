# fancyBTN – Taster-Auswertung mit Klick, Doppelklick und Long-Press

Wertet einen booleschen Tastereingang aus und liefert einen Statuscode:
gedrückt, neu gedrückt, losgelassen, Klick, Doppelklick, Long-Press.

Die Bibliothek liest **keinen Pin selbst** – der Zustand wird von außen
übergeben. Dadurch funktioniert sie gleichermaßen mit `digitalRead()` und mit
Portexpandern wie dem [TCA9534DW](../TCA9534DW/TCA9534DW.md).

Dateien: `fancyBTN.h`, `fancyBTN.cpp`

---

## Konfiguration

```cpp
fancyBTN BTN1(50, 6, 8, true);
```

| Parameter | Typ | Einheit | Bedeutung |
|---|---|---|---|
| `sensitivity` | `uint8_t` | **× 2 ms** | Zeitfenster für den Doppelklick. `0` = keine Doppelklick-Erkennung, `255` = 510 ms |
| `longPressTime` | `uint8_t` | **× 500 ms** | Haltedauer für Long-Press. `0` = keine Long-Press-Erkennung, `255` = 127,5 s |
| `debounceTime` | `uint8_t` | ms | Entprellzeit; Flanken innerhalb dieser Zeit lösen keinen Doppelklick aus |
| `InputWhenOnBTNdown` | `bool` | – | `true`: Eingang ist `HIGH` bei gedrücktem Taster. `false`: invertiert (Pull-Up-Verdrahtung) |

Beispiel oben: Doppelklick-Fenster 100 ms, Long-Press ab 3 s, Entprellung
8 ms, aktiv-High.

> **Achtung:** Die Defaultwerte für `debounceTime` und `InputWhenOnBTNdown`
> stehen in der `.cpp`-Definition, nicht in der Deklaration im Header. Sie
> gelten deshalb außerhalb von `fancyBTN.cpp` nicht – von außen müssen immer
> **alle vier** Argumente übergeben werden.

---

## Statuscodes

`giveStatusCode()` liefert (Enum `fancyBTNStatusCode`):

| Wert | Name | Bedeutung |
|---|---|---|
| 0 | `NOT_PRESSED` | Taster nicht gedrückt (Ruhezustand) |
| 1 | `NEW_PRESSED` | Flanke: gerade gedrückt |
| 2 | `PRESSED` | wird gehalten |
| 3 | `NEW_RELEASED` | Flanke: gerade losgelassen |
| 4 | `fancy_CLICKED` | kurzer Klick (nach Ablauf des Doppelklick-Fensters bestätigt) |
| 5 | `fancy_DOUBLE_CLICK` | zweiter Druck innerhalb des Doppelklick-Fensters |
| 6 | `fancy_LONG_PRESS` | beim Loslassen war die Haltedauer > `longPressTime × 500 ms` |

Zeitlicher Ablauf:

- **Klick** wird erst gemeldet, wenn das Doppelklick-Fenster abgelaufen ist –
  ein Klick kommt also mit `sensitivity × 2 ms` Verzögerung.
- **Doppelklick** unterdrückt den anschließenden Long-Press.
- **Long-Press** wird beim Loslassen gemeldet und unterdrückt den
  anschließenden Doppelklick.

---

## API

| Funktion | Beschreibung |
|---|---|
| `void update(bool buttonState)` | **Jeden Zyklus aufrufen.** `buttonState` = roher Eingangszustand |
| `uint8_t giveStatusCode() const` | Aktueller Statuscode (siehe Tabelle) |
| `bool isPressed() const` | Interner `_buttonState` |

`update()` arbeitet mit `millis()`, es wird nicht geblockt.

---

## Verwendung im Projekt

[`systemlib`](../systemlib/systemlib.md) legt fünf Instanzen an, alle mit
`(50, 6, 8, true)`, und speist sie aus dem Button-Expander:

```cpp
BTN1.update(Exp_MotorB.givePinState(BUTTON1_BIT)); // Platine V2.0: BUTTON1
...
fancyBTNstates[0] = BTN1.giveStatusCode();
```

Abholen im Hauptprogramm:

```cpp
uint8_t states[5];
sys.giveButtonStates(states);
```

Auf einem PDS-Kanal liegt bisher nur der Startschalter (Kanal 25, siehe
[`docs/PDS_Belegung.md`](../../docs/PDS_Belegung.md)). Für die fünf Taster sind 26–31 frei; sie lassen sich im Sketch auch
ohne feste Nummer mit `PDS.plot("Taster_1", buttons[0])` anzeigen.

---

## Beispiel

```cpp
#include <fancyBTN.h>

fancyBTN btn(50, 6, 8, true);   // 100ms Doppelklick, 3s Long-Press

void setup() { pinMode(2, INPUT_PULLDOWN); }

void loop() {
    btn.update(digitalRead(2));

    switch (btn.giveStatusCode()) {
        case fancy_CLICKED:       /* ... */ break;
        case fancy_DOUBLE_CLICK:  /* ... */ break;
        case fancy_LONG_PRESS:    /* ... */ break;
    }
}
```

---

## Bekannte Baustellen

- Der Konstruktor initialisiert nur `_sensitivity`, `_longPressTime`,
  `_debounceTime` und `_InputWhenOnBTNdown`. Alle übrigen Member
  (`_lastButtonState`, `_lastTimePressed`, `_statusCode`, …) bleiben
  uninitialisiert – bei globalen Instanzen sind sie durch die statische
  Initialisierung `0`, bei lokalen Instanzen nicht.
- Die Statuscodes werden in `update()` als Zahlenliteral gesetzt, das Enum
  `fancyBTNStatusCode` wird intern nicht verwendet.
