# commlib – Roboter-zu-Roboter-Kommunikation

Datenaustausch zwischen den beiden Robotern im 2vs2-Spiel: eigene Position,
Ball- und Torinformationen senden bzw. die des Partners empfangen.

Dateien: `commlib.h`, `commlib.cpp`, `communication.md` (Protokollnotizen)

> **Status: Gerüst.** Struktur und API stehen, das eigentliche Senden und
> Empfangen ist noch nicht implementiert (`sendOwnFlush()` und der
> Empfangspfad sind TODO). `getOtherFlush()` liefert derzeit immer den
> uninitialisierten Puffer zurück.

---

## Datenstruktur

```cpp
struct RobotDataCom
{
    float positionX;     // eigene Position auf dem Feld
    float positionY;
    float BallAngle;     // Winkel zum Ball
    float BallDistance;  // Entfernung zum Ball
    float GoalAngle;     // Winkel zum Tor
    float GoalDistance;  // Entfernung zum Tor
};
```

Sechs `float`, also **24 Byte** Nutzdaten pro Paket. Einheiten sind noch nicht
festgelegt – im Projekt werden Winkel sonst durchgängig in Grad (±180°, siehe
[`Utils`](../Utils/Utils.md)) geführt, das ist die naheliegende Konvention.

---

## API

| Funktion | Beschreibung |
|---|---|
| `void setOwnFlush(float x, float y, float ballAngle, float ballDistance, float goalAngle, float goalDistance)` | Füllt den eigenen Sendepuffer. Sendet noch nicht. |
| `void sendOwnFlush()` | Verschickt den Sendepuffer. **TODO – aktuell leer.** |
| `RobotDataCom getOtherFlush() const` | Liefert die zuletzt empfangenen Daten des Partnerroboters. **TODO – es wird nichts empfangen.** |

---

## Verwendung im Projekt

[`systemlib`](../systemlib/systemlib.md) hält die Instanz `communication` und
kapselt sie:

```cpp
sys.communicationSend();                       // sendet den aktuell gesetzten Puffer
sys.communicationSend(x, y, bA, bD, gA, gD);   // setzt und sendet in einem Schritt

RobotDataCom partner = sys.giveOtherRobotData();
```

---

## Offene Punkte für die Implementierung

Damit die Bibliothek nutzbar wird, fehlen im Wesentlichen:

1. **Transportweg festlegen.** Kandidaten laut Pinbelegung
   ([`enum.h`](../include/include.md)): `UART1_TNS_TX/RX` (Pins 24/25) oder
   `UART2_TNS_TX/RX` (Pins 28/29), alternativ ein Funkmodul.
2. **Paketformat.** Sinnvollerweise analog zum
   [Power Debug System](../Debugger/PDS.md): Magic-Header + Payload, damit sich
   der Empfänger resynchronisieren kann.
3. **Frische-Erkennung.** Zeitstempel beim Empfang mitschreiben und ein
   `isFresh()` anbieten – sonst rechnet die Taktik mit veralteten Partnerdaten
   weiter (Vorbild: `paramsAreFresh()` im PDS).
4. **Rollenzuordnung.** Debug-Kanal 46 (`Rolle`) und 47 (`Connected`) sind in
   [`Channel.md`](../Debugger/Channel.md) bereits reserviert.

Protokolldetails gehören nach `communication.md`.
