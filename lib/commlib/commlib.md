# commlib – Roboter-zu-Roboter-Kommunikation

Datenaustausch zwischen den beiden Robotern im 2vs2-Spiel: eigene Position,
Ball- und Torinformationen senden bzw. die des Partners empfangen.

Dateien: `commlib.h`, `commlib.cpp`, `communication.md` (Protokollnotizen)

> **Status: Gerüst.** Struktur und API stehen, das eigentliche Senden und
> Empfangen ist noch nicht implementiert (`sendOwnFlush()` und der
> Empfangspfad sind TODO). `giveOtherFlush()` liefert derzeit immer den
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
| `RobotDataCom giveOtherFlush() const` | Liefert die zuletzt empfangenen Daten des Partnerroboters. **TODO – es wird nichts empfangen.** |

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

1. **Transportweg festlegen.** Auf der Platine V2.0 ist der Link zum
   ESP32-S3 `UART_ESP` (`Serial6`, Pins 24/25); Kamera-UARTs sind `UART_CAM1/2`, siehe
   [`enum.h`](../include/include.md).
2. **Paketformat.** Sinnvollerweise analog zum
   [Power Debug System](../PDS/PDS.md): Magic-Header + Payload, damit sich
   der Empfänger resynchronisieren kann.
3. **Frische-Erkennung.** Zeitstempel beim Empfang mitschreiben und ein
   `isFresh()` anbieten – sonst rechnet die Taktik mit veralteten Partnerdaten
   weiter (Vorbild: `paramsAreFresh()` im PDS).
4. **Rollenzuordnung.** `Rolle` und `Connected` liegen noch auf keinem
   PDS-Kanal. Sie brauchen auch keine feste Nummer – `PDS.plot("Rolle", r)`
   im Sketch reicht, die Vergabe macht die Bibliothek (siehe
   [`docs/PDS_Belegung.md`](../../docs/PDS_Belegung.md)).

Protokolldetails gehören nach `communication.md`.
