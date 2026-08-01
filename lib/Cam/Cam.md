# Cam – OpenMV-Kameraanbindung (Tor-Erkennung über UART)

Liest Zeilen von einer OpenMV-Kamera über UART, parst daraus Tor-Winkel und
Blob-Höhe und rechnet den Kamerawinkel in das **Feld-Koordinatensystem** um.

Dateien: `Cam.h`, `Cam.cpp`

> **Status:** Die IMU-Kopplung ist noch nicht verdrahtet – in `Update()` steht
> `int imu_deg = 0;` fest. Solange liefert `give_Angle()` denselben Wert wie
> `give_Angle_Cam()`. Siehe [Bekannte Baustellen](#bekannte-baustellen).

---

## Konfiguration

### Initialisierung

```cpp
CamC cam;
cam.init(Serial1, 115200);
```

| Parameter | Default | Bedeutung |
|---|---|---|
| `ser` | `Serial1` | HardwareSerial-Port, an dem die OpenMV hängt |
| `baud` | `115200` | Baudrate; muss zur OpenMV-Firmware passen |

Im Projekt sind für die Kamera die Pins `TNS_TX_PIXY_RX = 14` /
`TNS_RX_PIXY_TX = 15` vorgesehen (siehe [`enum.h`](../include/include.md)) –
das entspricht `Serial3` auf dem Teensy 4.0. Der Default `Serial1` ist also
ggf. anzupassen.

### Vorzeichen-Konvention

```cpp
cam.setSign(true);   // field = imu + cam   (Default)
cam.setSign(false);  // field = imu - cam
```

Richtig eingestellt ist es, wenn `give_Angle()` **konstant bleibt**, während
sich der Roboter auf der Stelle dreht.

### Compile-Zeit-Konstanten

| Konstante | Wert | Bedeutung |
|---|---|---|
| `LINE_BUF_SIZE` | `96` | max. Zeilenlänge; längere Zeilen werden verworfen |
| Gültigkeitsfenster in `Update()` | `250 ms` | älter → kein Update des Feldwinkels |
| Default in `isValid()` | `200 ms` | max. Alter, das noch als „gültig" zählt |

---

## Protokoll (OpenMV → Teensy)

Zeilenbasiert, mit `\n` abgeschlossen (ein optionales `\r` wird entfernt).

**Tor erkannt:**

```
A,<goal>,<angle_cam_deg>,<blob_h_px>
```

Beispiel: `A,B,-12.34,37`

| Feld | Typ | Bedeutung |
|---|---|---|
| `A` | Kennung | Datensatz gültig |
| `<goal>` | 1 Zeichen | Tor-Kennung (z.B. `B`/`Y`); nur das erste Zeichen wird ausgewertet |
| `<angle_cam_deg>` | float | Winkel relativ zur Kamera-Blickrichtung, Grad |
| `<blob_h_px>` | int | Höhe des Blobs in Pixeln (grobes Distanzmaß) |

**Kein Tor erkannt:** Zeile beginnt mit `N` → `rawValid = false`.

Ein Datensatz wird nur übernommen, wenn **alle vier Felder** vorhanden sind.

---

## API

| Funktion | Beschreibung |
|---|---|
| `void init(HardwareSerial& ser, uint32_t baud)` | UART öffnen, Parser zurücksetzen |
| `void Update()` | **Muss zyklisch aufgerufen werden.** Pollt den UART, parst Zeilen und berechnet den Feldwinkel |
| `float give_Angle() const` | Tor-Peilung im Feld-KS, Grad, −180…180 |
| `float give_Angle_Cam() const` | Winkel relativ zur Kamera, Grad |
| `int give_BlobH() const` | Blob-Höhe in Pixeln |
| `bool isValid(uint32_t maxAgeMs = 200) const` | `true`, wenn zuletzt ein gültiger Datensatz jünger als `maxAgeMs` empfangen wurde |
| `void setSign(bool usePlus)` | Vorzeichen-Konvention, siehe oben |

`Update()` blockiert nicht: es wird nur gelesen, was im UART-Puffer liegt.
Bei 20–60 Hz Kamerarate reicht ein Aufruf pro Hauptschleifen-Durchlauf.

---

## Beispiel

```cpp
#include "Cam.h"

CamC cam;

void setup() {
    cam.init(Serial3, 115200);
}

void loop() {
    cam.Update();

    if (cam.isValid()) {
        float bearing = cam.give_Angle();   // -180..180
        int   height  = cam.give_BlobH();
    }
}
```

---

## Bekannte Baustellen

- **IMU nicht angebunden:** `Update()` verwendet `imu_deg = 0`. Für die
  eigentliche Funktion muss hier der Heading aus
  [`systemlib`](../systemlib/systemlib.md) (`giveOrbitDir()`) bzw. dem
  [BNO055](../BNO055/BNO055.md) eingesetzt werden. Der `#include "BNO055.h"`
  in `Cam.cpp` ist dafür schon vorhanden, wird aber nicht genutzt.
- Das Feld `goal` wird geparst und gespeichert, aber nach außen nicht
  zugänglich gemacht (kein Getter).
- `CamC::wrap180` ist deklariert, die Implementierung liegt als freie Funktion
  `wrap180f` in der `.cpp`.
- `Cam` wird derzeit nicht von `systemlib` eingebunden.
