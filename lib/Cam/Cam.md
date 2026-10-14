# Cam – OpenMV-Kameraanbindung (Tor-Erkennung über UART)

Liest Zeilen von einer OpenMV-Kamera über UART, parst daraus Tor-Winkel und
Blob-Höhe und rechnet den Kamerawinkel in das **Feld-Koordinatensystem** um.

Dateien: `Cam.h`, `Cam.cpp`

> **Status:** Die IMU-Kopplung ist noch nicht verdrahtet – in `update()` steht
> `int imu_deg = 0;` fest. Solange liefert `giveAngle()` denselben Wert wie
> `giveAngleCam()`. Siehe [Bekannte Baustellen](#bekannte-baustellen).

---

## Konfiguration

### Initialisierung

```cpp
Cam cam;
cam.begin(Serial1, 115200);
```

| Parameter | Default | Bedeutung |
|---|---|---|
| `ser` | – (Pflicht) | HardwareSerial-Port, an dem die OpenMV hängt: `UART_CAM1` (`Serial2`) oder `UART_CAM2` (`Serial7`) |
| `baud` | `115200` | Baudrate; muss zur OpenMV-Firmware passen |

Auf der Platine V2.0 hängen die beiden Kameras an `UART_CAM1` (`Serial2`, Pins 7/8) und `UART_CAM2`
(`Serial7`, Pins 28/29), siehe [`enum.h`](../include/include.md). Es gibt keinen Standardport mehr, `Serial1` gehört dem PDS.

### Vorzeichen-Konvention

```cpp
cam.setSign(true);   // field = imu + cam   (Default)
cam.setSign(false);  // field = imu - cam
```

Richtig eingestellt ist es, wenn `giveAngle()` **konstant bleibt**, während
sich der Roboter auf der Stelle dreht.

### Compile-Zeit-Konstanten

| Konstante | Wert | Bedeutung |
|---|---|---|
| `LINE_BUF_SIZE` | `96` | max. Zeilenlänge; längere Zeilen werden verworfen |
| Gültigkeitsfenster in `update()` | `250 ms` | älter → kein Update des Feldwinkels |
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
| `void begin(HardwareSerial& ser, uint32_t baud)` | UART öffnen, Parser zurücksetzen |
| `void update()` | **Muss zyklisch aufgerufen werden.** Pollt den UART, parst Zeilen und berechnet den Feldwinkel |
| `float giveAngle() const` | Tor-Peilung im Feld-KS, Grad, −180…180 |
| `float giveAngleCam() const` | Winkel relativ zur Kamera, Grad |
| `int giveBlobH() const` | Blob-Höhe in Pixeln |
| `bool isValid(uint32_t maxAgeMs = 200) const` | `true`, wenn zuletzt ein gültiger Datensatz jünger als `maxAgeMs` empfangen wurde |
| `void setSign(bool usePlus)` | Vorzeichen-Konvention, siehe oben |

`update()` blockiert nicht: es wird nur gelesen, was im UART-Puffer liegt.
Bei 20–60 Hz Kamerarate reicht ein Aufruf pro Hauptschleifen-Durchlauf.

---

## Beispiel

```cpp
#include "Cam.h"

Cam cam;

void setup() {
    cam.begin(Serial3, 115200);
}

void loop() {
    cam.update();

    if (cam.isValid()) {
        float bearing = cam.giveAngle();   // -180..180
        int   height  = cam.giveBlobH();
    }
}
```

---

## Bekannte Baustellen

- **IMU nicht angebunden:** `update()` verwendet `imu_deg = 0`. Für die
  eigentliche Funktion muss hier der Heading aus
  [`systemlib`](../systemlib/systemlib.md) (`giveOrbitDir()`) bzw. dem
  [BNO055](../BNO055/BNO055.md) eingesetzt werden. Der `#include "BNO055.h"`
  in `Cam.cpp` ist dafür schon vorhanden, wird aber nicht genutzt.
- Das Feld `goal` wird geparst und gespeichert, aber nach außen nicht
  zugänglich gemacht (kein Getter).
- `Cam::wrap180` ist deklariert, die Implementierung liegt als freie Funktion
  `wrap180f` in der `.cpp`.
- `Cam` wird derzeit nicht von `systemlib` eingebunden.
