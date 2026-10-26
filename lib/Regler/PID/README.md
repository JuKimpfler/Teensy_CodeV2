# PID – Universeller PID-Regler für Teensy 4.0 / PlatformIO

Eine universell einsetzbare PID-Reglerbibliothek. Regelgesetz:

```
u = Kp*e + Ki*Integral(e dt) + Kd*d(e)/dt,   e = setpoint - measurement
```

Im Vergleich zu einem "Lehrbuch-PID" enthält diese Bibliothek zwei in der
Praxis wichtige Erweiterungen (siehe unten): **Anti-Windup** und
**Derivative-Kick-Vermeidung**. Ansonsten bewusst im gleichen Stil und mit
der gleichen API wie die begleitende `ADRC`-Bibliothek gehalten – beide
lassen sich 1:1 austauschen, um z.B. PID- und ADRC-Regelung für denselben
Regelkreis zu vergleichen.

## Einsatzbereiche

| Anwendung | Konfiguration |
|---|---|
| Winkel-/Heading-Regelung eines omnidirektionalen Roboters | `cfg.angleMode = true` |
| Geschwindigkeitsregelung (linear, Rad, ...) | `cfg.angleMode = false` |

Für Winkelregelung `cfg.angleMode = true` setzen: Fehler und die für die
Ableitung genutzte Messwertdifferenz werden automatisch auf `[-pi, pi]`
gewrappt, sodass der Roboter immer den kürzesten Drehweg nimmt (kein Sprung
bei ±180°).

## Einbindung in PlatformIO

**Option A – lokal im Projekt:**
Ordner `src/` (PID.h/.cpp, TrackingDifferentiator.h/.cpp) nach
`<dein_projekt>/lib/PID/` kopieren. PlatformIO bindet Bibliotheken in `lib/`
automatisch ein, kein Eintrag in `platformio.ini` nötig.

**Option B – als eigenes Git-Repo:**
```ini
lib_deps = https://github.com/DEIN_USER/PID.git
```

## Minimalbeispiel

```cpp
#include "PID.h"

PIDConfig cfg;
PID* ctrl;

void setup() {
    cfg.Kp = 2.0f;
    cfg.Ki = 0.5f;
    cfg.Kd = 0.05f;
    cfg.angleMode = true; // bei Winkelregelung
    ctrl = new PID(cfg);
}

void loop() {
    float u = ctrl->update(sollWinkel, istWinkel, dt); // u in [-1, 1]
    // u auf Motoren mischen ...
}
```

Vollständige, lauffähige Beispiele: `examples/AngleControl` (Ausrichtung
eines 3-Rad-Omnidrives) und `examples/VelocityControl` (lineare
Geschwindigkeitsregelung).

## Variabler Regeltakt (empfohlen bei unregelmäßiger Loop-Dauer)

Ein fester Regeltakt ist **nicht erforderlich**. `update()` gibt es in zwei
Varianten:

```cpp
float update(float setpoint, float measurement, float dt); // empfohlen
float update(float setpoint, float measurement);            // nutzt cfg.Ts fest
```

Bei der `dt`-Variante misst du selbst, wie viel Zeit seit dem letzten Aufruf
vergangen ist, und übergibst sie direkt:

```cpp
uint32_t lastMicros;

void setup() {
    // ... cfg.Kp/Ki/Kd setzen ...
    lastMicros = micros();
}

void loop() {
    uint32_t now = micros();
    float dt = (now - lastMicros) * 1e-6f; // us -> s
    lastMicros = now;

    float u = ctrl->update(sollwert, istwert, dt);
    // ... u ausgeben, restlicher (schneller) Loop-Code ...
}
```

**Wichtig – Begrenzung von `dt`:** Über `cfg.dtMin`/`cfg.dtMax` wird `dt`
intern geclamped:
- `dtMin` verhindert Probleme bei `dt ≈ 0`.
- `dtMax` verhindert, dass ein einzelner ungewöhnlich langer Zyklus (z.B.
  weil dein Hauptprogramm mal durch eine andere Aufgabe blockiert war) dem
  Integrator einen großen Sprung beschert oder die Ableitung stark
  verfälscht. Setze `dtMax` auf die längste Zykluszeit, die im
  Normalbetrieb realistisch vorkommen kann (z.B. 20–50 ms).

## Anti-Windup (konditionale Integration)

Ohne Gegenmaßnahme "bläht" sich der I-Anteil auf, wenn die Stellgröße
länger in der Begrenzung (`uMin`/`uMax`) hängt (z.B. Roboter gegen ein
Hindernis gefahren, oder Sollwertsprung zu groß für die Aktorik) – beim
Verlassen der Sättigung schwingt der Regler dann stark über.

Diese Bibliothek nutzt **konditionale Integration**: Der Integrator wird
nur dann weiter aufaddiert, wenn dies die Sättigung nicht verstärkt. Das
ist kein zusätzlicher Parameter, den du einstellen musst – es funktioniert
automatisch anhand von `uMin`/`uMax`.

## Derivative Kick vermeiden

Ein Sollwertsprung (z.B. neuer Ziel-Winkel) erzeugt bei einem D-Anteil, der
auf dem *Fehler* gebildet wird, kurzzeitig eine sehr große (unphysikalische)
Ableitung – einen "Kick" im Stellsignal. Per Default
(`cfg.derivativeOnMeasurement = true`) wird die Ableitung stattdessen auf
dem *Messwert* gebildet, der sich bei einem Sollwertsprung nicht schlagartig
ändert. Für den (unüblichen) Fall, dass du die klassische Fehler-Ableitung
brauchst, kannst du `derivativeOnMeasurement = false` setzen.

Zusätzlich wird die Ableitung über ein Tiefpassfilter 1. Ordnung geglättet
(Zeitkonstante `cfg.Tf`), da die reine Differenzenbildung sehr empfindlich
auf Sensor-/Messrauschen reagiert. `Tf = 0` schaltet das Filter aus (nicht
empfohlen, außer dein Messsignal ist bereits extrem sauber).

## Tuning-Anleitung

Klassisches, praxisnahes Vorgehen (Ziegler-Nichols-artig, aber manuell und
robuster für mobile Roboter):

1. **Nur P** verwenden (`Ki = Kd = 0`). `Kp` schrittweise erhöhen, bis der
   Regelkreis beginnt zu schwingen bzw. spürbar überschwingt. Danach `Kp`
   wieder auf ca. 50–70 % dieses Werts reduzieren.
2. **`Ki` zuschalten**, klein beginnen und schrittweise erhöhen, bis die
   bleibende Regelabweichung (z.B. durch Reibung, Hangneigung, Asymmetrien)
   in akzeptabler Zeit verschwindet, ohne dass der Regelkreis merklich zu
   schwingen beginnt. Zu groß gewähltes `Ki` äußert sich in langsamen,
   niederfrequenten Schwingungen.
3. **`Kd` optional zuschalten**, um Überschwingen zu dämpfen bzw. schneller
   auf Störungen zu reagieren. Bei verrauschten Messgrößen (z.B. rohe
   Encoder-Differenzen) vorsichtig dosieren und `Tf` ggf. erhöhen – sonst
   wird Sensorrauschen direkt ins Stellsignal verstärkt.
4. **`outputRateLimit`** nutzen, um mechanische Sprünge/Ruckartigkeit zu
   vermeiden (z.B. Rad-Schlupf bei zu hartem Anlauf).

**Faustregeln je Anwendungsfall:**
- Winkel-/Headingregelung: meist `Kp` dominant, kleines `Ki` (kompensiert
  z.B. asymmetrische Reibung zwischen den Omniwheels), kleines bis
  moderates `Kd` zur Dämpfung.
- Geschwindigkeitsregelung: `Kp` + `Ki` reichen meist aus; `Kd = 0` ist bei
  verrauschten Geschwindigkeitsmessungen (Encoder-Differenzen) oft die
  bessere Wahl.

## Erweiterte Optionen

- `TrackingDifferentiator`: optionales Vorfilter, das harte
  Sollwertsprünge in einen glatten Verlauf überführt, bevor sie dem PID
  als Sollwert übergeben werden. Reduziert zusätzlich zur eingebauten
  Derivative-Kick-Vermeidung ruckartige Stellgrößensprünge direkt nach
  einem Sollwertwechsel.

## Wichtige Hinweise

- Für jede zu regelnde Größe (z.B. Heading UND Geschwindigkeit
  gleichzeitig) eine **eigene `PID`-Instanz** anlegen – die Klasse ist
  bewusst zustandsbehaftet und pro Regelkreis gedacht.
- `reset()` beim Start bzw. nach einer Zwangspause/größeren
  Diskontinuität (z.B. Roboter manuell gedreht) aufrufen, damit
  Integrator und Ableitung nicht von einem falschen Zustand aus starten.
- Diese Bibliothek benötigt (im Gegensatz zu ADRC) ein grobes Gefühl für
  die Streckendynamik beim manuellen Tuning von `Kp`/`Ki`/`Kd` – sie
  schätzt keine Störungen/Modellfehler automatisch. Bei stark
  schwankenden Störeinflüssen (z.B. wechselnder Bodenbelag, variable
  Zuladung) kann die begleitende `ADRC`-Bibliothek robuster sein.
