# ADRC – Active Disturbance Rejection Control für Teensy 4.0 / PlatformIO

Eine universell einsetzbare ADRC-Reglerbibliothek. Kernidee von ADRC: statt ein
exaktes Streckenmodell zu benötigen (wie z.B. bei klassischer PID-Regelung mit
Feedforward), schätzt ein **Extended State Observer (ESO)** laufend die
"totale Störung" (unmodellierte Dynamik, Reibung, Kopplungen, externe
Störungen, Parameterfehler ...) und kompensiert sie aktiv im Stellsignal.
Dadurch ist der Regler sehr robust gegenüber Modellunsicherheiten – ideal für
mobile Roboter, bei denen Reibung, Bodenbeschaffenheit, Batteriespannung etc.
schwer exakt modellierbar sind.

## Zwei Betriebsarten

| `ADRCOrder` | Streckenmodell     | Typischer Einsatz |
|---|---|---|
| `FIRST`  | y' = b0·u + f(t)  | Geschwindigkeitsregelung (lineare Fahrgeschwindigkeit, Radgeschwindigkeit) |
| `SECOND` | y'' = b0·u + f(t) | Winkel-/Positionsregelung (Ausrichtung auf einen Soll-Winkel) |

Für Winkelregelung `cfg.angleMode = true` setzen: Regelabweichung und
ESO-Innovation werden dann automatisch auf `[-pi, pi]` gewrappt, sodass der
Roboter immer den kürzesten Drehweg nimmt (kein Sprung bei ±180°).

## Einbindung in PlatformIO

**Option A – lokal im Projekt:**
Ordner `src/` (ADRC.h/.cpp, TrackingDifferentiator.h/.cpp) nach
`<dein_projekt>/lib/ADRC/` kopieren. PlatformIO bindet Bibliotheken in `lib/`
automatisch ein, kein Eintrag in `platformio.ini` nötig.

**Option B – als eigenes Git-Repo:**
Diesen Ordner in ein eigenes Repository packen und in `platformio.ini`
referenzieren:
```ini
lib_deps = https://github.com/DEIN_USER/ADRC.git
```

## Minimalbeispiel

```cpp
#include "ADRC.h"

ADRCConfig cfg;
ADRC* ctrl;

void setup() {
    cfg.order = ADRCOrder::SECOND;  // Winkelregelung
    cfg.Ts    = 0.005f;             // 200 Hz
    cfg.wc    = 15.0f;
    cfg.wo    = 80.0f;
    cfg.b0    = 8.0f;
    cfg.angleMode = true;
    ctrl = new ADRC(cfg);
}

void loop() {
    float u = ctrl->update(sollWinkel, istWinkel); // u in [-1, 1]
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
    // ... cfg.wc/wo/b0 setzen ...
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

Intern nutzt der ESO eine explizite Euler-Integration, die pro Aufruf direkt
mit dem übergebenen `dt` rechnet – ein fester Takt ist dafür nicht nötig,
solange `dt` in etwa die tatsächlich vergangene Zeit widerspiegelt.

**Wichtig – Begrenzung von `dt`:** Über `cfg.dtMin`/`cfg.dtMax` wird `dt`
intern geclamped, bevor es verwendet wird:
- `dtMin` verhindert Probleme bei `dt ≈ 0` (z.B. sehr kurz aufeinander-
  folgende Aufrufe).
- `dtMax` verhindert, dass ein einzelner ungewöhnlich langer Zyklus (z.B.
  weil dein Hauptprogramm mal durch eine andere Aufgabe blockiert war) die
  Euler-Integration im ESO "überschießen" lässt und den Regler kurzzeitig
  instabil macht. Setze `dtMax` auf die längste Zykluszeit, die im
  Normalbetrieb realistisch vorkommen kann (z.B. 20–50 ms) – tritt dann mal
  ein längerer Ausreißer auf, wird einfach mit dem gedeckelten Wert
  weitergerechnet, statt dass der Beobachter "explodiert".

Ein paar Prozent Jitter in `dt` (normales Verhalten eines nicht-echtzeit-
fähigen Hauptprogramms) sind unproblematisch – ADRC ist strukturell robust
gegenüber solchen Schwankungen, da die Störschätzung im ESO ohnehin laufend
nachkorrigiert. Kritisch wird es erst bei sehr großen, seltenen Ausreißern
in der Zykluszeit – dafür ist `dtMax` da.

## Tuning-Anleitung

Nur 3 Parameter müssen eingestellt werden:

1. **`b0`** – grobe Schätzung, wie stark die Stellgröße auf die
   (Winkel-)Beschleunigung bzw. Geschwindigkeitsänderung wirkt. Faustregel
   zum Ermitteln: Stellgröße sprunghaft ändern, resultierende
   Beschleunigung messen, `b0 ≈ Δbeschleunigung / Δu`. ADRC ist relativ
   tolerant gegenüber Fehlern in `b0` (Faktor 2–3 daneben ist meist noch
   ok), aber je genauer, desto besser die Performance.

2. **`wo`** (Beobachterbandbreite) – zuerst einstellen. Bestimmt, wie
   schnell der ESO Zustand und Störung schätzt. Zu klein → träge,
   überschwingende Regelung. Zu groß → Rauschverstärkung (Messrauschen
   wird direkt in die Störschätzung durchgereicht) → unruhiges
   Stellsignal. Start: `wo = 30–50 rad/s`, danach beobachten und anpassen.

3. **`wc`** (Reglerbandbreite) – danach einstellen. Bestimmt, wie
   aggressiv der Regler auf die (geschätzte) Regelabweichung reagiert.
   Faustregel: `wo = (3…10) × wc`. Zu groß → Überschwingen/Instabilität,
   zu klein → träge Regelung.

**Vorgehen in der Praxis:**
- `wc` klein starten (z.B. 5), `wo = 5×wc`.
- `wo` schrittweise erhöhen, bis Stellsignal noch akzeptabel ruhig bleibt
  (Grenze meist durch Sensor-/Messrauschen bestimmt).
- Danach `wc` erhöhen, bis gewünschte Dynamik erreicht ist, ohne dass
  Überschwingen/Oszillation auftritt.
- `outputRateLimit` nutzen, um mechanische Sprünge/Ruckartigkeit zu
  vermeiden (z.B. Rad-Schlupf bei zu hartem Anlauf).

## Erweiterte Optionen

- `useNonlinearESO` / `useNonlinearLaw`: aktiviert die klassische,
  nichtlineare `fal()`-basierte ADRC-Variante (nach Han) statt der
  linearen Bandbreiten-Parametrierung. Kann bei kleinen Fehlern
  ruhiger/genauer sein, ist aber schwerer zu tunen (zusätzliche
  Parameter `falAlpha1`, `falAlpha2`, `falDelta`). Für die meisten
  Anwendungen reicht die lineare Variante (Standardeinstellung) völlig.

- `TrackingDifferentiator`: optionales Vorfilter, das harte
  Sollwertsprünge (z.B. neuer Zielwinkel) in einen glatten Verlauf
  überführt, bevor sie dem ADRC als Sollwert übergeben werden. Erzeugt
  zusätzlich die Sollwert-Ableitung. Sinnvoll bei sprunghaften
  Vorgaben, für einfache Geschwindigkeitsregelung meist nicht nötig.

## Wichtige Hinweise

- `update()` muss mit **fester Abtastzeit `Ts`** aufgerufen werden (im
  Beispielcode über `micros()`-Vergleich realisiert). Bei stark
  schwankendem Aufruftakt leidet die Schätzgüte des ESO.
- Für jede zu regelnde Größe (z.B. Heading UND Geschwindigkeit
  gleichzeitig) eine **eigene `ADRC`-Instanz** anlegen – die Klasse ist
  bewusst zustandsbehaftet und pro Regelkreis gedacht.
- `reset()` beim Start bzw. nach einer Zwangspause/größeren
  Diskontinuität (z.B. Roboter manuell gedreht) aufrufen, damit der ESO
  nicht von einem falschen Zustand aus einschwingen muss.
