/**
 * PID.h
 * ------------------------------------------------------------------------
 * Universeller PID-Regler.
 *
 * Regelgesetz:  u = Kp*e + Ki*Integral(e dt) + Kd*d(e)/dt
 * mit e = setpoint - measurement.
 *
 * Einsetzbar fuer beliebige Regelstrecken, insbesondere:
 *   - Winkel-/Positionsregelung (z.B. Ausrichtung eines omnidirektionalen
 *     Roboters auf einen Soll-Winkel) -> cfg.angleMode = true
 *   - Geschwindigkeitsregelung (z.B. lineare Fahrgeschwindigkeit,
 *     Radgeschwindigkeit) -> cfg.angleMode = false
 *
 * Enthaelt zwei in der Praxis wichtige Erweiterungen gegenueber einem
 * "Lehrbuch-PID":
 *
 *  1) ANTI-WINDUP (konditionale Integration):
 *     Der Integrator wird nur dann weiter aufintegriert, wenn die
 *     Stellgroesse dadurch NICHT tiefer in die Saettigung laeuft. Das
 *     verhindert das klassische PID-Problem, dass der I-Anteil bei
 *     laengerer Saettigung "aufblaeht" und danach stark ueberschwingt.
 *
 *  2) DERIVATIVE KICK VERMEIDUNG:
 *     Die Ableitung wird per Default auf den MESSWERT statt auf den
 *     Fehler gebildet (cfg.derivativeOnMeasurement = true). Ein
 *     Sollwertsprung (z.B. neuer Ziel-Winkel) erzeugt so keinen
 *     kurzzeitigen Ableitungs-"Spike" im Stellsignal. Zusaetzlich wird
 *     die Ableitung ueber ein Tiefpassfilter (Zeitkonstante cfg.Tf)
 *     geglaettet, da die reine Differenzenableitung sehr empfindlich
 *     auf Messrauschen reagiert.
 *
 * Fuer Winkelregelung (angleMode = true) werden Fehler UND die fuer die
 * Ableitung genutzte Messwertdifferenz automatisch auf [-pi, pi] gewrapped,
 * damit der Regler immer den kuerzesten Drehweg waehlt und an der +-pi
 * Sprungstelle kein Ausreisser entsteht.
 *
 * VARIABLER REGELTAKT:
 * update() nimmt optional die seit dem letzten Aufruf vergangene Zeit dt [s]
 * entgegen (z.B. per micros()-Differenz gemessen). Ein fester Takt ist NICHT
 * erforderlich. dt wird intern auf [dtMin, dtMax] begrenzt, damit ein
 * einzelner sehr langer Zyklus (z.B. durch eine blockierende Operation im
 * Hauptprogramm) Integrator/Ableitung nicht destabilisiert. Wird kein dt
 * uebergeben, verwendet die Bibliothek cfg.Ts als festen Wert
 * (Legacy-/Einfachbetrieb).
 * ------------------------------------------------------------------------
 */
#pragma once

#include <Arduino.h>
#include <math.h>

struct PIDConfig {
    float Ts = 0.001f;      // Nominelle/feste Abtastzeit [s] -- nur genutzt,
                             // wenn update() OHNE dt-Argument aufgerufen wird

    float dtMin = 1e-6f;    // untere Begrenzung fuer variables dt [s]
    float dtMax = 0.05f;    // obere Begrenzung fuer variables dt [s]
                             // (auf max. plausible Zykluszeit deines
                             //  Hauptprogramms einstellen)

    float Kp = 0.5f;        // Proportionalverstaerkung
    float Ki = 0.0f;        // Integralverstaerkung [1/s]
    float Kd = 0.0f;        // Differentialverstaerkung [s]

    float uMin = -1.0f;     // untere Stellgroessengrenze
    float uMax =  1.0f;     // obere Stellgroessengrenze

    bool angleMode = true; // true: Regelgroesse ist ein Winkel [rad],
                             // Fehler wird auf [-pi, pi] gewrapped

    bool derivativeOnMeasurement = true; // vermeidet Derivative Kick bei
                                          // Sollwertspruengen (empfohlen)
    float Tf = 0.01f;       // Zeitkonstante Tiefpassfilter D-Anteil [s],
                             // 0 = kein Filter (nicht empfohlen, rauschanfaellig)

    float outputRateLimit = 0.0f; // max. |du/dt| in Stellgroesse/s, 0 = aus
};

class PID {
public:
    explicit PID(const PIDConfig& cfg = PIDConfig());

    // Setzt Regler + Integrator/Ableitungszustand zurueck. y0 = aktueller
    // Messwert, sinnvoll beim (Neu-)Start bzw. nach einer Zwangspause.
    void reset(float y0 = 0.0f);

    // Fuehrt einen Regelzyklus mit VARIABLEM Zeitschritt aus (empfohlen).
    // dt = seit dem letzten Aufruf vergangene Zeit [s]. Wird intern auf
    // [cfg.dtMin, cfg.dtMax] begrenzt.
    // Rueckgabe: Stellgroesse (bereits begrenzt / ratenlimitiert).
    float update(float setpoint, float measurement, float dt);

    // Bequemlichkeits-Overload: nutzt cfg.Ts als festen Zeitschritt
    // (nur sinnvoll bei wirklich konstantem Aufruftakt).
    float update(float setpoint, float measurement);

    void setGains(float Kp, float Ki, float Kd);
    void setOutputLimits(float uMin, float uMax);
    void setConfig(const PIDConfig& cfg);
    const PIDConfig& getConfig() const { return cfg_; }

    // Einzelanteile auslesen (Diagnose/Tuning):
    float getPTerm() const { return pTerm_; }
    float getITerm() const { return integrator_; }
    float getDTerm() const { return dTerm_; }
    float getLastOutput() const { return lastU_; }

    // Hilfsfunktion: Winkel auf [-pi, pi] wrappen
    static float wrapPi(float angle);

private:
    PIDConfig cfg_;

    float integrator_ = 0.0f;
    float lastMeasurement_ = 0.0f;
    float lastError_ = 0.0f;
    float filteredDeriv_ = 0.0f;

    float pTerm_ = 0.0f;
    float dTerm_ = 0.0f;

    float lastU_ = 0.0f;

    float clampDt(float dt) const;
    float applyOutputStage(float uRaw, float dt);
};
