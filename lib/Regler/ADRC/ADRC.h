/**
 * ADRC.h
 * ------------------------------------------------------------------------
 * Universeller Active Disturbance Rejection Control (ADRC) Regler.
 *
 * Unterstuetzt zwei Streckenordnungen:
 *
 *   ADRCOrder::FIRST  -> Strecke 1. Ordnung:  y' = b0*u + f(t)
 *                         Typisch: Geschwindigkeitsregelung (Motor-/Radgeschw.,
 *                         lineare Fahrgeschwindigkeit eines Roboters)
 *
 *   ADRCOrder::SECOND -> Strecke 2. Ordnung:  y'' = b0*u + f(t)
 *                         Typisch: Positions-/Winkelregelung (Ausrichtung
 *                         eines omnidirektionalen Roboters auf einen
 *                         Soll-Winkel)
 *
 * f(t) ist die "totale Stoerung" (unmodellierte Dynamik, externe Stoerungen,
 * Parameterunsicherheiten, Nichtlinearitaeten, Reibung, ...). Der ADRC schaetzt
 * f(t) online mit einem Extended State Observer (ESO) und kompensiert sie
 * aktiv im Stellsignal -> daher ist i.d.R. KEINE genaue Streckenkenntnis
 * noetig, nur eine grobe Schaetzung des Verstaerkungsfaktors b0.
 *
 * Parametrierung erfolgt (wie bei Gao 2003 / "Linear ADRC") ueber nur drei
 * physikalisch interpretierbare Groessen:
 *
 *   wc  - Regler-Bandbreite   (wie schnell/aggressiv geregelt wird)
 *   wo  - Beobachter-Bandbreite (wie schnell der ESO schaetzt),
 *         Faustregel: wo = (3...10) * wc
 *   b0  - Streckenverstaerkung (grobe Schaetzung reicht, ADRC ist robust
 *         gegenueber Fehlern in b0)
 *
 * Optional kann statt der linearen auch eine nichtlineare fal()-basierte
 * ESO-/Regelgesetz-Variante (klassisches Han-ADRC) aktiviert werden.
 *
 * Fuer Winkelregelung (angleMode = true) wird die Regelabweichung sowie die
 * ESO-Innovation automatisch auf [-pi, pi] gewrapped, damit der Regler immer
 * den kuerzesten Drehweg waehlt und keine Sprungstelle bei +-pi entsteht.
 *
 * VARIABLER REGELTAKT:
 * update() nimmt optional die seit dem letzten Aufruf vergangene Zeit dt [s]
 * entgegen (z.B. per micros()-Differenz gemessen). Ein fester Takt ist NICHT
 * erforderlich -- die interne Euler-Integration des ESO verarbeitet variable
 * dt-Werte direkt. dt wird intern auf [dtMin, dtMax] begrenzt, damit ein
 * einzelner sehr langer Zykluses (z.B. durch eine blockierende Operation im
 * Hauptprogramm) den Beobachter nicht destabilisiert. Wird kein dt uebergeben,
 * verwendet die Bibliothek cfg.Ts als festen Wert (Legacy-/Einfachbetrieb).
 * ------------------------------------------------------------------------
 */
#pragma once

#include <Arduino.h>
#include <math.h>

enum class ADRCOrder : uint8_t
{
    FIRST = 1, // Strecke 1. Ordnung (z.B. Geschwindigkeit)
    SECOND = 2 // Strecke 2. Ordnung (z.B. Winkel/Position)
};

struct ADRCConfig
{
    ADRCOrder order = ADRCOrder::SECOND;

    float Ts = 0.001f; // Nominelle/feste Abtastzeit [s] -- nur genutzt,
                       // wenn update() OHNE dt-Argument aufgerufen wird

    float dtMin = 1e-6f; // untere Begrenzung fuer variables dt [s]
                         // (verhindert Division/Instabilitaet bei dt=0)
    float dtMax = 0.05f; // obere Begrenzung fuer variables dt [s]
                         // (verhindert ESO-Ueberschwinger bei Ausreissern,
                         //  z.B. wenn der Loop mal blockiert hat;
                         //  auf max. plausible Zykluszeit deines
                         //  Hauptprogramms einstellen)

    float wc = 10.0f; // Regler-Bandbreite [rad/s]
    float wo = 50.0f; // Beobachter-Bandbreite [rad/s] (typ. 3-10x wc)
    float b0 = 1.0f;  // Streckenverstaerkung (Schaetzwert)

    float uMin = -1.0f; // untere Stellgroessengrenze
    float uMax = 1.0f;  // obere Stellgroessengrenze

    bool angleMode = false; // true: Regelgroesse ist ein Winkel [rad],
                            // Fehler wird auf [-pi, pi] gewrapped

    bool useNonlinearESO = false; // fal()-basierte ESO statt linearer ESO
    bool useNonlinearLaw = false; // fal()-basiertes Regelgesetz statt linearem PD

    float falAlpha1 = 0.5f;  // fal-Exponent fuer Lageanteil
    float falAlpha2 = 0.25f; // fal-Exponent fuer Geschw./Stoerungsanteil
    float falDelta = 0.01f;  // Breite der Linearisierungszone um 0

    float outputRateLimit = 0.0f; // max. |du/dt| in Stellgroesse/s, 0 = aus
};

class ADRC
{
public:
    explicit ADRC(const ADRCConfig &cfg = ADRCConfig());

    // Setzt den Regler + ESO-Zustaende zurueck. y0 = aktueller Messwert,
    // sinnvoll beim (Neu-)Start, damit der ESO nicht erst "einschwingen" muss.
    void reset(float y0 = 0.0f);

    // Fuehrt einen Regelzyklus mit VARIABLEM Zeitschritt aus (empfohlen).
    // setpoint = Sollwert, measurement = Istwert,
    // dt = seit dem letzten Aufruf vergangene Zeit [s] (z.B. per micros()
    // gemessen). dt wird intern auf [cfg.dtMin, cfg.dtMax] begrenzt.
    // Rueckgabe: Stellgroesse (bereits begrenzt / ratenlimitiert).
    float update(float setpoint, float measurement, float dt);

    // Bequemlichkeits-Overload: nutzt cfg.Ts als festen Zeitschritt
    // (nur sinnvoll bei wirklich konstantem Aufruftakt).
    float update(float setpoint, float measurement);

    void setBandwidths(float wc, float wo);
    void setB0(float b0);
    void setOutputLimits(float uMin, float uMax);
    void setConfig(const ADRCConfig &cfg);
    const ADRCConfig &getConfig() const { return cfg_; }

    // ESO-Zustaende auslesen (Diagnose/Tuning):
    //   Ordnung 1: z1 = geschaetzte Regelgroesse, z2 = geschaetzte Stoerung
    //   Ordnung 2: z1 = Lage, z2 = Geschwindigkeit, z3 = geschaetzte Stoerung
    float getState(uint8_t i) const;
    float getDisturbanceEstimate() const;
    float getLastOutput() const { return lastU_; }

    // Hilfsfunktion: Winkel auf [-pi, pi] wrappen
    static float wrapPi(float angle);

private:
    ADRCConfig cfg_;

    float z1_ = 0.0f;
    float z2_ = 0.0f;
    float z3_ = 0.0f; // nur bei SECOND order genutzt

    float beta1_ = 0.0f, beta2_ = 0.0f, beta3_ = 0.0f; // ESO-Verstaerkungen
    float kp_ = 0.0f, kd_ = 0.0f;                      // Regelgesetz-Verstaerkungen

    float lastU_ = 0.0f;

    void computeGains();
    float fal(float e, float alpha, float delta) const;
    float clampDt(float dt) const;

    float update1st(float setpoint, float measurement, float dt);
    float update2nd(float setpoint, float measurement, float dt);

    float applyOutputStage(float uRaw, float dt);
};
