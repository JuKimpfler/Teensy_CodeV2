#include "ADRC.h"

ADRC::ADRC(const ADRCConfig& cfg) : cfg_(cfg) {
    computeGains();
    reset(0.0f);
}

void ADRC::reset(float y0) {
    z1_ = y0;
    z2_ = 0.0f;
    z3_ = 0.0f;
    lastU_ = 0.0f;
}

void ADRC::setBandwidths(float wc, float wo) {
    cfg_.wc = wc;
    cfg_.wo = wo;
    computeGains();
}

void ADRC::setB0(float b0) {
    // b0 darf nie 0 sein, sonst Division durch 0 im Regelgesetz
    cfg_.b0 = (fabsf(b0) < 1e-6f) ? 1e-6f : b0;
}

void ADRC::setOutputLimits(float uMin, float uMax) {
    cfg_.uMin = uMin;
    cfg_.uMax = uMax;
}

void ADRC::setConfig(const ADRCConfig& cfg) {
    cfg_ = cfg;
    computeGains();
}

void ADRC::computeGains() {
    // Bandbreiten-Parametrierung nach Gao (2003) / "Linear ADRC":
    // Beobachterpole und Reglerpole werden jeweils auf ein n-faches
    // Realteil-Pol (Butterworth-artig) gelegt -> nur 2 Parameter (wc, wo)
    // statt vieler einzelner Verstaerkungen.
    switch (cfg_.order) {
        case ADRCOrder::FIRST:
            beta1_ = 2.0f * cfg_.wo;
            beta2_ = cfg_.wo * cfg_.wo;
            kp_    = cfg_.wc;
            break;

        case ADRCOrder::SECOND:
            beta1_ = 3.0f * cfg_.wo;
            beta2_ = 3.0f * cfg_.wo * cfg_.wo;
            beta3_ = cfg_.wo * cfg_.wo * cfg_.wo;
            kp_    = cfg_.wc * cfg_.wc;
            kd_    = 2.0f * cfg_.wc;
            break;
    }
}

float ADRC::fal(float e, float alpha, float delta) const {
    // Klassische fal()-Funktion aus Han's ADRC: nahe 0 linear (glatt),
    // weiter weg potenzfoermig (schneller/aggressiver) -> reduziert
    // "High-Frequency Chattering" bei kleinen Fehlern.
    float ae = fabsf(e);
    if (ae <= delta) {
        return e / powf(delta, 1.0f - alpha);
    }
    return copysignf(powf(ae, alpha), e);
}

float ADRC::wrapPi(float angle) {
    return atan2f(sinf(angle), cosf(angle));
}

float ADRC::clampDt(float dt) const {
    // Begrenzung ist wichtig bei variablem Zeitschritt:
    // - dt zu klein/0 (z.B. Aufruf doppelt im selben Mikrosekunden-Tick)
    //   wuerde die Regelung praktisch einfrieren -> unkritisch, aber unnoetig
    // - dt zu gross (z.B. Hauptprogramm hat mal laenger blockiert) wuerde
    //   die Euler-Integration im ESO einen sehr grossen Sprung machen lassen
    //   -> kann zu Ueberschwingern/Instabilitaet fuehren, daher hart deckeln
    if (dt < cfg_.dtMin) return cfg_.dtMin;
    if (dt > cfg_.dtMax) return cfg_.dtMax;
    return dt;
}

float ADRC::applyOutputStage(float uRaw, float dt) {
    if (cfg_.outputRateLimit > 0.0f) {
        float maxDelta = cfg_.outputRateLimit * dt;
        float delta = uRaw - lastU_;
        if (delta > maxDelta) delta = maxDelta;
        if (delta < -maxDelta) delta = -maxDelta;
        uRaw = lastU_ + delta;
    }
    if (uRaw > cfg_.uMax) uRaw = cfg_.uMax;
    if (uRaw < cfg_.uMin) uRaw = cfg_.uMin;
    lastU_ = uRaw;
    return uRaw;
}

float ADRC::update1st(float setpoint, float measurement, float dt) {
    // --- Extended State Observer (2 Zustaende: y, Stoerung f) ---
    float innov = z1_ - measurement;
    if (cfg_.angleMode) innov = wrapPi(innov);

    float dz1, dz2;
    if (cfg_.useNonlinearESO) {
        dz1 = z2_ + cfg_.b0 * lastU_ - beta1_ * fal(innov, 0.5f, cfg_.falDelta);
        dz2 = -beta2_ * fal(innov, 0.25f, cfg_.falDelta);
    } else {
        dz1 = z2_ + cfg_.b0 * lastU_ - beta1_ * innov;
        dz2 = -beta2_ * innov;
    }
    z1_ += dz1 * dt;
    z2_ += dz2 * dt;

    // --- Regelgesetz (P-Regler auf geschaetzten Zustand) ---
    float e1 = setpoint - z1_;
    if (cfg_.angleMode) e1 = wrapPi(e1);

    float u0 = cfg_.useNonlinearLaw
                   ? kp_ * fal(e1, cfg_.falAlpha1, cfg_.falDelta)
                   : kp_ * e1;

    // --- Stoerkompensation ---
    return (u0 - z2_) / cfg_.b0;
}

float ADRC::update2nd(float setpoint, float measurement, float dt) {
    // --- Extended State Observer (3 Zustaende: y, y', Stoerung f) ---
    float innov = z1_ - measurement;
    if (cfg_.angleMode) innov = wrapPi(innov);

    float dz1, dz2, dz3;
    if (cfg_.useNonlinearESO) {
        dz1 = z2_ - beta1_ * fal(innov, 0.5f, cfg_.falDelta);
        dz2 = z3_ + cfg_.b0 * lastU_ - beta2_ * fal(innov, 0.25f, cfg_.falDelta);
        dz3 = -beta3_ * fal(innov, 0.125f, cfg_.falDelta);
    } else {
        dz1 = z2_ - beta1_ * innov;
        dz2 = z3_ + cfg_.b0 * lastU_ - beta2_ * innov;
        dz3 = -beta3_ * innov;
    }
    z1_ += dz1 * dt;
    z2_ += dz2 * dt;
    z3_ += dz3 * dt;

    // --- Regelgesetz (PD auf geschaetzte Zustaende, Soll-Geschwindigkeit = 0
    //     da Regulierung auf einen festen Soll-Winkel/-Position) ---
    float e1 = setpoint - z1_;
    if (cfg_.angleMode) e1 = wrapPi(e1);

    float u0;
    if (cfg_.useNonlinearLaw) {
        u0 = kp_ * fal(e1, cfg_.falAlpha1, cfg_.falDelta)
             - kd_ * fal(z2_, cfg_.falAlpha2, cfg_.falDelta);
    } else {
        u0 = kp_ * e1 - kd_ * z2_;
    }

    // --- Stoerkompensation ---
    return (u0 - z3_) / cfg_.b0;
}

float ADRC::update(float setpoint, float measurement, float dt) {
    dt = clampDt(dt);
    float uRaw = (cfg_.order == ADRCOrder::FIRST)
                     ? update1st(setpoint, measurement, dt)
                     : update2nd(setpoint, measurement, dt);
    return applyOutputStage(uRaw, dt);
}

float ADRC::update(float setpoint, float measurement) {
    // Legacy-/Einfachbetrieb mit fest angenommenem Zeitschritt cfg.Ts.
    return update(setpoint, measurement, cfg_.Ts);
}

float ADRC::getState(uint8_t i) const {
    switch (i) {
        case 0: return z1_;
        case 1: return z2_;
        case 2: return z3_;
        default: return 0.0f;
    }
}

float ADRC::getDisturbanceEstimate() const {
    return (cfg_.order == ADRCOrder::FIRST) ? z2_ : z3_;
}
