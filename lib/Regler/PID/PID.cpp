#include "PID.h"

PID::PID(const PIDConfig &cfg) : cfg_(cfg)
{
    reset(0.0f);
}

void PID::reset(float y0)
{
    integrator_ = 0.0f;
    lastMeasurement_ = y0;
    lastError_ = 0.0f;
    filteredDeriv_ = 0.0f;
    pTerm_ = 0.0f;
    dTerm_ = 0.0f;
    lastU_ = 0.0f;
}

void PID::setGains(float Kp, float Ki, float Kd)
{
    cfg_.Kp = Kp;
    cfg_.Ki = Ki;
    cfg_.Kd = Kd;
}

void PID::setOutputLimits(float uMin, float uMax)
{
    cfg_.uMin = uMin;
    cfg_.uMax = uMax;
}

void PID::setConfig(const PIDConfig &cfg)
{
    cfg_ = cfg;
}

float PID::wrapPi(float angle)
{
    // Normalfall: Winkel liegt klar innerhalb (-pi, pi) -> unveraendert zurueck (spart sin, cos, atan2).
    // Nahe +-pi und ausserhalb bleibt die exakte Rechnung.
    if (angle > -3.14f && angle < 3.14f)
    {
        return angle;
    }
    return atan2f(sinf(angle), cosf(angle));
}

float PID::clampDt(float dt) const
{
    // Begrenzung ist wichtig bei variablem Zeitschritt:
    // - dt zu klein/0 wuerde Ki/Kd-Anteile unnoetig verrauschen bzw.
    //   durch Division/Multiplikation mit ~0 Artefakte erzeugen
    // - dt zu gross (z.B. Hauptprogramm hat mal laenger blockiert) wuerde
    //   dem Integrator einen sehr grossen Sprung bescheren und die
    //   Ableitung stark verfaelschen -> daher hart deckeln
    if (dt < cfg_.dtMin)
        return cfg_.dtMin;
    if (dt > cfg_.dtMax)
        return cfg_.dtMax;
    return dt;
}

float PID::applyOutputStage(float uRaw, float dt)
{
    if (cfg_.outputRateLimit > 0.0f)
    {
        float maxDelta = cfg_.outputRateLimit * dt;
        float delta = uRaw - lastU_;
        if (delta > maxDelta)
            delta = maxDelta;
        if (delta < -maxDelta)
            delta = -maxDelta;
        uRaw = lastU_ + delta;
    }
    if (uRaw > cfg_.uMax)
        uRaw = cfg_.uMax;
    if (uRaw < cfg_.uMin)
        uRaw = cfg_.uMin;
    lastU_ = uRaw;
    return uRaw;
}

float PID::update(float setpoint, float measurement, float dt)
{
    dt = clampDt(dt);

    // --- Fehler ---
    float error = setpoint - measurement;
    if (cfg_.angleMode)
        error = wrapPi(error);

    // --- P-Anteil ---
    pTerm_ = cfg_.Kp * error;

    // --- D-Anteil (mit Derivative-Kick-Vermeidung + Tiefpassfilter) ---
    float rawDeriv;
    if (cfg_.derivativeOnMeasurement)
    {
        // Ableitung auf den Messwert statt auf den Fehler: ein
        // Sollwertsprung wirkt sich NICHT unmittelbar auf die Ableitung
        // aus (kein "Kick"), da lastMeasurement_ stetig ist.
        float dm = measurement - lastMeasurement_;
        if (cfg_.angleMode)
            dm = wrapPi(dm);
        rawDeriv = dm / dt;
    }
    else
    {
        // Klassische Ableitung des Fehlers -- einfacher, aber anfaellig
        // fuer Ausreisser bei Sollwertspruengen.
        float de = error - lastError_;
        if (cfg_.angleMode)
            de = wrapPi(de);
        rawDeriv = de / dt;
    }
    lastMeasurement_ = measurement;
    lastError_ = error;

    // Tiefpassfilter 1. Ordnung auf die Ableitung (reduziert
    // Rauschverstaerkung durch die Differenzenbildung)
    if (cfg_.Tf > 0.0f)
    {
        float alpha = dt / (cfg_.Tf + dt);
        filteredDeriv_ += alpha * (rawDeriv - filteredDeriv_);
    }
    else
    {
        filteredDeriv_ = rawDeriv;
    }

    // Vorzeichen: bei Ableitung auf Messwert ist d(error)/dt = -d(measurement)/dt
    // (fuer konstanten Sollwert), daher hier das Minuszeichen, damit das
    // Ergebnis dem klassischen Kd*d(error)/dt entspricht.
    dTerm_ = cfg_.derivativeOnMeasurement ? -cfg_.Kd * filteredDeriv_
                                          : cfg_.Kd * filteredDeriv_;

    // --- I-Anteil mit Anti-Windup (konditionale Integration) ---
    // Zuerst pruefen, ob die (noch unbegrenzte) Stellgroesse mit dem
    // BISHERIGEN Integratorstand schon in der Saettigung waere. Nur wenn
    // eine weitere Integration die Saettigung NICHT verstaerken wuerde,
    // wird integriert. So "blaeht" sich der I-Anteil waehrend laengerer
    // Saettigung nicht unbegrenzt auf (verhindert Ueberschwinger beim
    // Verlassen der Saettigung).
    float outputUnsat = pTerm_ + integrator_ + dTerm_;
    bool saturatedHigh = outputUnsat > cfg_.uMax;
    bool saturatedLow = outputUnsat < cfg_.uMin;

    bool allowIntegration = true;
    if (saturatedHigh && error > 0.0f)
        allowIntegration = false;
    if (saturatedLow && error < 0.0f)
        allowIntegration = false;

    if (allowIntegration)
    {
        integrator_ += cfg_.Ki * error * dt;
    }

    // --- Gesamtausgang ---
    float uRaw = pTerm_ + integrator_ + dTerm_;
    return applyOutputStage(uRaw, dt);
}

float PID::update(float setpoint, float measurement)
{
    // Legacy-/Einfachbetrieb mit fest angenommenem Zeitschritt cfg.Ts.
    return update(setpoint, measurement, cfg_.Ts);
}
