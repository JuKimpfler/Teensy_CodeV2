/**
 * Beispiel: Regelung der linearen Fahrgeschwindigkeit eines Roboters
 * mit ADRC 1. Ordnung.
 *
 * Regelstrecke: Geschwindigkeit (1. Ordnung, da Motorspannung/PWM ->
 * ueber Motor-/Fahrzeugdynamik ~ Kraft -> Geschwindigkeitsaenderung)
 * Stellgroesse: normierter Motor-Duty-Cycle [-1..1]
 *
 * Passe getVelocity() und setMotorDuty() an deine Hardware
 * (Encoder/Odometrie bzw. Motortreiber) an.
 */
#include <Arduino.h>
#include <ADRC/ADRC.h>

constexpr float TS_NOMINAL = 0.005f; // nur Fallback fuer cfg.Ts, siehe loop()

ADRCConfig cfg;
ADRC* velocityController;
uint32_t lastMicros = 0;

volatile float targetVelocity = 0.0f; // Soll-Geschwindigkeit [m/s]

// ---------------------------------------------------------------------
// Platzhalter fuer Sensorik / Aktorik -> an eigene Hardware anpassen
// ---------------------------------------------------------------------

// Liefert die aktuelle (gemessene) lineare Geschwindigkeit [m/s],
// z.B. aus Encoder-Differenzen oder Odometrie-Fusion.
float getVelocity() {
    // TODO: echten Encoder/Sensor einbinden
    return 0.0f;
}

// Gibt den Duty-Cycle an den Motortreiber aus.
void setMotorDuty(float duty) {
    duty = constrain(duty, -1.0f, 1.0f);
    // TODO: z.B. analogWrite() / DIR-Pin setzen, oder an ESC/Motortreiber senden
}

void setup() {
    Serial.begin(115200);

    cfg.order = ADRCOrder::FIRST;    // Geschwindigkeit = Strecke 1. Ordnung
    cfg.Ts    = TS_NOMINAL;          // nur Fallback, siehe loop()
    cfg.dtMin = 1e-5f;
    cfg.dtMax = 0.03f;               // deckelt Ausreisser bei langsamen Zyklen
    cfg.wc    = 8.0f;                // Reglerbandbreite
    cfg.wo    = 40.0f;               // Beobachterbandbreite (~5x wc)
    cfg.b0    = 3.0f;                // grobe Schaetzung: wie stark Duty auf
                                      // Beschleunigung wirkt -> per Test anpassen
    cfg.uMin  = -1.0f;
    cfg.uMax  =  1.0f;
    cfg.angleMode = false;           // keine Winkelgroesse
    cfg.outputRateLimit = 4.0f;      // begrenzt Ruck (du/dt)

    velocityController = new ADRC(cfg);
    velocityController->reset(getVelocity());
    lastMicros = micros();
}

void loop() {
    // Variabler Zyklustakt -- keine feste Wartezeit, einfach dt messen.
    uint32_t now = micros();
    float dt = (now - lastMicros) * 1e-6f;
    lastMicros = now;

    float v = getVelocity();
    float duty = velocityController->update(targetVelocity, v, dt);
    setMotorDuty(duty);

    // Optional: Diagnose ueber Serial
    // Serial.printf("v=%.3f target=%.3f duty=%.3f disturbance=%.3f\n",
    //     v, targetVelocity, duty, velocityController->getDisturbanceEstimate());
}
