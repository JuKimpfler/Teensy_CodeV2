/**
 * Beispiel: Ausrichtung eines omnidirektionalen Roboters auf einen
 * Soll-Winkel (Heading) mit ADRC 2. Ordnung.
 *
 * Regelstrecke: Winkel (2. Ordnung, da Drehmoment -> Winkelbeschleunigung)
 * Stellgroesse:  normierte Winkelgeschwindigkeit omega_cmd [-1..1],
 *                die anschliessend auf 3 Omniwheels gemischt wird.
 *
 * Passe getHeadingRad() und setWheelSpeeds() an deine Hardware
 * (IMU/Odometrie bzw. Motortreiber) an.
 */
#include <Arduino.h>
#include <ADRC/ADRC.h>

// ---------------------------------------------------------------------
// Regelparameter
// ---------------------------------------------------------------------
// Nominalwert, wird nur fuer die initiale Konfiguration/Verhaeltnis
// wc:wo genutzt -- der tatsaechliche Regeltakt ist variabel, siehe loop().
constexpr float TS_NOMINAL = 0.005f;

ADRCConfig cfg;
ADRC *headingController;
uint32_t lastMicros = 0;

// Soll-Winkel [rad], z.B. per Funk/State-Machine gesetzt
volatile float targetHeading = 0.0f;

// ---------------------------------------------------------------------
// Platzhalter fuer Sensorik / Aktorik -> an eigene Hardware anpassen
// ---------------------------------------------------------------------

// Liefert die aktuelle Roboter-Ausrichtung in rad, z.B. aus IMU-Fusion
// (Madgwick/Mahony) oder Odometrie. Muss NICHT auf [-pi,pi] begrenzt sein,
// aber angleMode im ADRC kuemmert sich ohnehin um den Wrap-around.
float getHeadingRad()
{
    // TODO: echten Sensor einbinden (z.B. BNO055, ICM20948, ...)
    return 0.0f;
}

// Mischt eine normierte Drehgeschwindigkeit (und optional vx, vy) auf
// 3 Omniwheels (120 Grad versetzt, "Kiwi-Drive"). Passe an eigene
// Radanordnung / Motortreiber (z.B. PWM+DIR, CAN, ODrive, ...) an.
void setOmniWheelSpeeds(float vx, float vy, float omega)
{
    static const float wheelAngles[3] = {
        0.0f, 2.0f * PI / 3.0f, 4.0f * PI / 3.0f};
    for (int i = 0; i < 3; ++i)
    {
        float w = -sinf(wheelAngles[i]) * vx + cosf(wheelAngles[i]) * vy + omega; // + R_robot * omega, falls Radius beruecksichtigt werden soll
        w = constrain(w, -1.0f, 1.0f);
        // TODO: w auf Motortreiber i ausgeben (PWM/DIR oder Geschwindigkeitssollwert)
    }
}

void setup()
{
    Serial.begin(115200);

    cfg.order = ADRCOrder::SECOND; // Winkel = Strecke 2. Ordnung
    cfg.Ts = TS_NOMINAL;           // nur Fallback, siehe loop()
    cfg.dtMin = 1e-5f;             // 10 us -- gegen dt=0
    cfg.dtMax = 0.03f;             // 30 ms -- deckelt Ausreisser, falls das
                                   // Hauptprogramm mal laenger braucht
    cfg.wc = 15.0f;                // Reglerbandbreite: wie schnell ausgerichtet wird
    cfg.wo = 80.0f;                // Beobachterbandbreite: ca. 5x wc als Startwert
    cfg.b0 = 8.0f;                 // grobe Schaetzung: wie stark omega_cmd auf
                                   // die Winkelbeschleunigung wirkt -> per Test anpassen
    cfg.uMin = -1.0f;
    cfg.uMax = 1.0f;
    cfg.angleMode = true;       // WICHTIG: sorgt fuer kuerzesten Drehweg (kein +-pi Sprung)
    cfg.outputRateLimit = 6.0f; // begrenzt Stellgroessen-Aenderungsrate (sanfter Anlauf)

    headingController = new ADRC(cfg);
    headingController->reset(getHeadingRad());
    lastMicros = micros();
}

void loop()
{
    // Variabler Zyklustakt: einfach die tatsaechlich vergangene Zeit messen
    // und an update() uebergeben -- kein busy-wait / kein fester Takt noetig.
    uint32_t now = micros();
    float dt = (now - lastMicros) * 1e-6f; // us -> s
    lastMicros = now;

    float heading = getHeadingRad();
    float omegaCmd = headingController->update(targetHeading, heading, dt);

    // Reine Drehung auf der Stelle (vx = vy = 0). Soll der Roboter gleichzeitig
    // geradeaus fahren, hier vx/vy von einer zweiten (Geschwindigkeits-)ADRC-
    // Instanz einspeisen -> siehe VelocityControl-Beispiel.
    setOmniWheelSpeeds(0.0f, 0.0f, omegaCmd);

    // Optional: Diagnose ueber Serial
    // Serial.printf("heading=%.3f target=%.3f u=%.3f disturbance=%.3f\n",
    //     heading, targetHeading, omegaCmd, headingController->getDisturbanceEstimate());
}
