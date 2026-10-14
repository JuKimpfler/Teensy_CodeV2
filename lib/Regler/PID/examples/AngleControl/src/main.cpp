/**
 * Beispiel: Ausrichtung eines omnidirektionalen Roboters auf einen
 * Soll-Winkel (Heading) mit PID.
 *
 * Stellgroesse: normierte Winkelgeschwindigkeit omega_cmd [-1..1],
 * die anschliessend auf 4 Omniwheels in Sternanordnung (90 Grad
 * versetzt, "+"-Konfiguration) gemischt wird.
 *
 * Passe getHeadingRad() und setOmniWheelSpeeds() an deine Hardware
 * (IMU/Odometrie bzw. Motortreiber) an.
 */
#include <Arduino.h>
#include <PID/PID.h>

constexpr float TS_NOMINAL = 0.005f; // nur Fallback fuer cfg.Ts, siehe loop()

PIDConfig cfg;
PID *headingController;
uint32_t lastMicros = 0;

// ---------------------------------------------------------------------
// Platzhalter fuer Sensorik / Aktorik -> an eigene Hardware anpassen
// ---------------------------------------------------------------------

// Liefert die aktuelle Roboter-Ausrichtung in rad, z.B. aus IMU-Fusion
// (Madgwick/Mahony) oder Odometrie. Muss NICHT auf [-pi,pi] begrenzt sein,
// angleMode im PID kuemmert sich ohnehin um den Wrap-around.
float getHeadingRad()
{
    // TODO: echten Sensor einbinden (z.B. BNO055, ICM20948, ...)
    return 0.0f;
}

// Mischt eine normierte Drehgeschwindigkeit (und optional vx, vy) auf
// 4 Omniwheels in Sternanordnung (0/90/180/270 Grad um den Roboter,
// jedes Rad rollt tangential zu seinem Radius -- optisch ein 4-zackiger
// Stern / "+"-Layout). Passe an eigene Radanordnung / Motortreiber an.
//
// Hinweis: Das System ist mit 4 Raedern fuer 3 Freiheitsgrade (vx, vy,
// omega) redundant/ueberbestimmt. Die tangentiale Projektionsformel
// liefert trotzdem für jedes Rad die korrekt vorzeichenrichtige
// Solldrehzahl; Radschlupf gleicht die Redundanz mechanisch aus.
void setOmniWheelSpeeds(float vx, float vy, float omega)
{
    // Radwinkel relativ zur Roboter-Vorwaertsachse (0 = vorne, math. positiv
    // = gegen den Uhrzeigersinn). 0/90/180/270 Grad -> Stern/"+"-Anordnung.
    static const float wheelAngles[4] = {
        0.0f,            // vorne
        PI / 2.0f,       // links
        PI,              // hinten
        3.0f * PI / 2.0f // rechts
    };
    for (int i = 0; i < 4; ++i)
    {
        float w = -sinf(wheelAngles[i]) * vx + cosf(wheelAngles[i]) * vy + omega;
        w = constrain(w, -1.0f, 1.0f);
        // TODO: w auf Motortreiber i ausgeben
    }
}

volatile float targetHeading = 0.0f; // Soll-Winkel [rad]

void setup()
{
    Serial.begin(115200);

    cfg.Ts = TS_NOMINAL; // nur Fallback, siehe loop()
    cfg.dtMin = 1e-5f;   // 10 us -- gegen dt=0
    cfg.dtMax = 0.03f;   // 30 ms -- deckelt Ausreisser, falls das
                         // Hauptprogramm mal laenger braucht

    cfg.Kp = 2.0f; // Start: nur P, dann I/D schrittweise zuschalten
    cfg.Ki = 0.5f;
    cfg.Kd = 0.05f;

    cfg.uMin = -1.0f;
    cfg.uMax = 1.0f;
    cfg.angleMode = true;               // WICHTIG: kuerzester Drehweg, kein +-pi Sprung
    cfg.derivativeOnMeasurement = true; // kein Kick bei Sollwinkel-Spruengen
    cfg.Tf = 0.02f;                     // D-Anteil filtern (Winkelmessung ist meist verrauscht)
    cfg.outputRateLimit = 6.0f;         // sanfter Anlauf / Ruckbegrenzung

    headingController = new PID(cfg);
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
    // geradeaus fahren, hier vx/vy von einer zweiten (Geschwindigkeits-)PID-
    // Instanz einspeisen -> siehe VelocityControl-Beispiel.
    setOmniWheelSpeeds(0.0f, 0.0f, omegaCmd);

    // Optional: Diagnose ueber Serial
    // Serial.printf("heading=%.3f target=%.3f u=%.3f P=%.3f I=%.3f D=%.3f\n",
    //     heading, targetHeading, omegaCmd,
    //     headingController->getPTerm(), headingController->getITerm(), headingController->getDTerm());
}
