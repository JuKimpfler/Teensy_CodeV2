
// Namespaces für Systemlib

/* ========================================================================
 * Positionsfusion: Odometrie (Raeder) + Radar  --  1D-Kalman je Achse
 *
 *  Vorhersage (jeder Zyklus):  x += v * dt          Odometrie fuehrt
 *                              P += Q * dt          Unsicherheit waechst
 *  Korrektur  (wenn Radar ok): x += K * (z - x)     Radar zieht nur sanft nach
 *
 *  Q waechst mit Geschwindigkeit und Drehrate (Schlupf der Omniraeder).
 *  R (Radar-Rauschen) waechst mit schlechter Certainty und mit Schraeglage.
 *
 *  Sprung-/Hinderniserkennung ueber Gate: Ein Radarwert, der weiter als
 *  FUS_GATE_SIGMA Sigma von der Vorhersage weg liegt, wird ignoriert.
 *  Bleibt ein abweichender Wert aber stabil ueber FUS_REACQ_MS, war die
 *  Odometrie wohl weggedriftet (Anschieben, Kollision) -> Position wird
 *  neu gesetzt. Werte, die wie ein Hindernis aussehen (Sensor sieht
 *  KUERZER als erwartet), muessen dafuer laenger stabil sein.
 * ===================================================================== */
namespace
{
uint32_t motorSampleUs = 0; // micros() der letzten ADC-Messung, gesetzt in updateMotors()

struct AxisMeas
{
    float z;        // gemessene Position auf der Achse (cm)
    float cert;     // 0..1
    int8_t suspect; // -1: Sensor an der niedrigen Wand, +1: an der hohen, 0: Paar stimmt ueberein
    uint32_t t;     // Zeitstempel des Radarwerts (millis)
};

struct AxisMeasSet
{
    AxisMeas m[2];
    uint8_t n;
};

struct AxisFilter
{
    float x = 0.0f;
    float P = FUS_P_INIT;
    float innov = 0.0f;
    bool accepted = false;
    float cand[3] = {0, 0, 0};
    uint32_t candSince[3] = {0, 0, 0};
    bool candActive[3] = {false, false, false};
    // Positionshistorie (Ringpuffer) fuer die Radar-Latenzkompensation
    uint32_t hT[FUS_HIST_N] = {0};
    float hX[FUS_HIST_N] = {0};
    uint8_t hHead = 0;
    uint8_t hCount = 0;
};

AxisFilter filtX;
AxisFilter filtY;

AxisMeas makeMeas(float z, float cert, int8_t suspect, uint32_t t)
{
    AxisMeas m;
    m.z = z;
    m.cert = cert;
    m.suspect = suspect;
    m.t = t;
    return m;
}

void histPush(AxisFilter &f, uint32_t t, float x)
{
    if (f.hCount > 0)
    {
        const uint8_t last = (f.hHead + FUS_HIST_N - 1) % FUS_HIST_N;
        if (f.hT[last] == t)
        {
            f.hX[last] = x;
            return;
        } // zwei Zyklen in derselben ms
    }
    f.hT[f.hHead] = t;
    f.hX[f.hHead] = x;
    f.hHead = (f.hHead + 1) % FUS_HIST_N;
    if (f.hCount < FUS_HIST_N)
        f.hCount++;
}

/// Position zum Zeitpunkt t (linear interpoliert). false, wenn t aelter als die Historie ist.
bool histLookup(const AxisFilter &f, uint32_t t, float &x)
{
    for (uint8_t k = 0; k < f.hCount; k++)
    {
        const uint8_t idx = (f.hHead + FUS_HIST_N - 1 - k) % FUS_HIST_N;
        if (f.hT[idx] <= t)
        {
            if (k == 0)
            {
                x = f.hX[idx];
                return true;
            } // t ist neuer als alles -> neuester Wert
            const uint8_t nxt = (idx + 1) % FUS_HIST_N;
            const uint32_t span = f.hT[nxt] - f.hT[idx];
            const float a = (span > 0) ? (float)(t - f.hT[idx]) / (float)span : 0.0f;
            x = f.hX[idx] + a * (f.hX[nxt] - f.hX[idx]);
            return true;
        }
    }
    return false;
}

/// Korrektur d gilt ab Messzeitpunkt tFrom -> auch in der Historie nachtragen,
/// sonst wuerde die naechste Messung dieselbe Korrektur nochmal anwenden.
void histShift(AxisFilter &f, uint32_t tFrom, float d)
{
    for (uint8_t k = 0; k < f.hCount; k++)
    {
        const uint8_t idx = (f.hHead + FUS_HIST_N - 1 - k) % FUS_HIST_N;
        if (f.hT[idx] >= tFrom)
            f.hX[idx] += d;
    }
}

/// Geschwindigkeit um die Rueckmelde-Verzoegerung nach vorne schieben.
/// Extrapoliert nie ueber Null hinaus (kein Rueckwaertsruckeln beim Bremsen).
float leadVelocity(float v, float a, float lead)
{
    if (v == 0.0f)
        return 0.0f;
    const float vt = v + a * lead;
    return (vt * v < 0.0f) ? 0.0f : vt;
}
}

namespace
{
/// sin/cos der festen Radwinkel, einmal berechnet (gleiche Ausdruecke wie zuvor, nur nicht in jedem Aufruf)
struct WheelTrig
{
    float s[4];
    float c[4];
};

const WheelTrig &wheelTrig()
{
    static WheelTrig t;
    static bool ready = false;
    if (!ready)
    {
        static const float wheelAngles[4] = {45, 315, 135, 225};
        for (int i = 0; i < 4; ++i)
        {
            t.s[i] = sinf(wheelAngles[i] * DEG_TO_RAD);
            t.c[i] = cosf(wheelAngles[i] * DEG_TO_RAD);
        }
        ready = true;
    }
    return t;
}
} // namespace


/* -------------- Port-Expander -------------- */

namespace
{
inline void setBit(uint8_t &value, uint8_t bit, bool on)
{
    if (on)
        value |= (uint8_t)(1u << bit);
    else
        value &= (uint8_t)~(1u << bit);
}

/// Ausgabebyte des System-Expanders (0x23): Freigaben und Leistungsschalter.
uint8_t systemExpanderByte(bool motorPower, bool dribblerPower)
{
    uint8_t out = 0xFF;
    setBit(out, VDRIB_EN_BIT, dribblerPower);
    setBit(out, VMOT_EN_BIT, motorPower);
    setBit(out, LIGHT_BARRIER_EN_BIT, false);    // niedrig = Lichtschranken-LED an
    setBit(out, I2C_INTERCON_3V3_EN_BIT, false); // niedrig = geschaltete 3V3 am ESP-Bus-Intercon an
    if (LINE_EN_LEVEL >= 0)
        setBit(out, LINE_EN_T_BIT, LINE_EN_LEVEL == 1);
    return out;
}

/// Ausgaberegister zuerst, dann die Richtung der Pins: so entsteht kein kurzer falscher Pegel an den ESCON-Eingaengen.
/// Start im Stillstand: Enable aus, Bremse an. Eingaenge (true): SWITCH1, BUTTON1, UART_DIO1, ungenutzte Pins.
void initExpanders(bool motorPower)
{
    uint8_t stopA = 0xFF;
    uint8_t stopB = 0xFF;
    setBit(stopA, MotorVL_EN_BIT, false);
    setBit(stopA, MotorHL_EN_BIT, false);
    setBit(stopA, MotorDR_EN_BIT, false);
    setBit(stopB, MotorVR_EN_BIT, false);
    setBit(stopB, MotorHR_EN_BIT, false);
    Exp_MotorA.write(stopA);
    Exp_MotorB.write(stopB);
    Exp_System.write(systemExpanderByte(motorPower, false));

    Exp_MotorA.begin(false);                                                // alle acht Pins Ausgang
    Exp_MotorB.begin(false, false, false, false, false, false, true, true); // P6/P7 = SWITCH1/BUTTON1
    Exp_System.begin(false, false, false, LINE_EN_LEVEL < 0, true, false, true, true);
}
} // namespace
