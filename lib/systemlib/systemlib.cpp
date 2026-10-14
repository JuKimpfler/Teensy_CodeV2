#include <systemlib.h>

/* -------------- Instanzen -------------- */

BNO055 BNO(BNO_ADDRESS, &I2C_BNO);
Pathfinder AStar; // ~130 kB: nur global anlegen. begin() in systemlib::initialize()
// PowerDebugger PDS;

elapsedMillis cycleTimer;
elapsedMillis tempTimer;  // CPU-Temperatur 1x pro Sekunde
elapsedMillis powerTimer; // drosselt die INA219-Abfrage fuer die PDS-Kanaele
elapsedMicros cycleTimer_us;
elapsedMillis lastvaliddata_Radar_X;
elapsedMillis lastvaliddata_Radar_Y;

commlib communication;

I2C_bus i2cIR(I2C_IR);
Device_handle ir_ring_handle = {i2cIR, I2C_target{0x0A}};
IR_ring ir_ring(ir_ring_handle);

INA219 INA219_Main(INA_ADDR_MAIN);
INA219 INA219_Motor(INA_ADDR_MOTOR);
INA219 INA219_Drib(INA_ADDR_DRIB);

TCA9534DW Exp_MotorA(&I2C_TNS, EXP_ADDR_MOTOR_A); // 0x22: Motor 1 (VL), Motor 2 (HL), Dribbler
TCA9534DW Exp_MotorB(&I2C_TNS, EXP_ADDR_MOTOR_B); // 0x20: Motor 3 (VR), Motor 4 (HR), SWITCH1, BUTTON1
TCA9534DW Exp_System(&I2C_TNS, EXP_ADDR_SYSTEM);  // 0x23: VMOT_EN, VDRIB_EN, Freigaben

// Motor-ADC und Linien-Platine teilen sich den Bus `SPI`; die Linien-ADCs werden ueber den Dekoder (A, B, C) gewaehlt
ADC108S102 ADC_Motor(&SPI, ADC108S102_Motor_CS);
ADC108S102 ADC_Line1(&SPI, LINE_DEC_A, LINE_DEC_B, LINE_DEC_C, 0);
ADC108S102 ADC_Line2(&SPI, LINE_DEC_A, LINE_DEC_B, LINE_DEC_C, 1);
ADC108S102 ADC_Line3(&SPI, LINE_DEC_A, LINE_DEC_B, LINE_DEC_C, 2);
ADC108S102 ADC_Line4(&SPI, LINE_DEC_A, LINE_DEC_B, LINE_DEC_C, 3);
ADC108S102 ADC_Line5(&SPI, LINE_DEC_A, LINE_DEC_B, LINE_DEC_C, 4);
ADC108S102 ADC_Line6(&SPI, LINE_DEC_A, LINE_DEC_B, LINE_DEC_C, 5);

PID *headingControllerP;
ADRC *headingControllerA;
uint32_t lastMicrosContrl = 0;

fancyBTN BTN1(50, 6, 8, true);
fancyBTN BTN2(50, 6, 8, true);
fancyBTN BTN3(50, 6, 8, true);
fancyBTN BTN4(50, 6, 8, true);
fancyBTN BTN5(50, 6, 8, true);

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
/// Eingaenge (true): DIO4 der Motoren, DIO2 des Dribblers, SWITCH1, BUTTON1, UART_DIO1, ungenutzte Pins.
void initExpanders(bool motorPower)
{
    Exp_MotorA.write(0xFF);
    Exp_MotorB.write(0xFF);
    Exp_System.write(systemExpanderByte(motorPower, false));

    Exp_MotorA.begin(false, false, true, false, false, true, false, true);
    Exp_MotorB.begin(false, false, true, false, false, true, true, true);
    Exp_System.begin(false, false, false, LINE_EN_LEVEL < 0, true, false, true, true);
}
} // namespace

/* -------------- System Worker -------------- */

void systemlib::initialize()
{
    Wire1.begin();
    Wire1.setClock(I2C_WIRE1_CLOCK_HZ);
    Wire.begin();
    Wire.setClock(I2C_WIRE_CLOCK_HZ);

    initExpanders(_motorPowerEnabled);
    readExpanders();

    SPI.begin();

    USBC_UART.begin(115200);

    INA219_Main.begin(I2C_TNS);
    INA219_Motor.begin(I2C_TNS);
    INA219_Drib.begin(I2C_TNS);

    INA219_Main.setMaxCurrentShunt(INA_MAX_CURRENT_A, INA_MAIN_SHUNT_OHM);
    INA219_Motor.setMaxCurrentShunt(INA_MAX_CURRENT_A, INA_MOTOR_SHUNT_OHM);
    INA219_Drib.setMaxCurrentShunt(INA_MAX_CURRENT_A, INA_DRIB_SHUNT_OHM);

    BNO.begin();
    AStar.begin();
    calibBNO();

#if DPDS_Mode == D_Mode_Test
    PDS.init();
    setupPDS();
#endif

    MoDataVL.ID = MotorVL_ID;
    MoDataVR.ID = MotorVR_ID;
    MoDataHL.ID = MotorHL_ID;
    MoDataHR.ID = MotorHR_ID;
    MoDataDR.ID = MotorDR_ID;

    pinMode(BLDC_DO1, OUTPUT);
    digitalWrite(BLDC_DO1, LOW); // Dribbler gesperrt

    Radar.begin(I2C_RADAR); // richtet alle Sensoren ein und startet die erste Messrunde

    //ir_ring.read_gains(IR_Gains);
    //ir_ring.read_offsets(IR_Offsets);

    setDebugTimeout(0);

    ADC_Motor.begin();
    ADC_Line1.begin();
    ADC_Line2.begin();
    ADC_Line3.begin();
    ADC_Line4.begin();
    ADC_Line5.begin();
    ADC_Line6.begin();

    setupHeadingController();

    Position.X = 90;
    Position.Y = 120;
}

void systemlib::readSensors(void)
{
    // Zyklische Messung der Laufzeit in ms (read -> read)
    cycleTime = cycleTimer;
    cycleTime_us = cycleTimer_us;
    delay(debugTimeout);
    cycleTimer = 0;
    cycleTimer_us = 0;

    readBNO();
    Radar.update();


    // Motordrehzahlen -> Feldgeschwindigkeit. Heading als Mittel ueber den Zyklus,
    // damit die Drehung waehrend des Zyklus nicht als Richtungsfehler eingeht.
    float motorRpm[4] = {MoDataVL.Speed_real, MoDataVR.Speed_real, MoDataHL.Speed_real, MoDataHR.Speed_real};
    static float lastHeadingOdo = giveOrbitDir();
    const float hNow = giveOrbitDir();
    const float hMid = Util.wrapAngle(lastHeadingOdo + 0.5f * Util.wrapAngle(hNow - lastHeadingOdo));
    lastHeadingOdo = hNow;
    RobotVelocity vel = calculateFieldVelocityFromMotorRpm(motorRpm, hMid);
    calculatePosition(millis(), vel.vx_field, vel.vy_field);
    // readIR();
    updateArray();
    updateLine();

    readExpanders();

    // Werte fuer die PDS-Kanaele nachfuehren. Die INA219 haengen am I2C und
    // kosten je Abfrage spuerbar Zeit -- Akkuspannung aendert sich nicht im
    // 100-Hz-Takt, 10 Hz reichen voellig.
    uptime_s = millis() / 1000.0f;
    if (tempTimer >= 1000)
    {
        tempTimer = 0;
        CPU_Temp = tempmonGetTemp(); // Chiptemperatur, Alarm/Halt des Kerns bei 90 Grad C
    }
    if (powerTimer >= 100)
    {
        powerTimer = 0;
        Akku_V = giveMainVoltage();
        Akku_I = giveMainCurrent();
        Motor_V = giveMotorVoltage();
        Motor_I = giveMotorCurrent();
        Drib_I = giveDribblerCurrent();
    }

#if DPDS_Mode == D_Mode_Test
    PDS.update();
#endif

    updateHeadingController();
    updateMotors();
}

/* -------------- PDS (Power Debug System) -------------- */

/**
 * Bindet die Systemwerte an FESTE PDS-Kanaele.
 *
 * "Fest" ist Absicht: die Overlays in lib/PDS/channel_config.h (Feldansicht,
 * Akkuanzeige, Wertetabelle) zeigen auf Kanalnummern, nicht auf Namen. Wer
 * hier etwas verschiebt, muss channel_config.h und docs/PDS_Belegung.md
 * nachziehen.
 *
 * Kanal 0..31 gehoeren systemlib, ab 32 vergibt PDS.plot()/PDS.track() im
 * Sketch automatisch -- dafuer sorgt -DPDS_AUTO_CHANNEL_BASE=32 in
 * platformio.ini. Ohne dieses Flag wuerde der Sketch in diesen Block
 * hineinschreiben.
 *
 * bind() merkt sich nur den Zeiger; die Bibliothek holt die Werte danach
 * selbstaendig 100x/s ab. Im loop() ist dafuer nichts mehr zu tun.
 */
void systemlib::setupPDS(void)
{
#if DPDS_Mode == D_Mode_Test

    // Firmware-Stand, damit in der GUI sichtbar ist, welcher Code auf welchem
    // Roboter laeuft. Die Makros kommen aus den build_flags in platformio.ini.
#if defined(BUILD_VERSION) && defined(BUILD_DATE) && defined(BUILD_TIME)
    // setFirmwareVersion() kopiert in einen eigenen Puffer -- der String hier
    // darf danach wieder verschwinden.
    const String version = "v" + String(BUILD_VERSION) +
                           " (Build " + String(BUILD_DATE) +
                           " " + String(BUILD_TIME) + ")";
    PDS.setFirmwareVersion(version.c_str());
#endif

    /* ---- Versorgung ---------------------------------------------------- */
    PDS.bind("Timer", &uptime_s, 0, "s");
    PDS.bind("Akku_V", &Akku_V, 1, "V");
    PDS.bind("Akku_I", &Akku_I, 2, "mA");
    PDS.bind("Motor_V", &Motor_V, 3, "V");
    PDS.bind("Motor_I", &Motor_I, 4, "mA");

    /* ---- Lage ---------------------------------------------------------- */
    PDS.bind("Heading", &BNO_eulerZ, 5, "°");
    PDS.bind("Orbit_Dir", &BNO_OrbitDir, 6, "°");
    PDS.bind("BNO_Nullpunkt", &BNO_calibration, 7, "°");

    /* ---- Ball (IR) ----------------------------------------------------- */
    // readIR() ist in readSensors() derzeit auskommentiert -- die Kanaele
    // stehen aber schon bereit und zeigen bis dahin 0.
    PDS.bind("Ball_Winkel", &IR_Angle, 8, "°");
    PDS.bind("Ball_Dist", &IR_Distance, 11);

    /* ---- Zykluszeit ---------------------------------------------------- */
    PDS.bind("Zykluszeit", &cycleTime, 9, "ms");
    PDS.bind("Zykluszeit_us", &cycleTime_us, 10, "us");

    /* ---- Radar --------------------------------------------------------- */
    // Dist ist ein Rohwert ("Counts"), NICHT Millimeter -- siehe RadarArray.h.
    PDS.bind("Radar_Vo", &Radar[RadarVo].Strenght, 12, "Counts");
    PDS.bind("Radar_Li", &Radar[RadarLi].Strenght, 13, "Counts");
    PDS.bind("Radar_Hi", &Radar[RadarHi].Strenght, 14, "Counts");
    PDS.bind("Radar_Re", &Radar[RadarRe].Strenght, 15, "Counts");

    /* ---- Feldposition -------------------------------------------------- */
    // Kanal 16/17 sind die Koordinaten, auf die das "bodies"-Overlay der GUI
    // zeigt (Feld-Draufsicht in channel_config.h).
    PDS.bind("Pos_X", &Position.X, 16, "cm");
    PDS.bind("Pos_Y", &Position.Y, 17, "cm");
    PDS.bind("Pos_certanty_x", &Position.XValid, 18);
    PDS.bind("Pos_certanty_y", &Position.YValid, 19);

    /* ---- Motoren ------------------------------------------------------- */
    PDS.bind("Motor_VL", &MoDataVL.Speed_real, 20, "%");
    PDS.bind("Motor_VR", &MoDataVR.Speed_real, 21, "%");
    PDS.bind("Motor_HL", &MoDataHL.Speed_real, 22, "%");
    PDS.bind("Motor_HR", &MoDataHR.Speed_real, 23, "%");

    PDS.bind("Motor_VL", &MoDataVL.Dir_real, 24, "%");
    PDS.bind("Motor_VR", &MoDataVR.Dir_real, 25, "%");
    PDS.bind("Motor_HL", &MoDataHL.Dir_real, 26, "%");
    PDS.bind("Motor_HR", &MoDataHR.Dir_real, 27, "%");

    /* ---- Schalter ------------------------------------------------------ */
    PDS.bind("Start", &startSwitch, 28);
    PDS.bind("CPU_Temp", &CPU_Temp, 29, "°C");
    PDS.bind("Dribbler_I", &Drib_I, 30, "mA");

    // 26..31 bleiben frei fuer weitere Systemwerte.

#endif // Game_mode
}

/* -------------- Position/Radar -------------- */

/**
 * Alter eines Messwerts -> Gewicht zwischen 1 (brandaktuell) und 0 (uralt).
 * Faellt linear von 1 bei 0 ms auf 0 bei POS_AGE_MAX_MS.
 */
static float posAgeFactor(uint32_t age_ms)
{
    if (age_ms >= POS_AGE_MAX_MS)
        return 0.0f;

    return 1.0f - ((float)age_ms / (float)POS_AGE_MAX_MS);
}

/**
 * Tiefe, um die die Y-Achse an EINEM Feldende durch das Tor kuerzer wird.
 *
 * Beide Tore stehen mittig auf der X-Achse und gleich tief im Feld, der Wert
 * gilt also fuer beide Enden. Ob der Sensor die Torrueckwand oder die Feldwand
 * trifft, haengt allein davon ab, wo der Roboter auf X steht:
 *
 *   Abstand zur Feldmitte + FIELD_GOAL_EDGE_CM < Torbreite/2  -> sicher Tor
 *   Abstand zur Feldmitte - FIELD_GOAL_EDGE_CM > Torbreite/2  -> sicher Wand
 *   dazwischen -> nicht entscheidbar, die Radarkeule deckt beides ab
 *
 * Im Uebergangsband wird linear zwischen beiden Faellen geblendet -- der Fehler
 * bleibt damit unter der halben Tortiefe statt sie voll zu treffen -- und
 * sureness faellt auf 0, weil eben nicht feststeht, was der Sensor gesehen hat.
 *
 * @param x_cm      aktuelle X-Position (muss VOR der Y-Achse gerechnet sein)
 * @param sureness  [out] 1 = eindeutig Tor oder eindeutig Wand ... 0 = genau auf der Torkante
 * @return Verkuerzung der Achse an diesem Ende in cm
 */
float systemlib::goalOffset(float x_cm, float &sureness) const
{
    sureness = 1.0f;

    // Kein Tor eingetragen -> nichts zu korrigieren, und vor allem keine
    // Certainty verschenken.
    if (FIELD_GOAL_DEPTH_CM == 0.0f || FIELD_GOAL_WIDTH_CM <= 0.0f)
        return 0.0f;

    const float halfGoal = FIELD_GOAL_WIDTH_CM / 2.0f;
    const float dx = fabsf(x_cm - (FIELD_X_CM / 2.0f)); // Abstand zur Feldmitte

    if (dx <= halfGoal - FIELD_GOAL_EDGE_CM) // klar vor der Toroeffnung
        return FIELD_GOAL_DEPTH_CM;

    if (dx >= halfGoal + FIELD_GOAL_EDGE_CM) // klar vor der Feldwand
        return 0.0f;

    // Uebergangsband: t = 1 an der Innenkante (ganz Tor) ... 0 an der Aussenkante
    // (ganz Wand). Genau auf der Torkante (t = 0.5) ist die Entscheidung ein
    // Muenzwurf -> sureness = 0.
    const float t = constrain((halfGoal + FIELD_GOAL_EDGE_CM - dx) / (2.0f * FIELD_GOAL_EDGE_CM), 0.0f, 1.0f);
    sureness = fabsf(2.0f * t - 1.0f);

    return FIELD_GOAL_DEPTH_CM * t;
}

/**
 * Rechnet Motordrehzahl + Richtungs-Bool in Radgeschwindigkeit (cm/s) um.
 */
float systemlib::convertMotorRpmToWheelSpeedCmS(float motor_rpm)
{
    return motor_rpm * ODO_RPM_TO_CM_S;
}

/**
 * Berechnet aus den 4 Motordrehzahlen [VL, VR, HL, HR] und dem IMU-Winkel
 * die Feld-Geschwindigkeiten vx_field und vy_field (cm/s).
 *
 * Index-Zuordnung (passend zu setOmniWheelDir):
 * [0] = Vorne Links (45°), [1] = Vorne Rechts (315°),
 * [2] = Hinten Links (135°), [3] = Hinten Rechts (225°)
 *
 * Aenderungen gegenueber vorher:
 *  - vx_rob / vy_rob waren int -> Nachkommastellen gingen verloren (bei
 *    langsamer Fahrt kam oft exakt 0 raus, bei schneller ein Stufenmuster).
 *  - Kalibrierfaktoren im Roboter-Frame statt in der Rotation.
 *  - Die Matrix selbst ist unveraendert.
 */
RobotVelocity systemlib::calculateFieldVelocityFromMotorRpm(const float motor_rpm[4], float heading_deg)
{
    const float v_VL = convertMotorRpmToWheelSpeedCmS(motor_rpm[0]);
    const float v_VR = convertMotorRpmToWheelSpeedCmS(motor_rpm[1]);
    const float v_HL = convertMotorRpmToWheelSpeedCmS(motor_rpm[2]);
    const float v_HR = convertMotorRpmToWheelSpeedCmS(motor_rpm[3]);

    static constexpr float s = 0.70710678f; // √2/2

    float vy_rob = 0.5f * s * (v_VL - v_VR + v_HL - v_HR);
    float vx_rob = -0.5f * s * (v_VL - v_VR - v_HL + v_HR);

    vx_rob *= ODO_SCALE_X;
    vy_rob *= ODO_SCALE_Y;

    const float heading_rad = heading_deg * ODO_DEG_TO_RAD;
    const float cos_h = cosf(heading_rad);
    const float sin_h = sinf(heading_rad);

    RobotVelocity vel;
    vel.vx_field = vx_rob * cos_h - vy_rob * sin_h;
    vel.vy_field = vx_rob * sin_h + vy_rob * cos_h;

    PDS.Channel(190, vel.vx_field);
    PDS.Channel(191, vel.vy_field);

    return vel;
}

/**
 * Hilfsfunktion: Berechnet die minimal erforderliche Signalstärke in mdB (milli-dB)
 * basierend auf der gemessenen Distanz (in cm).
 */
static inline int32_t getMinStrengthMdb(float distCm)
{
    float distM = distCm / 100.0f;
    float minDb = POS_STRENGTH_MIN_NEAR_MDB - (distM * POS_STRENGTH_SLOPE_MDB_M);
    return (int32_t)max(minDb, POS_STRENGTH_MIN_FAR_MDB);
}

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

/**
 * Radar-Validierung (ehemals calcAxisPosition), liefert
 * aber KEINE fertige Position mehr, sondern 0..2 Messungen. Ueber Sprung,
 * Hinderniss und Vertrauen entscheidet der Filter, der die Vorhersage kennt.
 */
static AxisMeasSet radarAxisMeasurements(const RadarData &low, const RadarData &high, uint32_t ageLow, uint32_t ageHigh, float feldSize, float offLow, float offHigh)
{
    AxisMeasSet out;
    out.n = 0;

    const float span = feldSize - offLow - offHigh;
    const float distLow = (low.DistRaw / 10.0f) + POS_RADAR_OFFSET_CM; // ungefilterter Wert
    const float distHigh = (high.DistRaw / 10.0f) + POS_RADAR_OFFSET_CM;

    const float minDist = (RADAR_START_MM / 10.0f) + POS_RADAR_OFFSET_CM;
    const float blindDistThreshold = max(POS_BLIND_CM, minDist + 2.0f);

    bool lowOk = low.isCon && (low.Peaks > 0) && (low.Strenght >= getMinStrengthMdb(distLow)) && (distLow >= minDist) && (distLow <= span + POS_MAX_OVERSHOOT_CM);

    bool highOk = high.isCon && (high.Peaks > 0) && (high.Strenght >= getMinStrengthMdb(distHigh)) && (distHigh >= minDist) && (distHigh <= span + POS_MAX_OVERSHOOT_CM);

    // Blindbereich des Gegenuebers
    const bool lowFar = lowOk && (distLow > span - blindDistThreshold);
    const bool highFar = highOk && (distHigh > span - blindDistThreshold);

    if (lowFar && highFar)
    {
        lowOk = false;
        highOk = false;
    }
    else if (lowFar)
    {
        highOk = false;
    }
    else if (highFar)
    {
        lowOk = false;
    }

    auto certOf = [](float plaus, uint32_t age, int32_t strength) -> float
    {
        const float sf = constrain(1.0f + (strength + POS_STRENGTH_CERT_OFFSET_MDB) / POS_STRENGTH_CERT_SPAN_MDB, POS_STRENGTH_CERT_MIN, 1.0f);
        return constrain(plaus * posAgeFactor(age) * sf, 0.0f, 1.0f);
    };

    const float zLow = offLow + constrain(distLow, 0.0f, span);
    const float zHigh = offLow + constrain(span - distHigh, 0.0f, span);

    if (lowOk && highOk)
    {
        const float err = fabsf((distLow + distHigh) - span);

        if (err <= POS_PAIR_TOL_CM)
        {
            // Beide Waende sehen dasselbe -> sehr belastbar, ein gemittelter Wert
            const float plaus = POS_PLAUS_PAIR - (err / POS_PAIR_TOL_CM) * (POS_PLAUS_PAIR - POS_PLAUS_SINGLE);
            out.m[0] = makeMeas(0.5f * (zLow + zHigh),
                                certOf(plaus, max(ageLow, ageHigh), min(low.Strenght, high.Strenght)), 0,
                                max(low.Time, high.Time));
            out.n = 1;
        }
        else
        {
            // Widerspruch: ein Sensor sieht ein Hindernis. Beide getrennt an den
            // Filter geben, die Vorhersage entscheidet, welcher glaubwuerdig ist.
            out.m[0] = makeMeas(zLow, certOf(POS_PLAUS_CONFLICT, ageLow, low.Strenght), -1, low.Time);
            out.m[1] = makeMeas(zHigh, certOf(POS_PLAUS_CONFLICT, ageHigh, high.Strenght), +1, high.Time);
            out.n = 2;
        }
    }
    else if (lowOk)
    {
        out.m[0] = makeMeas(zLow, certOf(POS_PLAUS_SINGLE, ageLow, low.Strenght), -1, low.Time);
        out.n = 1;
    }
    else if (highOk)
    {
        out.m[0] = makeMeas(zHigh, certOf(POS_PLAUS_SINGLE, ageHigh, high.Strenght), +1, high.Time);
        out.n = 1;
    }
    return out;
}

static void fusionPredict(AxisFilter &f, float dx, float speed, float omega, float dt, float limit)
{
    f.x = constrain(f.x + dx, 0.0f, limit);
    const float q = FUS_Q_BASE + (FUS_Q_SPEED * speed) * (FUS_Q_SPEED * speed) + (FUS_Q_ROT * omega) * (FUS_Q_ROT * omega);
    f.P += q * dt;
}

/** @return true, wenn die Messung den Zustand veraendert hat */
static bool fusionUpdate(AxisFilter &f, const AxisMeas &m, float tiltFactor, uint32_t now_ms)
{
    const float cert = max(m.cert, FUS_MIN_CERT);
    const float sigma = FUS_SIGMA_RADAR_CM / cert;
    const float R = sigma * sigma * tiltFactor;

    // Der Radarwert ist alt: er gehoert zur Position von vor ~Latenz, nicht zu "jetzt".
    const uint32_t tMeas = (m.t > FUS_RADAR_LATENCY_MS) ? (m.t - FUS_RADAR_LATENCY_MS) : 0;
    float xRef = f.x;
    if (!histLookup(f, tMeas, xRef))
        xRef = f.x;

    const float y = m.z - xRef;
    const float S = f.P + R;

    f.innov = y;
    f.accepted = false;

    if (y * y <= FUS_GATE_SIGMA * FUS_GATE_SIGMA * S)
    {
        const float K = f.P / S;
        const float d = K * y;
        f.x += d;
        f.P *= (1.0f - K);
        histShift(f, tMeas, d);
        for (uint8_t i = 0; i < 3; i++)
            f.candActive[i] = false;
        f.accepted = true;
        return true;
    }

    // Ausreisser: Hindernis / Sprung. Merken, ob er stabil bleibt.
    const bool obstacleLike = (m.suspect != 0) && (y * (float)m.suspect > 0.0f);
    const uint32_t need = obstacleLike ? FUS_REACQ_OBST_MS : FUS_REACQ_MS;
    const uint8_t idx = (m.suspect < 0) ? 0 : (m.suspect > 0 ? 1 : 2);

    if (!f.candActive[idx] || fabsf(m.z - f.cand[idx]) > FUS_REACQ_TOL_CM)
    {
        f.cand[idx] = m.z;
        f.candSince[idx] = now_ms;
        f.candActive[idx] = true;
    }
    else if (now_ms - f.candSince[idx] >= need)
    {
        f.x += y; // = Messwert + Weg seit dem Messzeitpunkt
        f.P = R;
        histShift(f, tMeas, y);
        for (uint8_t i = 0; i < 3; i++)
            f.candActive[i] = false;
        f.accepted = true;
        return true;
    }
    return false;
}

/**
 * Feldposition berechnen (Odometrie-gefuehrt, Radar korrigiert).
 *
 * Signatur unveraendert. current_time_ms muss millis() sein (gleiche Zeitbasis
 * wie Radar[..].Time); dt kommt aus micros().
 *
 * Gegen das "Hinterherhinken":
 *  - Motor: Die Analog-Rueckmeldung hinkt der echten Drehzahl nach. Aus der
 *    Beschleunigung (zwischen zwei ADC-Messungen) wird die Geschwindigkeit um
 *    FUS_MOTOR_LAG_MS + Alter der Messung nach vorne geschoben.
 *  - Radar: Der Messwert wird mit der Position von damals verglichen
 *    (Historie, FUS_RADAR_LATENCY_MS), die Korrektur wirkt dann auf "jetzt".
 */
PositionData systemlib::calculatePosition(uint32_t current_time_ms, float vx_field, float vy_field, bool use_radar, bool use_odometry)
{
    static bool first = true;
    static uint32_t lastUs = 0;
    static float lastHeading = 0.0f;
    static float vxPrev = 0.0f; // zuletzt verwendete (vorgeschobene) Geschwindigkeit
    static float vyPrev = 0.0f;
    static uint32_t radarUsed[4] = {0, 0, 0, 0};   // Time des zuletzt verrechneten Radarwerts je Sensor
    static uint32_t lastSampleUs = 0;              // ADC-Zeitstempel der letzten Auswertung
    static float vxSample = 0.0f, vySample = 0.0f; // Messwerte der vorigen ADC-Messung
    static float axLP = 0.0f, ayLP = 0.0f;         // geglaettete Beschleunigung [cm/s²]
    static float latEstX = 0.0f, latEstY = 0.0f;   // Tuning-Hilfe, siehe PDS 196/197

    // ---- Zeit & Drehrate ------------------------------------------------
    const uint32_t nowUs = micros();
    const float heading = giveOrbitDir(); // Grad, +-180
    float dt = 0.0f;
    float omega = 0.0f; // rad/s

    if (first)
    {
        filtX.x = Position.X;
        filtY.x = Position.Y;
        vxPrev = vx_field;
        vyPrev = vy_field;
        first = false;
    }
    else
    {
        dt = constrain((nowUs - lastUs) * 1e-6f, 0.0f, 0.05f);
        if (dt > 0.0f)
            omega = DEG_TO_RAD * Util.wrapAngle(heading - lastHeading) / dt;
    }
    lastUs = nowUs;
    lastHeading = heading;

    // ---- Geschwindigkeit: Totzeit der Analogrueckmeldung ausgleichen ------
    float vxm = vx_field;
    float vym = vy_field;
    if (sqrtf(vxm * vxm + vym * vym) < FUS_V_DEADBAND)
    {
        vxm = 0.0f;
        vym = 0.0f;
    }

    const uint32_t sUs = motorSampleUs;
    if (sUs != lastSampleUs)
    {
        if (lastSampleUs != 0 && sUs != 0)
        {
            const float sdt = (uint32_t)(sUs - lastSampleUs) * 1e-6f; // Abstand zweier ADC-Messungen
            if (sdt > 0.0005f && sdt < 0.1f)
            {
                const float ax = constrain((vxm - vxSample) / sdt, -FUS_MAX_ACCEL, FUS_MAX_ACCEL);
                const float ay = constrain((vym - vySample) / sdt, -FUS_MAX_ACCEL, FUS_MAX_ACCEL);
                axLP += FUS_ACCEL_LPF * (ax - axLP);
                ayLP += FUS_ACCEL_LPF * (ay - ayLP);
            }
        }
        vxSample = vxm;
        vySample = vym;
        lastSampleUs = sUs;
    }

    // Alter der Messung: ADC wurde am Ende des letzten Zyklus gelesen
    float age = (sUs != 0) ? (uint32_t)(nowUs - sUs) * 1e-6f : 0.0f;
    age = constrain(age, 0.0f, 0.05f);
    const float lead = FUS_MOTOR_LAG_MS * 1e-3f + age;

    const float vx = leadVelocity(vxm, axLP, lead);
    const float vy = leadVelocity(vym, ayLP, lead);
    const float speed = sqrtf(vx * vx + vy * vy);

    // ---- Vorhersage (Raeder) --------------------------------------------
    if (use_odometry)
    {
        // Trapez: Mittel aus voriger und aktueller (vorgeschobener) Geschwindigkeit
        fusionPredict(filtX, 0.5f * (vx + vxPrev) * dt, speed, fabsf(omega), dt, FIELD_X_CM);
        fusionPredict(filtY, 0.5f * (vy + vyPrev) * dt, speed, fabsf(omega), dt, FIELD_Y_CM);
    }
    else
    {
        fusionPredict(filtX, 0.0f, FUS_ASSUMED_SPEED, fabsf(omega), dt, FIELD_X_CM);
        fusionPredict(filtY, 0.0f, FUS_ASSUMED_SPEED, fabsf(omega), dt, FIELD_Y_CM);
    }
    vxPrev = vx;
    vyPrev = vy;

    // Historie fuer die Radar-Latenzkompensation (nach der Vorhersage, vor den Korrekturen)
    histPush(filtX, current_time_ms, filtX.x);
    histPush(filtY, current_time_ms, filtY.x);
    filtX.accepted = false;
    filtY.accepted = false;

    // ---- Korrektur (Radar) ----------------------------------------------
    // Sensoren sind am Roboter fest montiert: bei ~180° schauen Li/Re und
    // Vo/Hi auf die jeweils andere Wand. Bei ~90° / 270° oder mitten in der
    // Drehung ist die Zuordnung unbrauchbar -> nur Raeder.
    if (use_radar)
    {
        const float hAbs = fabsf(heading);
        const bool backward = (hAbs > 90.0f);
        const float tilt = backward ? (180.0f - hAbs) : hAbs;

        if (tilt <= FUS_MAX_TILT_DEG)
        {
            const float tn = tilt / FUS_TILT_SCALE_DEG;
            const float tiltFactor = 1.0f + tn * tn; // 25° -> Sigma x2.7

            // Das Radar liefert nur alle paar Zyklen einen NEUEN Wert. Jeder Wert
            // darf nur einmal in den Filter, sonst zaehlt er mehrfach als
            // "unabhaengige" Messung und der Filter wird ueberselbstsicher.
            auto isFresh = [&](uint8_t a, uint8_t b) -> bool
            {
                return (Radar[a].Time != radarUsed[a]) || (Radar[b].Time != radarUsed[b]);
            };
            auto markUsed = [&](uint8_t a, uint8_t b)
            {
                radarUsed[a] = Radar[a].Time;
                radarUsed[b] = Radar[b].Time;
            };

            // X
            const uint8_t xl = backward ? RadarRe : RadarLi;
            const uint8_t xh = backward ? RadarLi : RadarRe;
            if (isFresh(xl, xh))
            {
                AxisMeasSet mx = radarAxisMeasurements(Radar[xl], Radar[xh],
                                                       Radar.giveAge(xl), Radar.giveAge(xh),
                                                       FIELD_X_CM, 0.0f, 0.0f);
                for (uint8_t i = 0; i < mx.n; i++)
                    if (fusionUpdate(filtX, mx.m[i], tiltFactor, current_time_ms))
                        lastvaliddata_Radar_X = 0;
                markUsed(xl, xh);
            }

            // Tor-Offset aus der GEFILTERTEN X-Position (ruhiger als Rohradar)
            const float sigmaX = sqrtf(filtX.P);
            const bool xFallback = (sigmaX > FUS_LOST_SIGMA);
            float goalSure = 1.0f;
            const float goalOff = goalOffset(xFallback ? FIELD_X_CM / 2.0f : filtX.x, goalSure);

            if (xFallback)
                goalSure = min(goalSure, 0.5f);
            else if (sigmaX > FIELD_GOAL_EDGE_CM)
                goalSure = min(goalSure, (float)(FIELD_GOAL_EDGE_CM / sigmaX)); // X unsicher -> Tor/Wand unsicher

            Position.AtGoal = (goalOff > 0.5f);

            // Y
            const uint8_t yl = backward ? RadarVo : RadarHi;
            const uint8_t yh = backward ? RadarHi : RadarVo;
            if (isFresh(yl, yh))
            {
                AxisMeasSet my = radarAxisMeasurements(Radar[yl], Radar[yh],
                                                       Radar.giveAge(yl), Radar.giveAge(yh),
                                                       FIELD_Y_CM, goalOff, goalOff);
                const float goalFactor = POS_PLAUS_GOAL_EDGE + (1.0f - POS_PLAUS_GOAL_EDGE) * goalSure;
                for (uint8_t i = 0; i < my.n; i++)
                {
                    my.m[i].cert *= goalFactor;
                    if (fusionUpdate(filtY, my.m[i], tiltFactor, current_time_ms))
                        lastvaliddata_Radar_Y = 0;
                }
                markUsed(yl, yh);
            }
        }
    }

    // ---- Tuning-Hilfe: wie falsch ist FUS_RADAR_LATENCY_MS? ----------------
    // Bei Fahrt mit v > 30 cm/s ist Innovation = -v * (Latenz_echt - Latenz_angenommen).
    // Positiver Wert in ms -> FUS_RADAR_LATENCY_MS um diesen Wert ERHOEHEN (Mittelwert ueber Zeit).
    if (filtX.accepted && fabsf(vx) > 30.0f)
        latEstX += 0.05f * ((-filtX.innov / vx) * 1000.0f - latEstX);
    if (filtY.accepted && fabsf(vy) > 30.0f)
        latEstY += 0.05f * ((-filtY.innov / vy) * 1000.0f - latEstY);

    // ---- Ausgabe --------------------------------------------------------
    const float sx = sqrtf(filtX.P);
    const float sy = sqrtf(filtY.P);

    Position.X = (sx > FUS_LOST_SIGMA) ? -1.0f : filtX.x;
    Position.Y = (sy > FUS_LOST_SIGMA) ? -1.0f : filtY.x;

    Position.XValid = (sx <= FUS_VALID_SIGMA);
    Position.YValid = (sy <= FUS_VALID_SIGMA);
    Position.Certainty = (Position.XValid && Position.YValid)
                             ? 1.0f / (1.0f + FUS_CERT_SLOPE * max(sx, sy))
                             : 0.0f;

    // Debug: Innovation = Radar minus Vorhersage. Springt sie oft -> Radar oder Odometrie tunen.
    PDS.Channel(192, filtX.innov);
    PDS.Channel(193, filtY.innov);
    PDS.Channel(194, sx);
    PDS.Channel(195, sy);
    PDS.Channel(196, latEstX);
    PDS.Channel(197, latEstY);

    return Position;
}

void systemlib::calibRadar()
{
    Radar.calib();
}

/* -------------- ADC/Line -------------- */

uint16_t systemlib::giveADC(uint8_t Port, uint8_t Pin)
{
    if (Port == 0)
    {
        return ADC_Line1.read(Pin);
    }
    else if (Port == 1)
    {
        return ADC_Line2.read(Pin);
    }
    else if (Port == 2)
    {
        return ADC_Line3.read(Pin);
    }
    else if (Port == 3)
    {
        return ADC_Line4.read(Pin);
    }
    else if (Port == 4)
    {
        return ADC_Line5.read(Pin);
    }
    else if (Port == 5)
    {
        return ADC_Line6.read(Pin);
    }
    else
    {
        return 0;
    }
}

void systemlib::updateLine()
{
    uint8_t line_summe = 0;
    int8_t line_Angle = 0;
    bool line_touching;

    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            if (i == 0)
            {
                line_array[j] = ADC_Line1.read(j);
            }
            else if (i == 1)
            {
                line_array[8 + j] = ADC_Line2.read(j);
            }
            else if (i == 2)
            {
                line_array[16 + j] = ADC_Line3.read(j);
            }
            else if (i == 3)
            {
                line_array[24 + j] = ADC_Line4.read(j);
            }
            else if (i == 4)
            {
                line_array[32 + j] = ADC_Line5.read(j);
            }
            else if (i == 5)
            {
                line_array[40 + j] = ADC_Line6.read(j);
            }
        }
    }

    for (int i = 0; i < 48; i++)
    {
        line_bool_array[i] = line_array[i] > LINE_THRESHOLD[i];
        if (line_bool_array[i])
        {
            line_summe++;
        }
    }

    if (line_summe > 1)
    {
        // 2) zwei Sensoren mit maximaler Winkel-Distanz finden (kreisförmig korrekt)
        float maxDistanz = -1.0f;
        float sensorA = 0.0f, sensorB = 0.0f;

        int aktiveSensoren[line_summe];
        int count = 0;
        for (int i = 0; i < 48; i++)
        {
            if (line_bool_array[i] == true)
            {
                aktiveSensoren[count] = Util.wrapAngle((i * 7.5));
                count++;
            }
        }

        for (int i = 0; i < line_summe; i++)
        {
            for (int j = i + 1; j < line_summe; j++)
            {
                float dist = fabsf(Util.wrapAngle(aktiveSensoren[i] - aktiveSensoren[j]));
                if (dist > maxDistanz)
                {
                    maxDistanz = dist;
                    sensorA = aktiveSensoren[i];
                    sensorB = aktiveSensoren[j];
                }
            }
        }

        // 3) Mittelwinkel zwischen A und B bestimmen (2 Möglichkeiten)
        float delta = Util.wrapAngle(sensorB - sensorA);
        float mitte1 = Util.wrapAngle(sensorA + (delta * 0.5f));
        float mitte2 = Util.wrapAngle(mitte1 + 180.0f);

        // 4) Mitte wählen, die näher liegt
        line_Angle = (fabsf(Util.wrapAngle(mitte1 - mitte2))) < (fabsf(Util.wrapAngle(mitte2 - sensorA))) ? mitte1 : mitte2;
        line_touching = true;
    }
    else if (line_summe == 1)
    {
        line_touching = true;
        for (int i = 0; i < 32; i++)
        {
            if (line_bool_array[i] == true)
            {
                line_Angle = (i * 11.25) - 90;
            }
        }
    }
    else
    {
        line_touching = false;
    }

    LineDepth = line_summe;
    LineWinkel = line_Angle;
    LinePresent = line_touching;
}

/* -------------- BNO -------------- */
void systemlib::calibBNO(void)
{
    BNO_calibration = BNO_eulerZ;
}

void systemlib::readBNO(void)
{
    BNO_eulerZ = BNO.eulHeading();
    BNO_OrbitDir = Util.wrapAngle(BNO_eulerZ - BNO_calibration);
}

/* -------------- IR -------------- */
void systemlib::readIR(void)
{
    IR_rawAngle = ir_ring.read_ball_angle();
    IR_rawDistance = ir_ring.read_ball_distance();

#if false
    ir_ring.read_raw_values(IR_rawSensorData);
    ir_ring.read_tssp(IR_TSSP);
#endif

    IR_Angle = Util.wrapAngle(IR_rawAngle - BNO_OrbitDir);
    IR_Distance = IR_rawDistance; // Hier muss noch Mathe hin
}

void systemlib::writeIRCalibration(uint8_t sensorNumber, int16_t offset, int16_t gain)
{
    ir_ring.write_offset(sensorNumber, offset);
    ir_ring.write_gain(sensorNumber, gain);
}

void systemlib::writeIRCalibration(const int16_t (&offsets)[16], const uint16_t (&gains)[16])
{
    ir_ring.write_offsets(offsets);
    ir_ring.write_gains(gains);
}

void systemlib::calibIR(void)
{
    ir_ring.read_raw_values(IR_rawSensorData); // only needed if disabled in readIR()

    for (int i = 0; i < 16; i++)
    {
        if (IR_rawSensorData[i] > IR_Gains[i])
            IR_Gains[i] = IR_rawSensorData[i];

        if (IR_rawSensorData[i] < IR_Offsets[i])
            IR_Offsets[i] = IR_rawSensorData[i];
    }

    ir_ring.write_offsets(IR_Offsets);
    ir_ring.write_gains(IR_Gains);
}

/* ---------- Communication -----------*/

// send

void systemlib::communicationSend(void)
{
    communication.sendOwnFlush();
}

void systemlib::communicationSend(float positionX, float positionY, float ballAngle, float ballDistance, float goalAngle, float goalDistance)
{
    communication.setOwnFlush(positionX, positionY, ballAngle, ballDistance, goalAngle, goalDistance);
    communication.sendOwnFlush();
}

float systemlib::giveMainVoltage(void)
{
    return (float)INA219_Main.getBusVoltage();
}

float systemlib::giveMotorVoltage(void)
{
    return (float)INA219_Motor.getBusVoltage();
}

float systemlib::giveMainCurrent(void)
{
    return (float)INA219_Main.getCurrent_mA();
}

float systemlib::giveDribblerCurrent(void)
{
    return (float)INA219_Drib.getCurrent_mA();
}

float systemlib::giveMotorCurrent(void)
{
    INA_filter[0] = INA_filter[1];
    INA_filter[1] = INA_filter[2];
    INA_filter[2] = INA_filter[3];
    INA_filter[3] = INA_filter[4];
    INA_filter[4] = INA_filter[5];
    INA_filter[5] = INA219_Motor.getCurrent_mA();

    int Summe = 0;
    for (int i = 0; i < 6; i++)
    {
        Summe += INA_filter[i];
    }
    Summe = Summe / 6;
    return Summe;
}

void systemlib::setMotorData(uint8_t ID, uint8_t Speed, bool Dir, bool Enable, bool Break)
{
    if (ID == MoDataVR.ID)
    {
        MoDataVR.Break = Break;
        MoDataVR.Speed = Speed;
        MoDataVR.Enable = Enable;
        MoDataVR.Dir = Dir;
    }
    else if (ID == MoDataHR.ID)
    {
        MoDataHR.Break = Break;
        MoDataHR.Speed = Speed;
        MoDataHR.Enable = Enable;
        MoDataHR.Dir = Dir;
    }
    else if (ID == MoDataVL.ID)
    {
        MoDataVL.Break = Break;
        MoDataVL.Speed = Speed;
        MoDataVL.Enable = Enable;
        MoDataVL.Dir = Dir;
    }
    else if (ID == MoDataHL.ID)
    {
        MoDataHL.Break = Break;
        MoDataHL.Speed = Speed;
        MoDataHL.Enable = Enable;
        MoDataHL.Dir = Dir;
    }
    else if (ID == MoDataDR.ID)
    {
        MoDataDR.Break = Break;
        MoDataDR.Speed = Speed;
        MoDataDR.Enable = Enable;
        MoDataDR.Dir = Dir;
    }
}

void systemlib::setMotorData(uint8_t ID, int8_t Speed, bool Enable, bool Break)
{
    bool Dir;

    if (Speed < 0)
    {
        Dir = false;
    }
    else
    {
        Dir = true;
    }

    Speed = abs(Speed);

    if (ID == MoDataVR.ID)
    {
        MoDataVR.Break = Break;
        MoDataVR.Speed = Speed;
        MoDataVR.Enable = Enable;
        MoDataVR.Dir = Dir;
    }
    else if (ID == MoDataHR.ID)
    {
        MoDataHR.Break = Break;
        MoDataHR.Speed = Speed;
        MoDataHR.Enable = Enable;
        MoDataHR.Dir = Dir;
    }
    else if (ID == MoDataVL.ID)
    {
        MoDataVL.Break = Break;
        MoDataVL.Speed = Speed;
        MoDataVL.Enable = Enable;
        MoDataVL.Dir = Dir;
    }
    else if (ID == MoDataHL.ID)
    {
        MoDataHL.Break = Break;
        MoDataHL.Speed = Speed;
        MoDataHL.Enable = Enable;
        MoDataHL.Dir = Dir;
    }
    else if (ID == MoDataDR.ID)
    {
        MoDataDR.Break = Break;
        MoDataDR.Speed = Speed;
        MoDataDR.Enable = Enable;
        MoDataDR.Dir = Dir;
    }
}

void systemlib::updateMotors(void)
{
    // Ist die Motorversorgung abgeschaltet (setMotorPower), stehen alle Antriebe still
    const bool motorsDisabled = !_motorPowerEnabled;

    MotorData *motors[5] = {&MoDataVL, &MoDataVR, &MoDataHL, &MoDataHR, &MoDataDR};
    for (MotorData *m : motors)
    {
        if (m->Speed == 0 || motorsDisabled)
        {
            m->Speed = 0;
            m->Enable = false;
            m->Break = true;
        }
    }

    analogWrite(MotorVL_PWM, abs(MoDataVL.Speed) * 2.55);
    analogWrite(MotorVR_PWM, abs(MoDataVR.Speed) * 2.55);
    analogWrite(MotorHR_PWM, abs(MoDataHR.Speed) * 2.55);
    analogWrite(MotorHL_PWM, abs(MoDataHL.Speed) * 2.55);
    analogWrite(MotorDR_PWM, abs(MoDataDR.Speed) * 2.55);
    digitalWrite(BLDC_DO1, MoDataDR.Enable);

    // Ausgabebytes der Expander (Bit n = Pin Pn). Eingangspins bleiben auf 1, ihr Wert ist ohne Wirkung.
    // Richtungsbit ist 0 bei Dir = true (wie bisher), Enable-Bit ist 1 bei Enable.
    uint8_t outA = 0xFF;
    uint8_t outB = 0xFF;
    setBit(outA, MotorVL_DIR_BIT, !MoDataVL.Dir);
    setBit(outA, MotorVL_EN_BIT, MoDataVL.Enable);
    setBit(outA, MotorHL_DIR_BIT, !MoDataHL.Dir);
    setBit(outA, MotorHL_EN_BIT, MoDataHL.Enable);
    setBit(outA, MotorDR_DIR_BIT, !MoDataDR.Dir);
    setBit(outB, MotorVR_DIR_BIT, !MoDataVR.Dir);
    setBit(outB, MotorVR_EN_BIT, MoDataVR.Enable);
    setBit(outB, MotorHR_DIR_BIT, !MoDataHR.Dir);
    setBit(outB, MotorHR_EN_BIT, MoDataHR.Enable);
    const uint8_t outC = systemExpanderByte(_motorPowerEnabled, MoDataDR.Enable);

    // Die Bytes aendern sich nur bei Richtungs-/Enable-Wechseln. Schreiben nur dann, plus eine
    // regelmaessige Auffrischung (falls ein Expander mal zurueckgesetzt wurde).
    struct ExpWriter
    {
        uint8_t last = 0;
        uint32_t lastMs = 0;
        bool written = false;
    };
    static ExpWriter writer[3];
    const uint32_t nowMs = millis();
    auto writeIfNeeded = [nowMs](ExpWriter &w, TCA9534DW &exp, uint8_t value)
    {
        if (!w.written || value != w.last || (nowMs - w.lastMs) >= MOTOR_EXP_REFRESH_MS)
        {
            exp.write(value);
            w.last = value;
            w.lastMs = nowMs;
            w.written = true;
        }
    };
    writeIfNeeded(writer[0], Exp_MotorA, outA);
    writeIfNeeded(writer[1], Exp_MotorB, outB);
    writeIfNeeded(writer[2], Exp_System, outC);

    /*for(int i = 0 ; i<8 ; i++){
        Serial.print("channel ");
        Serial.print(i);
        Serial.print(" : ");
        Serial.println(ADC_Motor.read(i));
    }

    Serial.println(ADC_Motor.read(0)-ADC_Motor.read(1));
    */

    motorSampleUs = micros(); // Zeitstempel der Drehzahlmessung (fuer Totzeitausgleich in calculatePosition)
    MoDataVL.Speed_real = ADC_Motor.read(MotorVL_AP1, true, true);
    MoDataVL.Current = ADC_Motor.read(MotorVL_AP2, true, false);
    MoDataVR.Speed_real = ADC_Motor.read(MotorVR_AP1, true, true);
    MoDataVR.Current = ADC_Motor.read(MotorVR_AP2, true, false);
    MoDataHL.Speed_real = ADC_Motor.read(MotorHL_AP1, true, true);
    MoDataHL.Current = ADC_Motor.read(MotorHL_AP2, true, false);
    MoDataHR.Speed_real = ADC_Motor.read(MotorHR_AP1, true, true);
    MoDataHR.Current = ADC_Motor.read(MotorHR_AP2, true, false);
}

MotorData systemlib::giveMotorData(uint8_t ID)
{
    MotorData Error_Data;
    Error_Data.ID = 100;

    if (ID == MoDataVR.ID)
    {
        return MoDataVR;
    }
    else if (ID == MoDataHR.ID)
    {
        return MoDataHR;
    }
    else if (ID == MoDataVL.ID)
    {
        return MoDataVL;
    }
    else if (ID == MoDataHL.ID)
    {
        return MoDataHL;
    }
    else if (ID == MoDataDR.ID)
    {
        return MoDataDR;
    }
    else
    {
        return Error_Data;
    }
}

/* -------------- Expanders -------------- */
void systemlib::readExpanders(void)
{
    // Eine I2C-Transaktion je Zyklus: nur der Expander 0x20 hat Eingaenge (SWITCH1, BUTTON1).
    Exp_MotorB.update();

    SWI[0] = Exp_MotorB.givePinState(SWITCH1_BIT);
    startSwitch = SWI[0];

    BTN1.update(Exp_MotorB.givePinState(BUTTON1_BIT));

    fancyBTNstates[0] = BTN1.giveStatusCode();
    fancyBTNstates[1] = BTN2.giveStatusCode();
    fancyBTNstates[2] = BTN3.giveStatusCode();
    fancyBTNstates[3] = BTN4.giveStatusCode();
    fancyBTNstates[4] = BTN5.giveStatusCode();
}

bool systemlib::giveExpander(uint8_t expnum, uint8_t portnum)
{
    if (expnum == 0)
    {
        return Exp_MotorB.readPin(portnum); // 0x20
    }
    else if (expnum == 1)
    {
        return Exp_MotorA.readPin(portnum); // 0x22
    }
    else if (expnum == 2)
    {
        return Exp_System.readPin(portnum); // 0x23
    }
    else
    {
        return false;
    }
}

void systemlib::setupHeadingController(bool useADRC)
{
    _useADRC = useADRC;
    if (_useADRC)
    {
        cfgA.order = ADRCOrder::SECOND; // Winkel = Strecke 2. Ordnung
        cfgA.Ts = HDG_TS_NOMINAL_S;
        cfgA.dtMin = HDG_DT_MIN_S;
        cfgA.dtMax = HDG_DT_MAX_S;
        cfgA.wc = HDG_ADRC_WC;
        cfgA.wo = HDG_ADRC_WO;
        cfgA.b0 = HDG_ADRC_B0;
        cfgA.uMin = HDG_U_MIN;
        cfgA.uMax = HDG_U_MAX;
        cfgA.angleMode = true; // WICHTIG: sorgt fuer kuerzesten Drehweg (kein +-pi Sprung)
        cfgA.outputRateLimit = HDG_RATE_LIMIT;

        headingControllerA = new ADRC(cfgA);
        headingControllerA->reset(DEG_TO_RAD * BNO_OrbitDir);
        lastMicrosContrl = micros();
    }
    else
    {
        cfgP.Ts = HDG_TS_NOMINAL_S;
        cfgP.dtMin = HDG_DT_MIN_S;
        cfgP.dtMax = HDG_DT_MAX_S;

        cfgP.Kp = HDG_PID_KP;
        cfgP.Ki = HDG_PID_KI;
        cfgP.Kd = HDG_PID_KD;

        cfgP.uMin = HDG_U_MIN;
        cfgP.uMax = HDG_U_MAX;
        cfgP.angleMode = true;               // WICHTIG: kuerzester Drehweg, kein +-pi Sprung
        cfgP.derivativeOnMeasurement = true; // kein Kick bei Sollwinkel-Spruengen
        cfgP.Tf = HDG_PID_TF_S;
        cfgP.outputRateLimit = HDG_RATE_LIMIT;

        headingControllerP = new PID(cfgP);
        headingControllerP->reset(DEG_TO_RAD * BNO_OrbitDir);
        lastMicrosContrl = micros();
    }
}

void systemlib::setHeadingControllerConfig()
{
    if (_useADRC)
    {
        headingControllerA->setConfig(cfgA);
    }
    else
    {
        headingControllerP->setConfig(cfgP);
    }
}

float systemlib::giveHeadingController()
{
    if (_useADRC)
    {
        return ADRC_Out;
    }
    else
    {
        return PID_Out;
    }
}

void systemlib::setHeading(float Heading)
{
    _OrbitHeading = Heading;
}

void systemlib::resetHeadingController()
{
    if (_useADRC)
    {
        headingControllerA->reset(DEG_TO_RAD * BNO_OrbitDir);
    }
    else
    {
        headingControllerP->reset(DEG_TO_RAD * BNO_OrbitDir);
    }
}

void systemlib::updateHeadingController()
{
    float dt = (micros() - lastMicrosContrl) * 1e-6f; // us -> s
    lastMicrosContrl = micros();

    if (_useADRC)
    {
        ADRC_Out = headingControllerA->update(_OrbitHeading, DEG_TO_RAD * giveOrbitDir(), dt);
    }
    else
    {
        PID_Out = headingControllerP->update(_OrbitHeading, DEG_TO_RAD * giveOrbitDir(), dt);
    }
}

bool systemlib::giveStart(void)
{
    if (RCJ_present)
    {
        return RCJ && !RCJ_Disable;
    }
    else
    {
        return startSwitch;
    }
}

void systemlib::moveToPoint(uint16_t X, uint16_t Y, uint8_t Speed, int16_t Andrehwinkel)
{
    float dx = X - Position.X;
    float dy = Y - Position.Y;

    float distanzCm = sqrtf(dy * dy + dx * dx);
    Speed = Speed * constrain(distanzCm / 40.0f, 0.0f, 1.0f); // 0 = am Ziel, 1 = 40cm oder mehr entfernt

    if (abs(distanzCm) < 5)
    {
        Speed = 0;
    }

    setHeading(Andrehwinkel * DEG_TO_RAD);
    setOmniWheelDir(Util.wrapAngle(atan2f(-dx, dy) * (180.0f / PI)), Speed, giveHeadingController() * 100);
}

namespace
{
struct MoveState
{
    bool active = false;
    bool arrived = false;
    bool correcting = false; // nach dem Wegschieben: Feinkorrektur mit MinSpeed
    float tx = 0.0f, ty = 0.0f;
    uint8_t pattern = 0, maxS = 0, minS = 0;
    float dist0 = 0.0f; // Abstand beim Start des Patterns
    float progress = 0.0f;
    float lastSpeed = 0.0f;
    uint32_t lastCallMs = 0;
    uint32_t lastUs = 0;
};
MoveState mv;

inline float smoothstep01(float x)
{
    x = constrain(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

/// Verlauf f(p) in 0..1 (p = Fortschritt 0..1)
float speedPatternValue(uint8_t pattern, float p)
{
    p = constrain(p, 0.0f, 1.0f);
    float f;
    switch (pattern)
    {
    case SPattern_Linear:
        f = 1.0f - p;
        break;
    case SPattern_LinearUp:
        f = p;
        break;
    case SPattern_Constant:
        f = 1.0f;
        break;
    case SPattern_Trapezoid:
        f = min(min(p / 0.2f, 1.0f), (1.0f - p) / 0.4f);
        break;
    case SPattern_Triangle:
        f = 1.0f - fabsf(1.0f - 2.0f * p);
        break;
    case SPattern_QuadDecel:
        f = (1.0f - p) * (1.0f - p);
        break;
    case SPattern_ConstDecel:
        f = sqrtf(1.0f - p);
        break;
    case SPattern_SmoothStep:
        f = 1.0f - smoothstep01(p);
        break;
    case SPattern_Cosine:
        f = 0.5f * (1.0f + cosf(PI * p));
        break;
    case SPattern_Sine:
        f = sinf(PI * p);
        break;
    case SPattern_SCurve:
        f = min(smoothstep01(p / 0.25f), 1.0f - smoothstep01((p - 0.6f) / 0.4f));
        break;
    case SPattern_LateBrake:
        f = (p < 0.7f) ? 1.0f : 0.5f * (1.0f + cosf(PI * (p - 0.7f) / 0.3f));
        break;
    case SPattern_FastThenCrawl:
        f = (p < 0.5f) ? 1.0f : ((p < 0.7f) ? 1.0f - (p - 0.5f) / 0.2f : 0.0f);
        break;
    case SPattern_QuadUp:
        f = p * p;
        break;
    case SPattern_CubicUp:
        f = p * p * p;
        break;
    case SPattern_Expo:
        f = (expf(-3.0f * p) - expf(-3.0f)) / (1.0f - expf(-3.0f));
        break;
    case SPattern_Stairs:
        f = 1.0f - min(floorf(p * 4.0f), 3.0f) / 3.0f;
        break;
    case SPattern_Wave:
        f = (1.0f - p) * (0.6f + 0.4f * sinf(2.0f * PI * 3.0f * p));
        break;
    case SPattern_Pulse:
        f = (fmodf(p * 5.0f, 1.0f) < 0.6f) ? 1.0f : 0.0f;
        break;
    default:
        f = 1.0f;
        break; // unbekanntes Pattern: konstant
    }
    return constrain(f, 0.0f, 1.0f);
}
}

void systemlib::resetMoveProfile(void)
{
    mv.active = false; // lastSpeed bleibt: rollt der Roboter weiter, steigt der Speed stetig statt von 0
}

bool systemlib::giveMoveArrived(void) const { return mv.active && mv.arrived; }

float systemlib::giveMoveProgress(void) const { return mv.progress; }

/**
 * Faehrt zum Punkt (X, Y) [cm, Feldkoordinaten] und dreht dabei auf Andrehwinkel [Grad].
 *
 * @param SpeedPattern SPattern_* aus params.h (Verlauf ueber den Fahrweg)
 * @param MaxSpeed     Speed [%] bei f = 1
 * @param MinSpeed     Speed [%] bei f = 0 (Mindestfahrt, damit der Roboter nicht stehen bleibt)
 * @param Andrehwinkel Soll-Heading [Grad], wird waehrend der ganzen Fahrt geregelt
 *
 * Zyklisch aufrufen. Position ungueltig (-1) -> Roboter haelt an, Heading wird weiter geregelt.
 * Zielkreis MOVE_ARRIVE_CM: darin Speed 0; wird der Roboter weggeschoben, faehrt er
 * nach MOVE_ARRIVE_HYST_CM mit MinSpeed zurueck. Status: giveMoveArrived().
 * Der Speed darf hoechstens mit MOVE_MAX_ACCEL_PCT_S ansteigen (weicher Anlauf, Bremsen nie begrenzt).
 */
void systemlib::moveToPoint(uint16_t X, uint16_t Y, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, int16_t Andrehwinkel)
{
    // Heading wird immer geregelt, auch wenn der Roboter steht
    setHeading(Andrehwinkel * DEG_TO_RAD);
    const float omega = giveHeadingController() * 100;

    if (MaxSpeed > 100)
        MaxSpeed = 100;
    if (MinSpeed > MaxSpeed)
        MinSpeed = MaxSpeed;

    const uint32_t nowMs = millis();
    const uint32_t nowUs = micros();
    float dt = (mv.lastUs != 0) ? (nowUs - mv.lastUs) * 1e-6f : 0.0f;
    if (dt > 0.1f)
        dt = 0.0f; // lange Aufrufpause: Anlaufbegrenzung beginnt von vorn statt sofort 60 % freizugeben
    mv.lastUs = nowUs;

    // Position verloren: anhalten, Profil verwerfen
    if (Position.X < 0.0f || Position.Y < 0.0f)
    {
        resetMoveProfile();
        mv.lastSpeed = 0.0f;
        mv.lastCallMs = nowMs;
        setOmniWheelDir(0, 0, omega);
        return;
    }

    const float tx = constrain((float)X, 0.0f, FIELD_X_CM);
    const float ty = constrain((float)Y, 0.0f, FIELD_Y_CM);
    const float dx = tx - Position.X;
    const float dy = ty - Position.Y;
    const float dist = sqrtf(dx * dx + dy * dy);

    // ---- Neustart des Patterns? -----------------------------------------
    const bool sameTarget = (fabsf(tx - mv.tx) <= MOVE_TARGET_TOL_CM) && (fabsf(ty - mv.ty) <= MOVE_TARGET_TOL_CM);
    const bool restart = !mv.active || !sameTarget || (SpeedPattern != mv.pattern) || (MaxSpeed != mv.maxS) || (MinSpeed != mv.minS) || ((nowMs - mv.lastCallMs) > (uint32_t)MOVE_RESTART_GAP_MS);
    mv.lastCallMs = nowMs;

    if (restart)
    {
        mv.active = true;
        mv.tx = tx;
        mv.ty = ty;
        mv.pattern = SpeedPattern;
        mv.maxS = MaxSpeed;
        mv.minS = MinSpeed;
        mv.dist0 = dist;
        mv.progress = 0.0f;
        mv.arrived = (dist <= MOVE_ARRIVE_CM);
        mv.correcting = false;
        // lastSpeed bleibt: rollt der Roboter noch, wird der Speed nicht ruckartig umgesetzt
    }

    // ---- Zielkreis mit Hysterese -----------------------------------------
    if (mv.arrived)
    {
        if (dist > MOVE_ARRIVE_CM + MOVE_ARRIVE_HYST_CM) // weggeschoben -> Feinkorrektur
        {
            mv.arrived = false;
            mv.correcting = true;
            mv.dist0 = dist;
            mv.progress = 0.0f;
        }
    }
    else if (dist <= MOVE_ARRIVE_CM)
    {
        mv.arrived = true;
    }

    // ---- Geschwindigkeit ----------------------------------------------------
    float speed = 0.0f;
    if (!mv.arrived)
    {
        mv.progress = (mv.dist0 > MOVE_ARRIVE_CM) ? constrain(1.0f - dist / mv.dist0, 0.0f, 1.0f) : 1.0f;

        const float f = mv.correcting ? 0.0f : speedPatternValue(mv.pattern, mv.progress);
        speed = MinSpeed + (MaxSpeed - MinSpeed) * f;

        // weicher Anlauf: Zunahme begrenzen, Abnahme (Bremsen) nie
        if (MOVE_MAX_ACCEL_PCT_S > 0.0f && speed > mv.lastSpeed + MOVE_MAX_ACCEL_PCT_S * dt)
            speed = mv.lastSpeed + MOVE_MAX_ACCEL_PCT_S * dt;
    }
    else
    {
        mv.progress = 1.0f;
    }
    mv.lastSpeed = speed;

    setOmniWheelDir(Util.wrapAngle(atan2f(-dx, dy) * (180.0f / PI)),
                    (uint8_t)constrain(speed + 0.5f, 0.0f, 100.0f), omega);
}

/* ========================================================================
 * Punkt-Array: Liste von Zielpunkten, die nacheinander angefahren werden
 *
 * Jeder Punkt hat Ziel (X, Y), Andrehwinkel, Speed-Pattern, Max-/MinSpeed und
 * optional eine Wartezeit. Gefahren wird mit moveToPoint(), der Verlauf des
 * Speed-Patterns startet also an jedem Punkt neu.
 *
 *   ArrayPoint route[] = {
 *       // X    Y    Winkel  Pattern             Max  Min  Wartezeit
 *       {  60,  80,    0,    SPattern_Linear,     80,  25,    0 },
 *       { 120, 160,   90,    SPattern_Trapezoid,  70,  25,  500 },
 *   };
 *   Robot.setPointArray(route, 2);   // zyklisch aufrufbar: unveraendert = kein Neustart
 *   Robot.startArray();              // zyklisch aufrufbar: laeuft schon = kein Neustart
 *   Robot.updateArray();             // JEDEN Zyklus: faehrt, schaltet beim Erreichen weiter
 *
 * Ein Punkt gilt als erreicht, wenn der Zielkreis (MOVE_ARRIVE_CM) erreicht ist,
 * der Roboter auf Andrehwinkel +-ARRAY_HEADING_TOL_DEG steht und die Wartezeit
 * abgelaufen ist.
 * ===================================================================== */
namespace
{
struct ArrayData
{
    ArrayPoint pts[ARRAY_MAX_POINTS];
    uint8_t count = 0;
    uint8_t index = 0; // aktueller Zielpunkt
    bool loop = false;
    ArrayState state = ArrayIdle;
    bool holding = false; // Punkt erreicht, Wartezeit laeuft
    uint32_t holdStartMs = 0;
};
ArrayData pa;

bool samePoint(const ArrayPoint &a, const ArrayPoint &b)
{
    return a.X == b.X && a.Y == b.Y && a.Andrehwinkel == b.Andrehwinkel && a.SpeedPattern == b.SpeedPattern && a.MaxSpeed == b.MaxSpeed && a.MinSpeed == b.MinSpeed && a.HoldMs == b.HoldMs && a.PassCm == b.PassCm;
}
}

/**
 * Ersetzt die komplette Liste. Zyklisch aufrufbar: sind Punkte und loop unveraendert,
 * passiert nichts (Fortschritt bleibt). Bei jeder Aenderung beginnt die Liste wieder
 * beim ersten Punkt (ein laufender oder pausierter Zustand bleibt erhalten,
 * ein beendeter/ruhender wird zu Idle -> startArray()).
 *
 * @param loop true = nach dem letzten Punkt wieder beim ersten weitermachen
 * @return false, wenn die Liste laenger als ARRAY_MAX_POINTS ist (dann unveraendert)
 */
bool systemlib::setPointArray(const ArrayPoint *points, uint8_t count, bool loop)
{
    if (count > ARRAY_MAX_POINTS || (points == nullptr && count > 0))
    {
        return false;
    }

    bool same = (count == pa.count) && (loop == pa.loop);
    for (uint8_t i = 0; same && i < count; i++)
    {
        same = samePoint(points[i], pa.pts[i]);
    }
    if (same)
    {
        return true;
    }

    for (uint8_t i = 0; i < count; i++)
    {
        pa.pts[i] = points[i];
    }
    pa.count = count;
    pa.loop = loop;
    pa.index = 0;
    pa.holding = false;
    resetMoveProfile();

    if (count == 0 || pa.state == ArrayFinished)
    {
        pa.state = ArrayIdle;
    }
    return true;
}

/**
 * Haengt einen Punkt hinten an. Eine laufende Fahrt wird dadurch nicht unterbrochen;
 * war die Liste schon beendet, faehrt der Roboter zum neuen Punkt weiter.
 *
 * @param HoldMs Wartezeit nach Erreichen des Punkts [ms], bevor der naechste folgt
 * @return false, wenn die Liste voll ist (ARRAY_MAX_POINTS)
 */
bool systemlib::addPointToArray(uint16_t X, uint16_t Y, int16_t Andrehwinkel, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, uint16_t HoldMs)
{
    ArrayPoint p;
    p.X = X;
    p.Y = Y;
    p.Andrehwinkel = Andrehwinkel;
    p.SpeedPattern = SpeedPattern;
    p.MaxSpeed = MaxSpeed;
    p.MinSpeed = MinSpeed;
    p.HoldMs = HoldMs;
    p.PassCm = 0;
    return addPointToArray(p);
}

bool systemlib::addPointToArray(const ArrayPoint &point)
{
    if (pa.count >= ARRAY_MAX_POINTS)
    {
        return false;
    }

    pa.pts[pa.count] = point;
    pa.count++;

    if (pa.state == ArrayFinished) // Liste war fertig -> zum neuen Punkt weiterfahren
    {
        pa.index = pa.count - 1;
        pa.state = ArrayRunning;
        pa.holding = false;
        resetMoveProfile();
    }
    return true;
}

/** Loescht alle Punkte und setzt den Zustand auf Idle. updateArray() sendet dann keine Motorbefehle mehr. */
void systemlib::resetArray(void)
{
    pa.count = 0;
    pa.index = 0;
    pa.loop = false;
    pa.state = ArrayIdle;
    pa.holding = false;
    resetMoveProfile();
}

/**
 * Startet die Liste beim ersten Punkt. Zyklisch aufrufbar: laeuft sie schon, ist sie
 * pausiert (-> resumeArray()) oder beendet, passiert nichts.
 *
 * @param restart true = in jedem Zustand von vorn beginnen
 */
void systemlib::startArray(bool restart)
{
    if (pa.count == 0)
    {
        return;
    }
    if (pa.state != ArrayIdle && !restart)
    {
        return;
    }

    pa.index = 0;
    pa.holding = false;
    pa.state = ArrayRunning;
    resetMoveProfile();
}

/** Haelt den Roboter an (Heading wird weiter geregelt). Der Fortschritt bleibt erhalten. */
void systemlib::pauseArray(void)
{
    if (pa.state == ArrayRunning)
    {
        pa.state = ArrayPaused;
        pa.holding = false;
        resetMoveProfile();
        mv.lastSpeed = 0.0f; // Roboter steht -> Anlauf nach resumeArray() wieder begrenzt
    }
}

/** Setzt eine pausierte Liste am aktuellen Punkt fort (das Speed-Pattern startet fuer diesen Punkt neu). */
void systemlib::resumeArray(void)
{
    if (pa.state == ArrayPaused)
    {
        pa.state = ArrayRunning;
        pa.holding = false;
        resetMoveProfile();
    }
}

/**
 * Jeden Zyklus aufrufen (wie moveToPoint). Faehrt den aktuellen Punkt an und schaltet
 * weiter, sobald er erreicht ist.
 *
 *  Idle      -> keine Motorbefehle
 *  Running   -> faehrt / wartet am Punkt
 *  Paused    -> steht, haelt Andrehwinkel des aktuellen Punkts
 *  Finished  -> haelt Position und Winkel des letzten Punkts
 */
void systemlib::updateArray(void)
{
    if (pa.count == 0)
    {
        return;
    }

    const ArrayPoint &p = pa.pts[pa.index];

    switch (pa.state)
    {
    case ArrayRunning:
        break;

    case ArrayFinished:
        moveToPoint(p.X, p.Y, p.SpeedPattern, p.MaxSpeed, p.MinSpeed, p.Andrehwinkel);
        return;

    case ArrayPaused:
        setHeading(p.Andrehwinkel * DEG_TO_RAD);
        setOmniWheelDir(0, 0, giveHeadingController() * 100);
        return;

    default: // ArrayIdle
        return;
    }

    moveToPoint(p.X, p.Y, p.SpeedPattern, p.MaxSpeed, p.MinSpeed, p.Andrehwinkel);

    // ---- Durchfahrpunkt: weiterschalten, ohne anzuhalten ------------------------
    // Zielkreis, Winkel und Wartezeit gelten nur fuer Haltepunkte (PassCm = 0) und den letzten Punkt.
    if (p.PassCm > 0 && !(pa.index + 1 >= pa.count && !pa.loop))
    {
        pa.holding = false;
        const float dx = p.X - Position.X;
        const float dy = p.Y - Position.Y;
        if (Position.X >= 0.0f && Position.Y >= 0.0f && sqrtf(dx * dx + dy * dy) <= p.PassCm)
        {
            pa.index = (pa.index + 1 < pa.count) ? (pa.index + 1) : 0;
            resetMoveProfile(); // Pattern fuer den naechsten Punkt von vorn, Speed laeuft stetig weiter
        }
        return;
    }

    // ---- Punkt erreicht? ----------------------------------------------------
    const float headingErr = fabsf(Util.wrapAngle(p.Andrehwinkel - giveOrbitDir()));
    const bool reached = giveMoveArrived() && (ARRAY_HEADING_TOL_DEG <= 0.0f || headingErr <= ARRAY_HEADING_TOL_DEG);

    if (!reached)
    {
        pa.holding = false;
        return;
    }

    const uint32_t nowMs = millis();
    if (!pa.holding)
    {
        pa.holding = true;
        pa.holdStartMs = nowMs;
    }
    if ((uint32_t)(nowMs - pa.holdStartMs) < p.HoldMs)
    {
        return; // Wartezeit laeuft
    }

    // ---- weiterschalten ---------------------------------------------------------
    pa.holding = false;
    if (pa.index + 1 < pa.count)
    {
        pa.index++;
        resetMoveProfile(); // Pattern fuer den neuen Punkt von vorn, auch bei gleichem Ziel
    }
    else if (pa.loop)
    {
        pa.index = 0;
        resetMoveProfile();
    }
    else
    {
        pa.state = ArrayFinished;
    }
}

ArrayState systemlib::giveArrayState(void) const { return pa.state; }
uint8_t systemlib::giveArrayIndex(void) const { return pa.index; }
uint8_t systemlib::giveArrayCount(void) const { return pa.count; }

/* ========================================================================
 * Pfadplanung: moveToPointAvoid()
 *
 * Verknuepft den A*-Pathfinder (lib/Pathfinder) mit dem Punkt-Array:
 *
 *   Position, Ziel, Ball, Partner  ->  Pathfinder::findPath()  (Raster PATH_GRID_X/Y_CELLS)
 *     ->  Zellpfad  ->  Pathfinder::giveWaypoints() (wenige Wegpunkte)  ->  in cm
 *     ->  Wegpunkte als ArrayPoints  ->  setPointArray() / updateArray()
 *
 * Das Speed-Pattern gilt fuer die GANZE Strecke: Zwischenpunkte bekommen je
 * ein konstantes Tempo, das aus dem Pattern an der Stelle des Teilstuecks
 * berechnet wird; sie werden ohne Halt durchfahren (PassCm). Das letzte
 * Teilstueck faehrt das Pattern selbst, inkl. Zielkreis und Andrehwinkel.
 *
 * Neu geplant wird bei: neuem Ziel / Pattern / Speed, bewegtem Hindernis
 * (> PATH_OBST_MOVE_CM) und zyklisch alle PATH_REPLAN_MS (nur wenn noch unterwegs).
 *
 * ACHTUNG: Waehrend moveToPointAvoid() benutzt wird, gehoert ihm das Punkt-Array.
 * ===================================================================== */
namespace
{
struct PathPlan
{
    bool valid = false; // es gibt eine Planung fuer das aktuelle Ziel
    bool found = false; // A* hat einen Weg gefunden
    uint16_t tx = 0, ty = 0;
    uint8_t pattern = 0, maxS = 0, minS = 0;
    int16_t ballIn[2] = {-1, -1}; // aktuelle Eingabe [cm], -1 = kein Hindernis
    int16_t robIn[2] = {-1, -1};
    int16_t ballUsed[2] = {-1, -1}; // beim letzten Planen verwendet
    int16_t robUsed[2] = {-1, -1};
    uint32_t lastPlanMs = 0;
    uint32_t planUs = 0;
    uint8_t wpCount = 0;
    float total0 = 1.0f; // Streckenlaenge [cm], auf die sich der Pattern-Fortschritt bezieht
};
PathPlan plan;

uint8_t cmToCell(float cm, float fieldCm, uint8_t cells)
{
    const int c = (int)(cm / fieldCm * cells);
    return (uint8_t)constrain(c, 0, (int)cells - 1);
}

float cellToCm(uint8_t cell, float fieldCm, uint8_t cells)
{
    return (cell + 0.5f) * fieldCm / cells;
}

/// true, wenn sich ein Hindernis gegenueber der letzten Planung bewegt hat (oder neu/weg ist)
bool obstacleMoved(const int16_t (&now)[2], const int16_t (&used)[2])
{
    const bool a = (now[0] >= 0 && now[1] >= 0);
    const bool b = (used[0] >= 0 && used[1] >= 0);
    if (a != b)
        return true;
    if (!a)
        return false;
    const float dx = now[0] - used[0];
    const float dy = now[1] - used[1];
    return sqrtf(dx * dx + dy * dy) > PATH_OBST_MOVE_CM;
}

/**
     * A* + Vereinfachung + Tempo je Teilstueck -> ArrayPoints.
     * Versuche: 1) mit Ball und Partner, 2) ohne Ball (Ziel liegt oft im Ballbereich),
     * 3) ohne Hindernisse (Startzelle im Sperrbereich). Jeder Fehlversuch kostet Rechenzeit.
     */
bool buildRoute(float sx, float sy, float fieldX, float fieldY, uint16_t tx, uint16_t ty,
                uint8_t pattern, uint8_t maxS, uint8_t minS, int16_t angle, bool restartPattern,
                ArrayPoint (&route)[ARRAY_MAX_POINTS], uint8_t &count)
{
    const uint8_t W = PATH_GRID_X_CELLS;
    const uint8_t H = PATH_GRID_Y_CELLS;

    const uint8_t scx = cmToCell(sx, fieldX, W), scy = cmToCell(sy, fieldY, H);
    const uint8_t tcx = cmToCell(tx, fieldX, W), tcy = cmToCell(ty, fieldY, H);

    const bool hasBall = (plan.ballIn[0] >= 0 && plan.ballIn[1] >= 0);
    const bool hasRob = (plan.robIn[0] >= 0 && plan.robIn[1] >= 0);
    const uint8_t bx = hasBall ? cmToCell(plan.ballIn[0], fieldX, W) : PATH_NO_OBSTACLE;
    const uint8_t by = hasBall ? cmToCell(plan.ballIn[1], fieldY, H) : PATH_NO_OBSTACLE;
    const uint8_t rx = hasRob ? cmToCell(plan.robIn[0], fieldX, W) : PATH_NO_OBSTACLE;
    const uint8_t ry = hasRob ? cmToCell(plan.robIn[1], fieldY, H) : PATH_NO_OBSTACLE;

    bool ok = false;
    for (uint8_t attempt = 0; attempt < 3 && !ok; attempt++)
    {
        if (attempt == 1 && !hasBall)
            continue;
        if (attempt == 2 && !hasRob)
            continue;

        const bool useBall = (attempt == 0);
        const bool useRob = (attempt <= 1);
        ok = AStar.findPath(scx, scy, tcx, tcy,
                            useBall ? bx : PATH_NO_OBSTACLE, useBall ? by : PATH_NO_OBSTACLE,
                            useRob ? rx : PATH_NO_OBSTACLE, useRob ? ry : PATH_NO_OBSTACLE);
    }
    if (!ok)
    {
        return false;
    }

    // ---- Wegpunkte (cm) -------------------------------------------------
    uint8_t wp[ARRAY_MAX_POINTS][2];
    uint8_t n = (AStar.givePathLength() >= 2) ? AStar.giveWaypoints(wp, ARRAY_MAX_POINTS) : 0;

    float px[ARRAY_MAX_POINTS + 1];
    float py[ARRAY_MAX_POINTS + 1];
    px[0] = sx;
    py[0] = sy;
    for (uint8_t i = 0; i < n; i++)
    {
        px[i + 1] = cellToCm(wp[i][0], fieldX, W);
        py[i + 1] = cellToCm(wp[i][1], fieldY, H);
    }
    if (n == 0)
        n = 1;  // Start und Ziel in derselben Zelle: nur das Ziel
    px[n] = tx; // letzter Punkt = exaktes Ziel, nicht Zellmitte
    py[n] = ty;

    // ---- Tempo: Pattern ueber die ganze Strecke --------------------------------
    float total = 0.0f;
    float seg[ARRAY_MAX_POINTS];
    for (uint8_t i = 0; i < n; i++)
    {
        const float dx = px[i + 1] - px[i];
        const float dy = py[i + 1] - py[i];
        seg[i] = sqrtf(dx * dx + dy * dy);
        total += seg[i];
    }
    if (total < 1.0f)
        total = 1.0f;

    // Der Pattern-Fortschritt bezieht sich auf die Strecke bei Fahrtbeginn (neues Ziel / Pattern / Speed).
    // Periodische Neuplanungen aendern das nicht, sonst wuerde das Tempo bei jeder Neuplanung neu anlaufen.
    if (restartPattern || total > plan.total0)
        plan.total0 = total;

    float cum = 0.0f;
    for (uint8_t i = 0; i < n; i++)
    {
        const float pStart = constrain(1.0f - (total - cum) / plan.total0, 0.0f, 1.0f);
        const float pMid = constrain(1.0f - (total - cum - 0.5f * seg[i]) / plan.total0, 0.0f, 1.0f);
        cum += seg[i];

        ArrayPoint &a = route[i];
        a.X = (uint16_t)constrain(px[i + 1] + 0.5f, 0.0f, fieldX);
        a.Y = (uint16_t)constrain(py[i + 1] + 0.5f, 0.0f, fieldY);
        a.Andrehwinkel = angle;
        a.HoldMs = 0;

        if (i + 1 < n) // Zwischenpunkt: konstantes Tempo, ohne Halt durchfahren
        {
            const float v = minS + (maxS - minS) * speedPatternValue(pattern, pMid);
            a.SpeedPattern = SPattern_Constant;
            a.MaxSpeed = (uint8_t)(v + 0.5f);
            a.MinSpeed = a.MaxSpeed;
            a.PassCm = PATH_PASS_CM;
        }
        else // letztes Teilstueck: Pattern bis zum Ziel
        {
            const float v0 = minS + (maxS - minS) * speedPatternValue(pattern, pStart);
            a.SpeedPattern = pattern;
            a.MaxSpeed = (uint8_t)(v0 + 0.5f);
            a.MinSpeed = minS;
            a.PassCm = 0;
        }
    }
    count = n;
    return true;
}
}

/**
 * Hindernisse fuer die Pfadplanung [cm, Feldkoordinaten]. -1 = kein Hindernis.
 * Zyklisch aufrufbar, vor moveToPointAvoid(). Ball = Ball, Roboter = Partner (oder Gegner).
 */
void systemlib::setPathObstacles(int16_t ballX, int16_t ballY, int16_t robotX, int16_t robotY)
{
    plan.ballIn[0] = ballX;
    plan.ballIn[1] = ballY;
    plan.robIn[0] = robotX;
    plan.robIn[1] = robotY;
}

/**
 * Wie moveToPoint(), aber um Hindernisse herum. Zyklisch aufrufen.
 * Alle Parameter wie bei moveToPoint(); das Speed-Pattern gilt fuer die ganze Strecke.
 *
 * Kein Weg gefunden: PATH_FALLBACK_DIRECT = true faehrt geradeaus per moveToPoint(),
 * sonst bleibt der Roboter stehen und haelt den Winkel. Status: givePathFound().
 */
void systemlib::moveToPointAvoid(uint16_t X, uint16_t Y, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, int16_t Andrehwinkel)
{
    // Position unbekannt: moveToPoint() haelt an und regelt den Winkel
    if (Position.X < 0.0f || Position.Y < 0.0f)
    {
        moveToPoint(X, Y, SpeedPattern, MaxSpeed, MinSpeed, Andrehwinkel);
        return;
    }

    const uint32_t nowMs = millis();

    const bool settingsChanged = !plan.valid || X != plan.tx || Y != plan.ty || SpeedPattern != plan.pattern || MaxSpeed != plan.maxS || MinSpeed != plan.minS;
    const bool moved = obstacleMoved(plan.ballIn, plan.ballUsed) || obstacleMoved(plan.robIn, plan.robUsed);
    const bool settled = plan.found && giveMoveArrived(); // am Ziel: nicht dauernd neu planen
    const uint32_t interval = plan.found ? PATH_REPLAN_MS : PATH_REPLAN_FAIL_MS;
    const bool due = !settled && (uint32_t)(nowMs - plan.lastPlanMs) >= interval;

    if (settingsChanged || moved || due)
    {
        plan.lastPlanMs = nowMs;
        plan.valid = true;
        plan.tx = X;
        plan.ty = Y;
        plan.pattern = SpeedPattern;
        plan.maxS = MaxSpeed;
        plan.minS = MinSpeed;
        for (uint8_t i = 0; i < 2; i++)
        {
            plan.ballUsed[i] = plan.ballIn[i];
            plan.robUsed[i] = plan.robIn[i];
        }

        ArrayPoint route[ARRAY_MAX_POINTS];
        uint8_t count = 0;

        const uint32_t t0 = micros();
        plan.found = buildRoute(Position.X, Position.Y, FIELD_X_CM, FIELD_Y_CM, X, Y,
                                SpeedPattern, MaxSpeed, MinSpeed, Andrehwinkel, settingsChanged, route, count);
        plan.planUs = micros() - t0;

        if (plan.found)
        {
            plan.wpCount = count;
            setPointArray(&route[0], count); // unveraenderte Wegpunkte: kein Neustart
        }
        else
        {
            plan.wpCount = 0;
            resetArray();
        }
    }

    if (plan.found)
    {
        // Winkel darf sich waehrend der Fahrt aendern, ohne dass neu geplant wird
        for (uint8_t i = 0; i < pa.count; i++)
        {
            pa.pts[i].Andrehwinkel = Andrehwinkel;
        }
        startArray();
        updateArray();
    }
    else
    {
        if (PATH_FALLBACK_DIRECT)
        {
            moveToPoint(X, Y, SpeedPattern, MaxSpeed, MinSpeed, Andrehwinkel);
        }
        else
        {
            setHeading(Andrehwinkel * DEG_TO_RAD);
            setOmniWheelDir(0, 0, giveHeadingController() * 100);
        }
    }
}

/**
 * Verwirft Planung, Punkt-Array und Fahrprofil (Speed 0). Aufrufen beim Wechsel in einen anderen
 * Modus, damit die naechste Fahrt ohne Reste der vorigen beginnt.
 */
void systemlib::resetPathPlan(void)
{
    plan.valid = false;
    plan.found = false;
    plan.wpCount = 0;
    plan.total0 = 1.0f;
    resetArray();
    mv.lastSpeed = 0.0f;
}

bool systemlib::givePathFound(void) const { return plan.found; }
uint8_t systemlib::givePathWaypoints(void) const { return plan.wpCount; }
uint32_t systemlib::givePathTimeUs(void) const { return plan.planUs; }

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

void systemlib::setOmniWheelVek(float vx, float vy, float omega)
{
    float w[4];
    const WheelTrig &wt = wheelTrig();
    for (int i = 0; i < 4; ++i)
    {
        w[i] = -wt.s[i] * -vx + wt.c[i] * vy + omega;
        w[i] = constrain(w[i], -100, 100);
    }
    setMotorData(MoDataVL.ID, w[0], true, false);
    setMotorData(MoDataVR.ID, w[1], true, false);
    setMotorData(MoDataHL.ID, w[2], true, false);
    setMotorData(MoDataHR.ID, w[3], true, false);
}

void systemlib::setOmniWheelDir(float directionDeg, uint8_t speedPercent, float omega)
{
    directionDeg = directionDeg - giveOrbitDir();

    directionDeg = Util.wrapAngle(directionDeg);

    float w[4];
    const WheelTrig &wt = wheelTrig();
    // Serial.println("omega : "+String(omega));  // deaktiviert: blockierendes USB-Serial im 100-Hz-Takt verfaelscht dt

    float directionRad = directionDeg * DEG_TO_RAD;
    float speedNorm = speedPercent / 100.0f; // 0..1

    float vx = cosf(directionRad) * speedNorm * 100;
    float vy = sinf(directionRad) * speedNorm * 100;

    for (int i = 0; i < 4; ++i)
    {
        w[i] = -wt.s[i] * -vx + wt.c[i] * vy + omega;
        w[i] = constrain(w[i], -100, 100);
    }

    setMotorData(MoDataVL.ID, w[0], true, false);
    setMotorData(MoDataVR.ID, w[1], true, false);
    setMotorData(MoDataHL.ID, w[2], true, false);
    setMotorData(MoDataHR.ID, w[3], true, false);
}

static int checkWallDistance(float posX, float posY)
{
    // Ungültige Positionen (z. B. -1 aus der Fallback-Logik) direkt abfangen
    if (posX < 0.0f || posY < 0.0f)
    {
        return -1;
    }

    // Prüfen, ob eine der 4 Wände unterschritten wird
    if (posX < MOVE_WALL_MARGIN_CM || posX > (FIELD_X_CM - MOVE_WALL_MARGIN_CM) ||
        posY < MOVE_WALL_MARGIN_CM || posY > (FIELD_Y_CM - MOVE_WALL_MARGIN_CM))
    {
        return 1; // Naeher als MOVE_WALL_MARGIN_CM an der Wand
    }

    return 0; // Sicherer Bereich
}

void systemlib::moveDir(int16_t Dir, uint8_t Speed, float Andrehwinkel)
{
    setHeading(Andrehwinkel * DEG_TO_RAD);
    if (checkWallDistance(Position.X, Position.Y) == 1)
    {
        setOmniWheelDir(Dir, MOVE_WALL_SPEED_PCT, giveHeadingController() * 100);
    }
    else
    {
        setOmniWheelDir(Dir, Speed, giveHeadingController() * 100);
    }
}
