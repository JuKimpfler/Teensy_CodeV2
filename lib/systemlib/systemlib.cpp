#include <systemlib.h>

/* -------------- Instanzen -------------- */

BNO055 BNO(BNO_ADDRESS, &I2C_BNO);
// PowerDebugger PDS;

elapsedMillis cycleTimer;
elapsedMillis powerTimer;   // drosselt die INA219-Abfrage fuer die PDS-Kanaele
elapsedMicros cycleTimer_us;
elapsedMillis lastvaliddata_Radar_X;
elapsedMillis lastvaliddata_Radar_Y;

commlib communication;

I2C_bus i2cIR(I2C_IR);
Device_handle ir_ring_handle = {i2cIR, I2C_target{0x0A}};
IR_ring ir_ring(ir_ring_handle);

INA219 INA219_Main(0x40);
INA219 INA219_Motor(0x41);

TCA9534DW SW_EXP(&I2C_SW, 0x22);
TCA9534DW Motor_EXP(&Wire1, 0x20);
TCA9534DW Button_EXP(&Wire1, 0x23);
TCA9534DW Rand_EXP(&Wire1, 0x21);

ADC108S102 ADC_Motor(&SPI1, 32);
ADC108S102 ADC_Line1(&SPI,25,22,0,0);
ADC108S102 ADC_Line2(&SPI,25,22,0,1);
ADC108S102 ADC_Line3(&SPI,25,22,0,2);
ADC108S102 ADC_Line4(&SPI,25,22,0,3);
ADC108S102 ADC_Line5(&SPI,25,22,0,4);
ADC108S102 ADC_Line6(&SPI,25,22,0,5);

constexpr float TS_NOMINAL = 0.005f; // nur Fallback fuer cfg.Ts, siehe loop()
PID *headingControllerP;
ADRC *headingControllerA;
uint32_t lastMicrosContrl = 0;

fancyBTN BTN1(50, 6, 8, true);
fancyBTN BTN2(50, 6, 8, true);
fancyBTN BTN3(50, 6, 8, true);
fancyBTN BTN4(50, 6, 8, true);
fancyBTN BTN5(50, 6, 8, true);

/* -------------- System Worker -------------- */

void systemlib::initialize()
{
    Wire1.begin();
    Wire1.setClock(100000);
    Wire.begin();
    Wire.setClock(100000);

    Rand_EXP.setup(false);
    Motor_EXP.setup(false);
    SW_EXP.setup(true);
    Button_EXP.setup(true);
    readExpanders();
    Motor_EXP.writePin(7,false);

    SPI.begin();
    SPI1.begin();

    USBC_UART.begin(115200);

    INA219_Main.begin(Wire1);
    INA219_Motor.begin(Wire1);

    INA219_Main.setMaxCurrentShunt(15);
    INA219_Motor.setMaxCurrentShunt(15);

    BNO.init();
    calibBNO();

    #if DPDS_Game == D_Mode_Test
    PDS.init();
    setupPDS();
    #endif

    MoDataVL.ID = MotorVL_ID;
    MoDataVR.ID = MotorVR_ID;
    MoDataHL.ID = MotorHL_ID;
    MoDataHR.ID = MotorHR_ID;
    MoDataDR.ID = MotorDR_ID;

    pinMode(START_SWITCH, INPUT);

    Motor_EXP.setup(false);
    Rand_EXP.setup(false);
    SW_EXP.setup(true);
    Button_EXP.setup(true);

    Radar.begin(I2C_RADAR); // richtet alle Sensoren ein und startet die erste Messrunde

    //ir_ring.read_gains(IR_Gains);
    //ir_ring.read_offsets(IR_Offsets);

    setDebugTimeout(0);

    ADC_Motor.init();
    ADC_Line1.init();
    ADC_Line2.init();
    ADC_Line3.init();
    ADC_Line4.init();
    ADC_Line5.init();
    ADC_Line6.init();

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
    float motorRpm[4] = { MoDataVL.Speed_real, MoDataVR.Speed_real, MoDataHL.Speed_real, MoDataHR.Speed_real };
    static float lastHeadingOdo = giveOrbitDir();
    const float hNow = giveOrbitDir();
    const float hMid = Util.AMap(lastHeadingOdo + 0.5f * Util.AMap(hNow - lastHeadingOdo));
    lastHeadingOdo = hNow;
    RobotVelocity vel = calculateFieldVelocityFromMotorRpm(motorRpm, hMid);
    calculatePosition(millis(), vel.vx_field, vel.vy_field);
    // readIR();
    updateLine();

    readExpanders();

    // Werte fuer die PDS-Kanaele nachfuehren. Die INA219 haengen am I2C und
    // kosten je Abfrage spuerbar Zeit -- Akkuspannung aendert sich nicht im
    // 100-Hz-Takt, 10 Hz reichen voellig.
    uptime_s = millis() / 1000.0f;
    if (powerTimer >= 100)
    {
        powerTimer = 0;
        Akku_V = giveMainVoltage();
        Akku_I = giveMainCurrent();
        Motor_V = giveMotorVoltage();
        Motor_I = giveMotorCurrent();
    }

    #if DPDS_Game == D_Mode_Test
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
    PDS.bind("Timer",         &uptime_s,     0,  "s");
    PDS.bind("Akku_V",        &Akku_V,       1,  "V");
    PDS.bind("Akku_I",        &Akku_I,       2,  "mA");
    PDS.bind("Motor_V",       &Motor_V,      3,  "V");
    PDS.bind("Motor_I",       &Motor_I,      4,  "mA");

    /* ---- Lage ---------------------------------------------------------- */
    PDS.bind("Heading",       &BNO_eulerZ,       5, "°");
    PDS.bind("Orbit_Dir",     &BNO_OrbitDir,     6, "°");
    PDS.bind("BNO_Nullpunkt", &BNO_calibration,  7, "°");

    /* ---- Ball (IR) ----------------------------------------------------- */
    // readIR() ist in readSensors() derzeit auskommentiert -- die Kanaele
    // stehen aber schon bereit und zeigen bis dahin 0.
    PDS.bind("Ball_Winkel",   &IR_Angle,     8,  "°");
    PDS.bind("Ball_Dist",     &IR_Distance,  11);

    /* ---- Zykluszeit ---------------------------------------------------- */
    PDS.bind("Zykluszeit",    &cycleTime,    9,  "ms");
    PDS.bind("Zykluszeit_us", &cycleTime_us, 10, "us");

    /* ---- Radar --------------------------------------------------------- */
    // Dist ist ein Rohwert ("Counts"), NICHT Millimeter -- siehe RadarArray.h.
    PDS.bind("Radar_Vo",      &Radar[RadarVo].Strenght, 12, "Counts");
    PDS.bind("Radar_Li",      &Radar[RadarLi].Strenght, 13, "Counts");
    PDS.bind("Radar_Hi",      &Radar[RadarHi].Strenght, 14, "Counts");
    PDS.bind("Radar_Re",      &Radar[RadarRe].Strenght, 15, "Counts");

    /* ---- Feldposition -------------------------------------------------- */
    // Kanal 16/17 sind die Koordinaten, auf die das "bodies"-Overlay der GUI
    // zeigt (Feld-Draufsicht in channel_config.h).
    PDS.bind("Pos_X",         &Position.X,         16, "cm");
    PDS.bind("Pos_Y",         &Position.Y,         17, "cm");
    PDS.bind("Pos_certanty_x",     &Position.XValid, 18);
    PDS.bind("Pos_certanty_y",     &Position.YValid, 19);

    /* ---- Motoren ------------------------------------------------------- */
    PDS.bind("Motor_VL",      &MoDataVL.Speed_real, 20, "%");
    PDS.bind("Motor_VR",      &MoDataVR.Speed_real, 21, "%");
    PDS.bind("Motor_HL",      &MoDataHL.Speed_real, 22, "%");
    PDS.bind("Motor_HR",      &MoDataHR.Speed_real, 23, "%");

    PDS.bind("Motor_VL",      &MoDataVL.Dir_real, 24, "%");
    PDS.bind("Motor_VR",      &MoDataVR.Dir_real, 25, "%");
    PDS.bind("Motor_HL",      &MoDataHL.Dir_real, 26, "%");
    PDS.bind("Motor_HR",      &MoDataHR.Dir_real, 27, "%");

    /* ---- Schalter ------------------------------------------------------ */
    PDS.bind("Start",         &startSwitch,    28);

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
 *   Abstand zur Feldmitte + POS_GOAL_EDGE_CM < Torbreite/2  -> sicher Tor
 *   Abstand zur Feldmitte - POS_GOAL_EDGE_CM > Torbreite/2  -> sicher Wand
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
    if (POS_GOAL_DEPTH_CM == 0.0f || POS_GOAL_WIDTH_CM <= 0.0f)
        return 0.0f;

    const float halfGoal = POS_GOAL_WIDTH_CM / 2.0f;
    const float dx       = fabsf(x_cm - (feld_x / 2.0f)); // Abstand zur Feldmitte

    if (dx <= halfGoal - POS_GOAL_EDGE_CM) // klar vor der Toroeffnung
        return POS_GOAL_DEPTH_CM;

    if (dx >= halfGoal + POS_GOAL_EDGE_CM) // klar vor der Feldwand
        return 0.0f;

    // Uebergangsband: t = 1 an der Innenkante (ganz Tor) ... 0 an der Aussenkante
    // (ganz Wand). Genau auf der Torkante (t = 0.5) ist die Entscheidung ein
    // Muenzwurf -> sureness = 0.
    const float t = constrain((halfGoal + POS_GOAL_EDGE_CM - dx) / (2.0f * POS_GOAL_EDGE_CM), 0.0f, 1.0f);
    sureness = fabsf(2.0f * t - 1.0f);

    return POS_GOAL_DEPTH_CM * t;
}

/**
 * Rechnet Motordrehzahl + Richtungs-Bool in Radgeschwindigkeit (cm/s) um.
 */
float systemlib::convertMotorRpmToWheelSpeedCmS(float motor_rpm)
{
    float wheel_rpm = motor_rpm / GEAR_RATIO;
    float v_cms = (wheel_rpm * 2.0f * M_PI * WHEEL_RADIUS_CM) / 60.0f;
    return v_cms;
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

    float vy_rob =  0.5f * s * (v_VL - v_VR + v_HL - v_HR);
    float vx_rob = -0.5f * s * (v_VL - v_VR - v_HL + v_HR);

    vx_rob *= ODO_SCALE_X;
    vy_rob *= ODO_SCALE_Y;

    const float heading_rad = heading_deg * (M_PI / 180.0f);
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
    float minDb = -11000.0f - (distM * 2500.0f);
    return (int32_t)max(minDb, -17000.0f);
}

/**
 * Eine Achse aus dem gegenüberliegenden Radar-Paar bestimmen.
 */
float systemlib::calcAxisPosition(uint8_t posLow, uint8_t posHigh, float feldSize, float offsetLow, float offsetHigh, float lastPos, float &certainty)
{
    const RadarData &low  = Radar[posLow];
    const RadarData &high = Radar[posHigh];

    const float span = feldSize - offsetLow - offsetHigh;

    const float distLow  = (low.Dist  / 10.0f) + POS_RADAR_OFFSET_CM;
    const float distHigh = (high.Dist / 10.0f) + POS_RADAR_OFFSET_CM;

    const float minDist = (RADAR_START_MM / 10.0f) + POS_RADAR_OFFSET_CM;
    const float blindDistThreshold = max(POS_BLIND_CM, minDist + 2.0f);

    // 1. Grundprüfung je Sensor
    bool lowOk  = low.isCon  && (low.Peaks  > 0) 
                  && (low.Strenght >= getMinStrengthMdb(distLow))
                  && (distLow  >= minDist) 
                  && (distLow  <= span + POS_MAX_OVERSHOOT_CM);

    bool highOk = high.isCon && (high.Peaks > 0) 
                  && (high.Strenght >= getMinStrengthMdb(distHigh))
                  && (distHigh >= minDist) 
                  && (distHigh <= span + POS_MAX_OVERSHOOT_CM);

    // 2. Blindbereich des Gegenübers
    const bool lowFar  = lowOk  && (distLow  > span - blindDistThreshold);
    const bool highFar = highOk && (distHigh > span - blindDistThreshold);

    if (lowFar && highFar)
    {
        lowOk  = false;
        highOk = false;
    }
    else if (lowFar)
        highOk = false;
    else if (highFar)
        lowOk = false;

    // 3. Position und Plausibilität
    float pos;
    float plaus = 0.0f;
    uint32_t age = 0;
    int32_t usedStrength = -20000;

    if (lowOk && highOk)
    {
        const float err = fabsf((distLow + distHigh) - span);

        if (err <= POS_PAIR_TOL_CM)
        {
            pos   = (distLow + (span - distHigh)) / 2.0f;
            plaus = POS_PLAUS_PAIR - (err / POS_PAIR_TOL_CM) * (POS_PLAUS_PAIR - POS_PLAUS_SINGLE);
            age   = max(Radar.giveAge(posLow), Radar.giveAge(posHigh));
            usedStrength = min(low.Strenght, high.Strenght);
        }
        else
        {
            int32_t diffMdb = low.Strenght - high.Strenght;
            bool useLow;

            if (diffMdb > 3000 || diffMdb < -3000)
            {
                useLow = (diffMdb > 0);
            }
            else
            {
                const bool sumTooSmall = ((distLow + distHigh) < span);
                useLow = sumTooSmall ? (distLow >= distHigh) : (distLow <= distHigh);
            }

            pos   = useLow ? distLow : (span - distHigh);
            plaus = POS_PLAUS_CONFLICT;
            age   = Radar.giveAge(useLow ? posLow : posHigh);
            usedStrength = useLow ? low.Strenght : high.Strenght;
        }
    }
    else if (lowOk)
    {
        pos   = distLow;
        plaus = POS_PLAUS_SINGLE;
        age   = Radar.giveAge(posLow);
        usedStrength = low.Strenght;
    }
    else if (highOk)
    {
        pos   = span - distHigh;
        plaus = POS_PLAUS_SINGLE;
        age   = Radar.giveAge(posHigh);
        usedStrength = high.Strenght;
    }
    else
    {
        certainty = 0.0f;
        return lastPos;
    }

    // 4. Alter und Signalstärke-Gewichtung
    float strengthFactor = constrain(1.0f + (usedStrength + 10000.0f) / 15000.0f, 0.7f, 1.0f);
    certainty = constrain(plaus * posAgeFactor(age) * strengthFactor, 0.0f, 1.0f);

    float calculatedPos = offsetLow + constrain(pos, 0.0f, span);

    // 5. Sprungbegrenzung
    const float MAX_JUMP_CM = 20.0f;
    if (lastPos >= 0.0f && certainty > POS_MIN_THRESHOLD && fabsf(calculatedPos - lastPos) > MAX_JUMP_CM)
    {
        certainty *= 0.2f;
        calculatedPos = lastPos;
    }

    return calculatedPos;
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
        float    z;       // gemessene Position auf der Achse (cm)
        float    cert;    // 0..1
        int8_t   suspect; // -1: Sensor an der niedrigen Wand, +1: an der hohen, 0: Paar stimmt ueberein
        uint32_t t;       // Zeitstempel des Radarwerts (millis)
    };

    struct AxisMeasSet
    {
        AxisMeas m[2];
        uint8_t  n;
    };

    struct AxisFilter
    {
        float    x = 0.0f;
        float    P = FUS_P_INIT;
        float    innov = 0.0f;
        bool     accepted = false;
        float    cand[3]      = {0, 0, 0};
        uint32_t candSince[3] = {0, 0, 0};
        bool     candActive[3] = {false, false, false};
        // Positionshistorie (Ringpuffer) fuer die Radar-Latenzkompensation
        uint32_t hT[FUS_HIST_N] = {0};
        float    hX[FUS_HIST_N] = {0};
        uint8_t  hHead  = 0;
        uint8_t  hCount = 0;
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
            if (f.hT[last] == t) { f.hX[last] = x; return; } // zwei Zyklen in derselben ms
        }
        f.hT[f.hHead] = t;
        f.hX[f.hHead] = x;
        f.hHead = (f.hHead + 1) % FUS_HIST_N;
        if (f.hCount < FUS_HIST_N) f.hCount++;
    }

    /// Position zum Zeitpunkt t (linear interpoliert). false, wenn t aelter als die Historie ist.
    bool histLookup(const AxisFilter &f, uint32_t t, float &x)
    {
        for (uint8_t k = 0; k < f.hCount; k++)
        {
            const uint8_t idx = (f.hHead + FUS_HIST_N - 1 - k) % FUS_HIST_N;
            if (f.hT[idx] <= t)
            {
                if (k == 0) { x = f.hX[idx]; return true; } // t ist neuer als alles -> neuester Wert
                const uint8_t nxt  = (idx + 1) % FUS_HIST_N;
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
            if (f.hT[idx] >= tFrom) f.hX[idx] += d;
        }
    }

    /// Geschwindigkeit um die Rueckmelde-Verzoegerung nach vorne schieben.
    /// Extrapoliert nie ueber Null hinaus (kein Rueckwaertsruckeln beim Bremsen).
    float leadVelocity(float v, float a, float lead)
    {
        if (v == 0.0f) return 0.0f;
        const float vt = v + a * lead;
        return (vt * v < 0.0f) ? 0.0f : vt;
    }
}

/**
 * Radar-Validierung (unveraendert aus calcAxisPosition uebernommen), liefert
 * aber KEINE fertige Position mehr, sondern 0..2 Messungen. Ueber Sprung,
 * Hinderniss und Vertrauen entscheidet der Filter, der die Vorhersage kennt.
 */
static AxisMeasSet radarAxisMeasurements(const RadarData &low, const RadarData &high, uint32_t ageLow, uint32_t ageHigh, float feldSize, float offLow, float offHigh)
{
    AxisMeasSet out;
    out.n = 0;

    const float span     = feldSize - offLow - offHigh;
    const float distLow  = (low.DistRaw  / 10.0f) + POS_RADAR_OFFSET_CM; // ungefilterter Wert
    const float distHigh = (high.DistRaw / 10.0f) + POS_RADAR_OFFSET_CM;

    const float minDist = (RADAR_START_MM / 10.0f) + POS_RADAR_OFFSET_CM;
    const float blindDistThreshold = max(POS_BLIND_CM, minDist + 2.0f);

    bool lowOk  = low.isCon  && (low.Peaks  > 0)
                  && (low.Strenght >= getMinStrengthMdb(distLow))
                  && (distLow  >= minDist)
                  && (distLow  <= span + POS_MAX_OVERSHOOT_CM);

    bool highOk = high.isCon && (high.Peaks > 0)
                  && (high.Strenght >= getMinStrengthMdb(distHigh))
                  && (distHigh >= minDist)
                  && (distHigh <= span + POS_MAX_OVERSHOOT_CM);

    // Blindbereich des Gegenuebers
    const bool lowFar  = lowOk  && (distLow  > span - blindDistThreshold);
    const bool highFar = highOk && (distHigh > span - blindDistThreshold);

    if (lowFar && highFar) { lowOk = false; highOk = false; }
    else if (lowFar)       { highOk = false; }
    else if (highFar)      { lowOk = false; }

    auto certOf = [](float plaus, uint32_t age, int32_t strength) -> float
    {
        const float sf = constrain(1.0f + (strength + 10000.0f) / 15000.0f, 0.7f, 1.0f);
        return constrain(plaus * posAgeFactor(age) * sf, 0.0f, 1.0f);
    };

    const float zLow  = offLow + constrain(distLow,        0.0f, span);
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
            out.m[0] = makeMeas(zLow,  certOf(POS_PLAUS_CONFLICT, ageLow,  low.Strenght),  -1, low.Time);
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
    const float q = FUS_Q_BASE
                    + (FUS_Q_SPEED * speed) * (FUS_Q_SPEED * speed)
                    + (FUS_Q_ROT * omega) * (FUS_Q_ROT * omega);
    f.P += q * dt;
}

/** @return true, wenn die Messung den Zustand veraendert hat */
static bool fusionUpdate(AxisFilter &f, const AxisMeas &m, float tiltFactor, uint32_t now_ms)
{
    const float cert  = max(m.cert, FUS_MIN_CERT);
    const float sigma = FUS_SIGMA_RADAR_CM / cert;
    const float R     = sigma * sigma * tiltFactor;

    // Der Radarwert ist alt: er gehoert zur Position von vor ~Latenz, nicht zu "jetzt".
    const uint32_t tMeas = (m.t > FUS_RADAR_LATENCY_MS) ? (m.t - FUS_RADAR_LATENCY_MS) : 0;
    float xRef = f.x;
    if (!histLookup(f, tMeas, xRef)) xRef = f.x;

    const float y = m.z - xRef;
    const float S = f.P + R;

    f.innov    = y;
    f.accepted = false;

    if (y * y <= FUS_GATE_SIGMA * FUS_GATE_SIGMA * S)
    {
        const float K = f.P / S;
        const float d = K * y;
        f.x += d;
        f.P *= (1.0f - K);
        histShift(f, tMeas, d);
        for (uint8_t i = 0; i < 3; i++) f.candActive[i] = false;
        f.accepted = true;
        return true;
    }

    // Ausreisser: Hindernis / Sprung. Merken, ob er stabil bleibt.
    const bool     obstacleLike = (m.suspect != 0) && (y * (float)m.suspect > 0.0f);
    const uint32_t need         = obstacleLike ? FUS_REACQ_OBST_MS : FUS_REACQ_MS;
    const uint8_t  idx          = (m.suspect < 0) ? 0 : (m.suspect > 0 ? 1 : 2);

    if (!f.candActive[idx] || fabsf(m.z - f.cand[idx]) > FUS_REACQ_TOL_CM)
    {
        f.cand[idx]       = m.z;
        f.candSince[idx]  = now_ms;
        f.candActive[idx] = true;
    }
    else if (now_ms - f.candSince[idx] >= need)
    {
        f.x += y; // = Messwert + Weg seit dem Messzeitpunkt
        f.P  = R;
        histShift(f, tMeas, y);
        for (uint8_t i = 0; i < 3; i++) f.candActive[i] = false;
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
    static bool     first        = true;
    static uint32_t lastUs       = 0;
    static float    lastHeading  = 0.0f;
    static float    vxPrev       = 0.0f; // zuletzt verwendete (vorgeschobene) Geschwindigkeit
    static float    vyPrev       = 0.0f;
    static uint32_t radarUsed[4] = {0, 0, 0, 0}; // Time des zuletzt verrechneten Radarwerts je Sensor
    static uint32_t lastSampleUs = 0;            // ADC-Zeitstempel der letzten Auswertung
    static float    vxSample = 0.0f, vySample = 0.0f; // Messwerte der vorigen ADC-Messung
    static float    axLP = 0.0f, ayLP = 0.0f;         // geglaettete Beschleunigung [cm/s²]
    static float    latEstX = 0.0f, latEstY = 0.0f;   // Tuning-Hilfe, siehe PDS 196/197

    // ---- Zeit & Drehrate ------------------------------------------------
    const uint32_t nowUs   = micros();
    const float    heading = giveOrbitDir(); // Grad, +-180
    float dt    = 0.0f;
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
            omega = DEG_TO_RAD * Util.AMap(heading - lastHeading) / dt;
    }
    lastUs      = nowUs;
    lastHeading = heading;

    // ---- Geschwindigkeit: Totzeit der Analogrueckmeldung ausgleichen ------
    float vxm = vx_field;
    float vym = vy_field;
    if (sqrtf(vxm * vxm + vym * vym) < FUS_V_DEADBAND) { vxm = 0.0f; vym = 0.0f; }

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
        fusionPredict(filtX, 0.5f * (vx + vxPrev) * dt, speed, fabsf(omega), dt, feld_x);
        fusionPredict(filtY, 0.5f * (vy + vyPrev) * dt, speed, fabsf(omega), dt, feld_y);
    }
    else
    {
        fusionPredict(filtX, 0.0f, FUS_ASSUMED_SPEED, fabsf(omega), dt, feld_x);
        fusionPredict(filtY, 0.0f, FUS_ASSUMED_SPEED, fabsf(omega), dt, feld_y);
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
        const float hAbs     = fabsf(heading);
        const bool  backward = (hAbs > 90.0f);
        const float tilt     = backward ? (180.0f - hAbs) : hAbs;

        if (tilt <= FUS_MAX_TILT_DEG)
        {
            const float tn         = tilt / 10.0f;
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
                                                       feld_x, 0.0f, 0.0f);
                for (uint8_t i = 0; i < mx.n; i++)
                    if (fusionUpdate(filtX, mx.m[i], tiltFactor, current_time_ms))
                        lastvaliddata_Radar_X = 0;
                markUsed(xl, xh);
            }

            // Tor-Offset aus der GEFILTERTEN X-Position (ruhiger als Rohradar)
            const float sigmaX = sqrtf(filtX.P);
            const bool  xFallback = (sigmaX > FUS_LOST_SIGMA);
            float goalSure = 1.0f;
            const float goalOff = goalOffset(xFallback ? feld_x / 2.0f : filtX.x, goalSure);

            if (xFallback)
                goalSure = min(goalSure, 0.5f);
            else if (sigmaX > POS_GOAL_EDGE_CM)
                goalSure = min(goalSure, (float)(POS_GOAL_EDGE_CM / sigmaX)); // X unsicher -> Tor/Wand unsicher

            Position.AtGoal = (goalOff > 0.5f);

            // Y
            const uint8_t yl = backward ? RadarVo : RadarHi;
            const uint8_t yh = backward ? RadarHi : RadarVo;
            if (isFresh(yl, yh))
            {
                AxisMeasSet my = radarAxisMeasurements(Radar[yl], Radar[yh],
                                                       Radar.giveAge(yl), Radar.giveAge(yh),
                                                       feld_y, goalOff, goalOff);
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
    if (filtX.accepted && fabsf(vx) > 30.0f) latEstX += 0.05f * ((-filtX.innov / vx) * 1000.0f - latEstX);
    if (filtY.accepted && fabsf(vy) > 30.0f) latEstY += 0.05f * ((-filtY.innov / vy) * 1000.0f - latEstY);

    // ---- Ausgabe --------------------------------------------------------
    const float sx = sqrtf(filtX.P);
    const float sy = sqrtf(filtY.P);

    Position.X = (sx > FUS_LOST_SIGMA) ? -1.0f : filtX.x;
    Position.Y = (sy > FUS_LOST_SIGMA) ? -1.0f : filtY.x;

    Position.XValid = (sx <= FUS_VALID_SIGMA);
    Position.YValid = (sy <= FUS_VALID_SIGMA);
    Position.Certainty = (Position.XValid && Position.YValid)
                         ? 1.0f / (1.0f + 0.3f * max(sx, sy))
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

uint16_t systemlib::giveADC(uint8_t Port, uint8_t Pin){
    if(Port == 0){return ADC_Line1.read(Pin);}
    else if(Port == 1){return ADC_Line2.read(Pin);}
    else if(Port == 2){return ADC_Line3.read(Pin);}
    else if(Port == 3){return ADC_Line4.read(Pin);}
    else if(Port == 4){return ADC_Line5.read(Pin);}
    else if(Port == 5){return ADC_Line6.read(Pin);}
    else{return 0;}
}

void systemlib::updateLine(){
    uint8_t line_summe=0;
    uint16_t line_depth=0;
    int8_t line_Angle = 0;
    bool line_touching;

    for(int i = 0; i<6 ; i++){
        for (int j = 0; j<8 ; j++){
            if(i == 0){line_array[j]= ADC_Line1.read(j);}
            else if(i == 1){line_array[8+j]= ADC_Line2.read(j);}
            else if(i == 2){line_array[16+j]= ADC_Line3.read(j);}
            else if(i == 3){line_array[24+j]= ADC_Line4.read(j);}
            else if(i == 4){line_array[32+j]= ADC_Line5.read(j);}
            else if(i == 5){line_array[40+j]= ADC_Line6.read(j);}
        }
    }

    for( int i = 0 ; i<48 ; i++){
        line_bool_array[i] = line_array[i]>LineSensorSchwelle[i];
        if(line_bool_array[i]){line_summe++;}
    }
    
    if(line_summe > 1){
        // 2) zwei Sensoren mit maximaler Winkel-Distanz finden (kreisförmig korrekt)
        float maxDistanz = -1.0f;
        float sensorA = 0.0f, sensorB = 0.0f;

        int aktiveSensoren[line_summe];
        int count = 0;
        for(int i = 0;i<48; i++){
            if(line_bool_array[i]==true){
            aktiveSensoren[count] = Util.AMap((i * 7.5));
            count++;
            }
        }

        for (int i = 0; i < line_summe; i++) {
            for (int j = i + 1; j < line_summe; j++) {
            float dist = fabsf(Util.AMap(aktiveSensoren[i] - aktiveSensoren[j]));
            if (dist > maxDistanz) {
                maxDistanz = dist;
                sensorA = aktiveSensoren[i];
                sensorB = aktiveSensoren[j];
            }
            }
        }

        // 3) Mittelwinkel zwischen A und B bestimmen (2 Möglichkeiten)
        float delta  = Util.AMap(sensorB - sensorA);
        line_depth = 1-abs(sin((abs(delta)/2)*DEG_TO_RAD));
        float mitte1 = Util.AMap(sensorA + (delta * 0.5f));
        float mitte2 = Util.AMap(mitte1 + 180.0f);

        // 4) Mitte wählen, die näher liegt
        line_Angle = (fabsf(Util.AMap(mitte1 - mitte2))) < (fabsf(Util.AMap(mitte2 - sensorA))) ? mitte1 : mitte2;
        line_touching=true;
    }
    else if(line_summe == 1){
        line_depth=1;
        line_touching=true;
        for(int i = 0 ; i<32 ; i++){
            if(line_bool_array[i]==true){
                line_Angle=(i*11.25)-90;
            }
        }
    }
    else{
        line_depth=1;
        line_touching=false;
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
    BNO_OrbitDir = Util.AMap(BNO_eulerZ - BNO_calibration);
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

    IR_Angle = Util.AMap(IR_rawAngle - BNO_OrbitDir);
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

void systemlib::setMotorData(uint8_t ID, uint8_t Speed, bool Dir, bool Enable, bool Break){
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

void systemlib::setMotorData(uint8_t ID, int8_t Speed, bool Enable, bool Break){
    bool Dir;
    
    if (Speed<0){Dir=false;}
    else{Dir=true;}

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
    uint8_t Port_List[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    byte out = 0;
    if (MoDataVL.Speed == 0 || giveExpander(0,3))
    {
        MoDataVL.Speed = 0;
        MoDataVL.Enable = false;
        MoDataVL.Break = true;
    }
    if (MoDataVR.Speed == 0 || giveExpander(0,3))
    {
        MoDataVR.Speed = 0;
        MoDataVR.Enable = false;
        MoDataVR.Break = true;
    }
    if (MoDataHL.Speed == 0 || giveExpander(0,3))
    {
        MoDataHL.Speed = 0;
        MoDataHL.Enable = false;
        MoDataHL.Break = true;
    }
    if (MoDataHR.Speed == 0 || giveExpander(0,3))
    {
        MoDataHR.Speed = 0;
        MoDataHR.Enable = false;
        MoDataHR.Break = true;
    }
    if (MoDataDR.Speed == 0 || giveExpander(0,3))
    {
        MoDataDR.Speed = 0;
        MoDataDR.Enable = false;
        MoDataDR.Break = true;
    }

    analogWrite(MotorVL_PWM, abs(MoDataVL.Speed) * 2.55);
    analogWrite(MotorVR_PWM, abs(MoDataVR.Speed) * 2.55);
    analogWrite(MotorHR_PWM, abs(MoDataHR.Speed) * 2.55);
    analogWrite(MotorHL_PWM, abs(MoDataHL.Speed) * 2.55);

    if (MoDataVL.Dir)
    {
        Port_List[MotorVL_P1] = 0;
    }
    else
    {
        Port_List[MotorVL_P1] = 1;
    }
    if (MoDataVR.Dir)
    {
        Port_List[MotorVR_P1] = 0;
    }
    else
    {
        Port_List[MotorVR_P1] = 1;
    }
    if (MoDataHL.Dir)
    {
        Port_List[MotorHL_P1] = 0;
    }
    else
    {
        Port_List[MotorHL_P1] = 1;
    }
    if (MoDataHR.Dir)
    {
        Port_List[MotorHR_P1] = 0;
    }
    else
    {
        Port_List[MotorHR_P1] = 1;
    }

    if (MoDataVL.Enable)
    {
        Port_List[MotorVL_P2] = 1;
    }
    else
    {
        Port_List[MotorVL_P2] = 0;
    }
    if (MoDataVR.Enable)
    {
        Port_List[MotorVR_P2] = 1;
    }
    else
    {
        Port_List[MotorVR_P2] = 0;
    }
    if (MoDataHL.Enable)
    {
        Port_List[MotorHL_P2] = 1;
    }
    else
    {
        Port_List[MotorHL_P2] = 0;
    }
    if (MoDataHR.Enable)
    {
        Port_List[MotorHR_P2] = 1;
    }
    else
    {
        Port_List[MotorHR_P2] = 0;
    }

    // Konvertieren von Liste zu byte
    for (int i = 0; i < 8; i++)
    {
        if (Port_List[i] == 1)
        {
            out |= (1 << (7 - i));
        }
    }

    Motor_EXP.write(out);

    /*for(int i = 0 ; i<8 ; i++){
        Serial.print("channel ");
        Serial.print(i);
        Serial.print(" : ");
        Serial.println(ADC_Motor.read(i));
    }

    Serial.println(ADC_Motor.read(0)-ADC_Motor.read(1));
    */

    motorSampleUs = micros(); // Zeitstempel der Drehzahlmessung (fuer Totzeitausgleich in calculatePosition)
    MoDataVL.Speed_real = ADC_Motor.read(MotorVL_AP1,true,true);
    MoDataVL.Current = ADC_Motor.read(MotorVL_AP2,true,false);
    MoDataVR.Speed_real = ADC_Motor.read(MotorVR_AP1,true,true);
    MoDataVR.Current = ADC_Motor.read(MotorVR_AP2,true,false);
    MoDataHL.Speed_real = ADC_Motor.read(MotorHL_AP1,true,true);
    MoDataHL.Current = ADC_Motor.read(MotorHL_AP2,true,false);
    MoDataHR.Speed_real = ADC_Motor.read(MotorHR_AP1,true,true);
    MoDataHR.Current = ADC_Motor.read(MotorHR_AP2,true,false);
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
    SW_EXP.read();     // Update the internal state of the port expander
    Button_EXP.read(); // Update the internal state of the port expander

    SWI[0] = SW_EXP.readPin(4);
    SWI[1] = SW_EXP.readPin(3);
    SWI[2] = SW_EXP.readPin(2);
    SWI[3] = SW_EXP.readPin(1);
    SWI[4] = SW_EXP.readPin(0);

    startSwitch = SW_EXP.readPin(5);
    RCJ = Button_EXP.readPin(5);
    RCJ_Disable = SW_EXP.readPin(6);
    RCJ_present = SW_EXP.readPin(7);

    /* BTN1.update(Button_EXP.readPin(0));
    BTN2.update(Button_EXP.readPin(1));
    BTN3.update(Button_EXP.readPin(2));
    BTN4.update(Button_EXP.readPin(3));
    BTN5.update(Button_EXP.readPin(4));*/

    BTN1.update(SW_EXP.readPin(0));
    BTN2.update(SW_EXP.readPin(6));
    BTN3.update(SW_EXP.readPin(7));

    fancyBTNstates[0] = BTN1.getStatusCode();
    fancyBTNstates[1] = BTN2.getStatusCode();
    fancyBTNstates[2] = BTN3.getStatusCode();
    fancyBTNstates[3] = BTN4.getStatusCode();
    fancyBTNstates[4] = BTN5.getStatusCode();

    // Read the state of each pin from the port expander
    // Example: bool pin0State = portExpander.getPinState(0);
}

bool systemlib::giveExpander(uint8_t expnum, uint8_t portnum)
{
    if (expnum == 0)
    {
        return SW_EXP.readPin(portnum);
    }
    else if (expnum == 1)
    {
        return Button_EXP.readPin(portnum);
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
        cfgA.Ts = TS_NOMINAL;           // nur Fallback, siehe loop()
        cfgA.dtMin = 1e-5f;             // 10 us -- gegen dt=0
        cfgA.dtMax = 0.03f;             // 30 ms -- deckelt Ausreisser, falls das
                                        // Hauptprogramm mal laenger braucht
        cfgA.wc = 15.0f;                // Reglerbandbreite: wie schnell ausgerichtet wird
        cfgA.wo = 80.0f;                // Beobachterbandbreite: ca. 5x wc als Startwert
        cfgA.b0 = 8.0f;                 // grobe Schaetzung: wie stark omega_cmd auf
                                        // die Winkelbeschleunigung wirkt -> per Test anpassen
        cfgA.uMin = -1.0f;
        cfgA.uMax = 1.0f;
        cfgA.angleMode = true;       // WICHTIG: sorgt fuer kuerzesten Drehweg (kein +-pi Sprung)
        cfgA.outputRateLimit = 6.0f; // begrenzt Stellgroessen-Aenderungsrate (sanfter Anlauf)

        headingControllerA = new ADRC(cfgA);
        headingControllerA->reset(DEG_TO_RAD * BNO_OrbitDir);
        lastMicrosContrl = micros();
    }
    else
    {
        cfgP.Ts = TS_NOMINAL; // nur Fallback, siehe loop()
        cfgP.dtMin = 1e-5f;   // 10 us -- gegen dt=0
        cfgP.dtMax = 0.03f;   // 30 ms -- deckelt Ausreisser, falls das
                              // Hauptprogramm mal laenger braucht

        cfgP.Kp = 0.2; // Start: nur P, dann I/D schrittweise zuschalten
        cfgP.Ki = 0;
        cfgP.Kd = 0.016f;

        cfgP.uMin = -1.0f;
        cfgP.uMax = 1.0f;
        cfgP.angleMode = true;               // WICHTIG: kuerzester Drehweg, kein +-pi Sprung
        cfgP.derivativeOnMeasurement = true; // kein Kick bei Sollwinkel-Spruengen
        cfgP.Tf = 0.02f;                     // D-Anteil filtern (Winkelmessung ist meist verrauscht)
        cfgP.outputRateLimit = 6.0f;         // sanfter Anlauf / Ruckbegrenzung

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
    headingControllerA->reset(DEG_TO_RAD * BNO_OrbitDir);
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

void systemlib::moveToPoint(uint16_t X, uint16_t Y, uint8_t Speed,int16_t Andrehwinkel){

    float dx = X - Position.X;
    float dy = Y - Position.Y;

    float distanzCm = sqrtf(dy * dy + dx * dx);
    Speed = Speed*constrain(distanzCm / 40.0f, 0.0f, 1.0f); // 0 = am Ziel, 1 = 40cm oder mehr entfernt

    if(abs(distanzCm)<5){
        Speed=0;
    }

    setHeading(Andrehwinkel*DEG_TO_RAD);
    setOmniWheelDir(Util.AMap(atan2f(-dx, dy) * (180.0f / PI)),Speed,giveHeadingController()*100);
}

namespace
{
    struct MoveState
    {
        bool     active     = false;
        bool     arrived    = false;
        bool     correcting = false; // nach dem Wegschieben: Feinkorrektur mit MinSpeed
        float    tx = 0.0f, ty = 0.0f;
        uint8_t  pattern = 0, maxS = 0, minS = 0;
        float    dist0    = 0.0f;    // Abstand beim Start des Patterns
        float    progress = 0.0f;
        float    lastSpeed = 0.0f;
        uint32_t lastCallMs = 0;
        uint32_t lastUs     = 0;
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
        case SPattern_Linear:        f = 1.0f - p; break;
        case SPattern_LinearUp:      f = p; break;
        case SPattern_Constant:      f = 1.0f; break;
        case SPattern_Trapezoid:     f = min(min(p / 0.2f, 1.0f), (1.0f - p) / 0.4f); break;
        case SPattern_Triangle:      f = 1.0f - fabsf(1.0f - 2.0f * p); break;
        case SPattern_QuadDecel:     f = (1.0f - p) * (1.0f - p); break;
        case SPattern_ConstDecel:    f = sqrtf(1.0f - p); break;
        case SPattern_SmoothStep:    f = 1.0f - smoothstep01(p); break;
        case SPattern_Cosine:        f = 0.5f * (1.0f + cosf(PI * p)); break;
        case SPattern_Sine:          f = sinf(PI * p); break;
        case SPattern_SCurve:        f = min(smoothstep01(p / 0.25f), 1.0f - smoothstep01((p - 0.6f) / 0.4f)); break;
        case SPattern_LateBrake:     f = (p < 0.7f) ? 1.0f : 0.5f * (1.0f + cosf(PI * (p - 0.7f) / 0.3f)); break;
        case SPattern_FastThenCrawl: f = (p < 0.5f) ? 1.0f : ((p < 0.7f) ? 1.0f - (p - 0.5f) / 0.2f : 0.0f); break;
        case SPattern_QuadUp:        f = p * p; break;
        case SPattern_CubicUp:       f = p * p * p; break;
        case SPattern_Expo:          f = (expf(-3.0f * p) - expf(-3.0f)) / (1.0f - expf(-3.0f)); break;
        case SPattern_Stairs:        f = 1.0f - min(floorf(p * 4.0f), 3.0f) / 3.0f; break;
        case SPattern_Wave:          f = (1.0f - p) * (0.6f + 0.4f * sinf(2.0f * PI * 3.0f * p)); break;
        case SPattern_Pulse:         f = (fmodf(p * 5.0f, 1.0f) < 0.6f) ? 1.0f : 0.0f; break;
        default:                     f = 1.0f; break; // unbekanntes Pattern: konstant
        }
        return constrain(f, 0.0f, 1.0f);
    }
}

void systemlib::resetMoveProfile(void)
{
    mv.active     = false;
    mv.lastSpeed  = 0.0f;
}

bool  systemlib::giveMoveArrived(void) const  { return mv.active && mv.arrived; }

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

    if (MaxSpeed > 100) MaxSpeed = 100;
    if (MinSpeed > MaxSpeed) MinSpeed = MaxSpeed;

    const uint32_t nowMs = millis();
    const uint32_t nowUs = micros();
    float dt = (mv.lastUs != 0) ? (nowUs - mv.lastUs) * 1e-6f : 0.0f;
    dt = constrain(dt, 0.0f, 0.1f);
    mv.lastUs = nowUs;

    // Position verloren: anhalten, Profil verwerfen
    if (Position.X < 0.0f || Position.Y < 0.0f)
    {
        resetMoveProfile();
        mv.lastCallMs = nowMs;
        setOmniWheelDir(0, 0, omega);
        return;
    }

    const float tx = constrain((float)X, 0.0f, feld_x);
    const float ty = constrain((float)Y, 0.0f, feld_y);
    const float dx = tx - Position.X;
    const float dy = ty - Position.Y;
    const float dist = sqrtf(dx * dx + dy * dy);

    // ---- Neustart des Patterns? -----------------------------------------
    const bool sameTarget = (fabsf(tx - mv.tx) <= MOVE_TARGET_TOL_CM) && (fabsf(ty - mv.ty) <= MOVE_TARGET_TOL_CM);
    const bool restart = !mv.active || !sameTarget
                         || (SpeedPattern != mv.pattern) || (MaxSpeed != mv.maxS) || (MinSpeed != mv.minS)
                         || ((nowMs - mv.lastCallMs) > (uint32_t)MOVE_RESTART_GAP_MS);
    mv.lastCallMs = nowMs;

    if (restart)
    {
        mv.active     = true;
        mv.tx = tx;  mv.ty = ty;
        mv.pattern = SpeedPattern;  mv.maxS = MaxSpeed;  mv.minS = MinSpeed;
        mv.dist0      = dist;
        mv.progress   = 0.0f;
        mv.arrived    = (dist <= MOVE_ARRIVE_CM);
        mv.correcting = false;
        // lastSpeed bleibt: rollt der Roboter noch, wird der Speed nicht ruckartig umgesetzt
    }

    // ---- Zielkreis mit Hysterese -----------------------------------------
    if (mv.arrived)
    {
        if (dist > MOVE_ARRIVE_CM + MOVE_ARRIVE_HYST_CM) // weggeschoben -> Feinkorrektur
        {
            mv.arrived    = false;
            mv.correcting = true;
            mv.dist0      = dist;
            mv.progress   = 0.0f;
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

    setOmniWheelDir(Util.AMap(atan2f(-dx, dy) * (180.0f / PI)),
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
#ifndef ARRAY_MAX_POINTS
#define ARRAY_MAX_POINTS 16
#endif
#ifndef ARRAY_HEADING_TOL_DEG
#define ARRAY_HEADING_TOL_DEG 5.0f // 0 = Winkel wird beim Weiterschalten nicht geprueft
#endif

namespace
{
    struct ArrayData
    {
        ArrayPoint pts[ARRAY_MAX_POINTS];
        uint8_t    count       = 0;
        uint8_t    index       = 0;     // aktueller Zielpunkt
        bool       loop        = false;
        ArrayState state       = ArrayIdle;
        bool       holding     = false; // Punkt erreicht, Wartezeit laeuft
        uint32_t   holdStartMs = 0;
    };
    ArrayData pa;

    bool samePoint(const ArrayPoint &a, const ArrayPoint &b)
    {
        return a.X == b.X && a.Y == b.Y && a.Andrehwinkel == b.Andrehwinkel
               && a.SpeedPattern == b.SpeedPattern && a.MaxSpeed == b.MaxSpeed
               && a.MinSpeed == b.MinSpeed && a.HoldMs == b.HoldMs;
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
    pa.count   = count;
    pa.loop    = loop;
    pa.index   = 0;
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
    p.X            = X;
    p.Y            = Y;
    p.Andrehwinkel = Andrehwinkel;
    p.SpeedPattern = SpeedPattern;
    p.MaxSpeed     = MaxSpeed;
    p.MinSpeed     = MinSpeed;
    p.HoldMs       = HoldMs;
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
        pa.index   = pa.count - 1;
        pa.state   = ArrayRunning;
        pa.holding = false;
        resetMoveProfile();
    }
    return true;
}

/** Loescht alle Punkte und setzt den Zustand auf Idle. updateArray() sendet dann keine Motorbefehle mehr. */
void systemlib::resetArray(void)
{
    pa.count   = 0;
    pa.index   = 0;
    pa.loop    = false;
    pa.state   = ArrayIdle;
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

    pa.index   = 0;
    pa.holding = false;
    pa.state   = ArrayRunning;
    resetMoveProfile();
}

/** Haelt den Roboter an (Heading wird weiter geregelt). Der Fortschritt bleibt erhalten. */
void systemlib::pauseArray(void)
{
    if (pa.state == ArrayRunning)
    {
        pa.state   = ArrayPaused;
        pa.holding = false;
        resetMoveProfile();
    }
}

/** Setzt eine pausierte Liste am aktuellen Punkt fort (das Speed-Pattern startet fuer diesen Punkt neu). */
void systemlib::resumeArray(void)
{
    if (pa.state == ArrayPaused)
    {
        pa.state   = ArrayRunning;
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

    // ---- Punkt erreicht? ----------------------------------------------------
    const float headingErr = fabsf(Util.AMap(p.Andrehwinkel - giveOrbitDir()));
    const bool  reached    = giveMoveArrived() && (ARRAY_HEADING_TOL_DEG <= 0.0f || headingErr <= ARRAY_HEADING_TOL_DEG);

    if (!reached)
    {
        pa.holding = false;
        return;
    }

    const uint32_t nowMs = millis();
    if (!pa.holding)
    {
        pa.holding     = true;
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
uint8_t    systemlib::giveArrayIndex(void) const { return pa.index; }
uint8_t    systemlib::giveArrayCount(void) const { return pa.count; }

void systemlib::setOmniWheelVek(float vx, float vy, float omega) {
    float w[4];
    static const float wheelAngles[4] = {45,315,135,225};
    for (int i = 0; i < 4; ++i) {
        w[i] = -sinf(wheelAngles[i]*DEG_TO_RAD) * -vx
                  +  cosf(wheelAngles[i]*DEG_TO_RAD) * vy
                  + omega;
        w[i] = constrain(w[i], -100, 100);
        
    }
    setMotorData(MoDataVL.ID,w[0],true,false);
    setMotorData(MoDataVR.ID,w[1],true,false);
    setMotorData(MoDataHL.ID,w[2],true,false);
    setMotorData(MoDataHR.ID,w[3],true,false);
}

void systemlib::setOmniWheelDir(float directionDeg, uint8_t speedPercent, float omega) {
    directionDeg = directionDeg-giveOrbitDir();

    directionDeg = Util.AMap(directionDeg);

    float w[4];
    static const float wheelAngles[4] = {45,315,135,225};
    // Serial.println("omega : "+String(omega));  // deaktiviert: blockierendes USB-Serial im 100-Hz-Takt verfaelscht dt
 
    float directionRad = directionDeg * DEG_TO_RAD;
    float speedNorm     = speedPercent / 100.0f; // 0..1
 
    float vx = cosf(directionRad) * speedNorm * 100;
    float vy = sinf(directionRad) * speedNorm * 100;
 
    for (int i = 0; i < 4; ++i) {
        w[i] = -sinf(wheelAngles[i]*DEG_TO_RAD) * -vx
                  +  cosf(wheelAngles[i]*DEG_TO_RAD) * vy
                  + omega;
        w[i] = constrain(w[i], -100, 100);
    }

    setMotorData(MoDataVL.ID,w[0],true,false);
    setMotorData(MoDataVR.ID,w[1],true,false);
    setMotorData(MoDataHL.ID,w[2],true,false);
    setMotorData(MoDataHR.ID,w[3],true,false);
}

int checkWallDistance(float posX, float posY)
{
    const float MARGIN_CM = 20.0f;
    const float FIELD_X_CM = 183.0f;
    const float FIELD_Y_CM = 243.0f;

    // Ungültige Positionen (z. B. -1 aus der Fallback-Logik) direkt abfangen
    if (posX < 0.0f || posY < 0.0f)
    {
        return -1;
    }

    // Prüfen, ob eine der 4 Wände unterschritten wird
    if (posX < MARGIN_CM || posX > (FIELD_X_CM - MARGIN_CM) ||
        posY < MARGIN_CM || posY > (FIELD_Y_CM - MARGIN_CM))
    {
        return 1; // Näher als 10 cm an der Wand
    }

    return 0; // Sicherer Bereich
}

void systemlib::moveDir(int16_t Dir,uint8_t Speed,float Andrehwinkel){
    setHeading(Andrehwinkel*DEG_TO_RAD);
    if(checkWallDistance(Position.X, Position.Y)==1){
        setOmniWheelDir(Dir,5,giveHeadingController()*100);
    }
    else{
        setOmniWheelDir(Dir,Speed,giveHeadingController()*100);
    }
}