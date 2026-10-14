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

    float adcValues[4] = { MoDataVL.Speed_real, MoDataVR.Speed_real, MoDataHL.Speed_real, MoDataHR.Speed_real };
    RobotVelocity vel = calculateFieldVelocityFromMotorRpm(adcValues, giveOrbitDir());
    calculatePosition(millis(), vel.vx_field, vel.vy_field);
    // readIR();
    updateLine();

    readExpanders();

    // Werte fuer die PDS-Kanaele nachfuehren
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

void systemlib::setupPDS(void)
{
    #if DPDS_Mode == D_Mode_Test

#if defined(BUILD_VERSION) && defined(BUILD_DATE) && defined(BUILD_TIME)
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
    PDS.bind("Ball_Winkel",   &IR_Angle,     8,  "°");
    PDS.bind("Ball_Dist",     &IR_Distance,  11);

    /* ---- Zykluszeit ---------------------------------------------------- */
    PDS.bind("Zykluszeit",    &cycleTime,    9,  "ms");
    PDS.bind("Zykluszeit_us", &cycleTime_us, 10, "us");

    /* ---- Radar --------------------------------------------------------- */
    PDS.bind("Radar_Vo",      &Radar[RadarVo].Strenght, 12, "Counts");
    PDS.bind("Radar_Li",      &Radar[RadarLi].Strenght, 13, "Counts");
    PDS.bind("Radar_Hi",      &Radar[RadarHi].Strenght, 14, "Counts");
    PDS.bind("Radar_Re",      &Radar[RadarRe].Strenght, 15, "Counts");

    /* ---- Feldposition -------------------------------------------------- */
    PDS.bind("Pos_X",         &Position.X,         16, "cm");
    PDS.bind("Pos_Y",         &Position.Y,         17, "cm");
    PDS.bind("Pos_certanty_x", &Position.XValid, 18);
    PDS.bind("Pos_certanty_y", &Position.YValid, 19);

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

    #endif // Game_mode
}

/* -------------- Position/Radar -------------- */

/**
 * Alter eines Messwerts -> Gewicht zwischen 1 (brandaktuell) und 0 (uralt).
 */
static float posAgeFactor(uint32_t age_ms)
{
    if (age_ms >= POS_AGE_MAX_MS)
        return 0.0f;

    return 1.0f - ((float)age_ms / (float)POS_AGE_MAX_MS);
}

/**
 * Tiefe, um die die Y-Achse an EINEM Feldende durch das Tor kuerzer wird.
 */
float systemlib::goalOffset(float x_cm, float &sureness) const
{
    sureness = 1.0f;

    if (POS_GOAL_DEPTH_CM == 0.0f || POS_GOAL_WIDTH_CM <= 0.0f)
        return 0.0f;

    const float halfGoal = POS_GOAL_WIDTH_CM / 2.0f;
    const float dx       = fabsf(x_cm - (feld_x / 2.0f));

    if (dx <= halfGoal - POS_GOAL_EDGE_CM)
        return POS_GOAL_DEPTH_CM;

    if (dx >= halfGoal + POS_GOAL_EDGE_CM)
        return 0.0f;

    const float t = constrain((halfGoal + POS_GOAL_EDGE_CM - dx) / (2.0f * POS_GOAL_EDGE_CM), 0.0f, 1.0f);
    sureness = fabsf(2.0f * t - 1.0f);

    return POS_GOAL_DEPTH_CM * t;
}

/**
 * Rechnet Motordrehzahl in Radgeschwindigkeit (cm/s) um.
 */
float systemlib::convertMotorRpmToWheelSpeedCmS(float motor_rpm)
{
    float wheel_rpm = motor_rpm / GEAR_RATIO;
    float v_cms = (wheel_rpm * 2.0f * M_PI * WHEEL_RADIUS_CM) / 60.0f;
    return v_cms;
}

/**
 * Berechnet aus den 4 Motordrehzahlen und dem IMU-Winkel die Feld-Geschwindigkeiten.
 */
RobotVelocity systemlib::calculateFieldVelocityFromMotorRpm(const float motor_rpm[4], float heading_deg)
{
    // 1. Rad-Geschwindigkeiten berechnen (cm/s)
    float v_VL = convertMotorRpmToWheelSpeedCmS(motor_rpm[0]); // VL
    float v_VR = convertMotorRpmToWheelSpeedCmS(motor_rpm[1]); // VR
    float v_HL = convertMotorRpmToWheelSpeedCmS(motor_rpm[2]); // HL
    float v_HR = convertMotorRpmToWheelSpeedCmS(motor_rpm[3]); // HR

    static constexpr float s = 0.70710678f;  // √2/2

    // Praezise float-Berechnung statt Integer-Rundung
    float vy_rob = 0.5f * s * ( v_VL - v_VR + v_HL - v_HR);
    float vx_rob = -0.5f * s * ( v_VL - v_VR - v_HL + v_HR);

    // 2. Transformation: Roboter-Koordinaten -> Feld-Koordinaten
    float heading_rad = heading_deg * (M_PI / 180.0f);
    float cos_h = cosf(heading_rad);
    float sin_h = sinf(heading_rad);

    // Kalibrierfaktoren fuer Radschlupf & ADC-Mapping
    static constexpr float SCALE_X = 0.83f;
    static constexpr float SCALE_Y = 0.857f;

    RobotVelocity vel;
    vel.vx_field = (vx_rob * cos_h - vy_rob * sin_h) * SCALE_X;
    vel.vy_field = (vx_rob * sin_h + vy_rob * cos_h) * SCALE_Y;

    PDS.Channel(190, vel.vx_field);
    PDS.Channel(191, vel.vy_field);

    return vel;
}

/**
 * Berechnet die minimal erforderliche Signalstärke in mdB.
 */
static inline int32_t getMinStrengthMdb(float distCm)
{
    float distM = distCm / 100.0f;
    float minDb = -11000.0f - (distM * 2500.0f);
    return (int32_t)max(minDb, -17000.0f);
}

/**
 * Eine Achse aus dem gegenüberliegenden Radar-Paar verifizieren und berechnen.
 * Verwendet lastPos (Odometrie-Vorhersage) zur Hindernis- und Sprung-Erkennung.
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

    // 3. Hinderniserkennung (z.B. Gegner fährt dazwischen)
    // Wenn ein Sensor eine deutlich kürzere Distanz als erwartet misst, wird er ignoriert.
    if (lastPos >= 0.0f)
    {
        float expectedDistLow = lastPos - offsetLow;
        float expectedDistHigh = (span + offsetLow) - lastPos;

        if (lowOk && (expectedDistLow - distLow > 15.0f))
        {
            lowOk = false; // Hindernis vor Sensor Low
        }
        if (highOk && (expectedDistHigh - distHigh > 15.0f))
        {
            highOk = false; // Hindernis vor Sensor High
        }
    }

    // 4. Position und Plausibilität
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
            // Konflikt-Auflösung über Odometrie-Abgleich
            if (lastPos >= 0.0f)
            {
                float posFromLow = distLow;
                float posFromHigh = span - distHigh;
                float errLow = fabsf(posFromLow - (lastPos - offsetLow));
                float errHigh = fabsf(posFromHigh - (lastPos - offsetLow));

                bool useLow = (errLow <= errHigh);
                pos   = useLow ? posFromLow : posFromHigh;
                plaus = (min(errLow, errHigh) < 15.0f) ? POS_PLAUS_SINGLE * 0.7f : POS_PLAUS_CONFLICT * 0.2f;
                age   = Radar.giveAge(useLow ? posLow : posHigh);
                usedStrength = useLow ? low.Strenght : high.Strenght;
            }
            else
            {
                int32_t diffMdb = low.Strenght - high.Strenght;
                bool useLow = (diffMdb > 0);
                pos   = useLow ? distLow : (span - distHigh);
                plaus = POS_PLAUS_CONFLICT;
                age   = Radar.giveAge(useLow ? posLow : posHigh);
                usedStrength = useLow ? low.Strenght : high.Strenght;
            }
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
        return (lastPos >= 0.0f) ? lastPos : (span / 2.0f);
    }

    // 5. Alter und Signalstärke-Gewichtung
    float strengthFactor = constrain(1.0f + (usedStrength + 10000.0f) / 15000.0f, 0.7f, 1.0f);
    certainty = constrain(plaus * posAgeFactor(age) * strengthFactor, 0.0f, 1.0f);

    float calculatedPos = offsetLow + constrain(pos, 0.0f, span);

    // 6. Sprungbegrenzung bezogen auf Odometrie
    const float MAX_JUMP_CM = 15.0f;
    if (lastPos >= 0.0f && fabsf(calculatedPos - lastPos) > MAX_JUMP_CM)
    {
        certainty *= 0.15f; // Starkes Abdämpfen bei Sprüngen
    }

    return calculatedPos;
}

/**
 * Feldposition berechnen mit Fusion von Reifen-Odometrie und Radar.
 */
PositionData systemlib::calculatePosition(uint32_t current_time_ms, float vx_field, float vy_field, bool use_radar, bool use_odometry)
{
    // 1. Delta-Zeit in Sekunden berechnen
    static uint32_t last_time_ms = 0;
    float dt_s = 0.0f;

    if (last_time_ms > 0 && current_time_ms > last_time_ms)
    {
        dt_s = (float)(current_time_ms - last_time_ms) / 1000.0f;
    }
    last_time_ms = current_time_ms;

    if (dt_s > 0.1f) dt_s = 0.005f; // Schutz vor langen Pausen

    // Start-Initialisierung bei ungültigen Werten
    if (Position.X < 0.0f) Position.X = feld_x / 2.0f;
    if (Position.Y < 0.0f) Position.Y = feld_y / 2.0f;

    // 2. Koppelnavigation (Odometrie): Reifen-Integration als Hauptbasis
    float predX = Position.X;
    float predY = Position.Y;

    if (use_odometry && dt_s > 0.0f)
    {
        predX += vx_field * dt_s;
        predY += vy_field * dt_s;
        predX = constrain(predX, 0.0f, feld_x);
        predY = constrain(predY, 0.0f, feld_y);
    }

    // 3. Drehraten-Einfluss bestimmen (Dämpfung von Radar bei schneller Eigenrotation)
    static float last_heading_deg = 0.0f;
    float current_heading = giveOrbitDir();
    float heading_diff = fabsf(Util.AMap(current_heading - last_heading_deg));
    last_heading_deg = current_heading;

    float turn_rate_deg_s = (dt_s > 0.0001f) ? (heading_diff / dt_s) : 0.0f;
    // Dämpfungsfaktor: 1.0 bei Stillstand, fällt ab bei schnellen Rotationen (>20°/s)
    float turn_factor = constrain(1.0f - (turn_rate_deg_s - 20.0f) / 100.0f, 0.1f, 1.0f);

    // 4. X-Achse berechnen & verifizieren
    float certainty_x = 0.0f;
    float radar_x = predX;

    if (use_radar)
    {
        radar_x = calcAxisPosition(RadarLi, RadarRe, feld_x, 0.0f, 0.0f, predX, certainty_x);
        certainty_x *= turn_factor; // Radar während des Drehens abdämpfen
    }

    // Komplementärfilter / Fusion für X-Achse
    if (use_radar && certainty_x >= POS_MIN_THRESHOLD)
    {
        lastvaliddata_Radar_X = 0;

        // Alpha-Faktor steuert die Stärke der Radar-Korrektur
        float alpha_x = 0.12f * certainty_x;
        if (!use_odometry) alpha_x = 1.0f; // Ohne Odometrie direkt übernehmen

        Position.X = predX + alpha_x * (radar_x - predX);
    }
    else
    {
        bool radarTimeout = use_radar && (lastvaliddata_Radar_X >= 2000);
        if (use_odometry && !radarTimeout)
        {
            Position.X = predX;
        }
        else if (!use_odometry && certainty_x < POS_MIN_THRESHOLD)
        {
            Position.X = -1.0f;
        }
        else
        {
            Position.X = predX;
        }
    }

    // 5. Y-Achse & Tor-Berechnung
    float xForGoal = constrain(Position.X, 0.0f, feld_x);
    float goalSure = 1.0f;
    const float goalOff = goalOffset(xForGoal, goalSure);

    float certainty_y = 0.0f;
    float radar_y = predY;

    if (use_radar)
    {
        radar_y = calcAxisPosition(RadarHi, RadarVo, feld_y, goalOff, goalOff, predY, certainty_y);
        certainty_y *= POS_PLAUS_GOAL_EDGE + (1.0f - POS_PLAUS_GOAL_EDGE) * goalSure;
        certainty_y *= turn_factor; // Radar während des Drehens abdämpfen
    }

    Position.AtGoal = (goalOff > 0.5f);

    // Komplementärfilter / Fusion für Y-Achse
    if (use_radar && certainty_y >= POS_MIN_THRESHOLD)
    {
        lastvaliddata_Radar_Y = 0;

        float alpha_y = 0.12f * certainty_y;
        if (!use_odometry) alpha_y = 1.0f;

        Position.Y = predY + alpha_y * (radar_y - predY);
    }
    else
    {
        bool radarTimeout = use_radar && (lastvaliddata_Radar_Y >= 2000);
        if (use_odometry && !radarTimeout)
        {
            Position.Y = predY;
        }
        else if (!use_odometry && certainty_y < POS_MIN_THRESHOLD)
        {
            Position.Y = -1.0f;
        }
        else
        {
            Position.Y = predY;
        }
    }

    // 6. Validitäts-Flags & Certainty
    Position.XValid = (certainty_x >= POS_TRUST_THRESHOLD) || use_odometry;
    Position.YValid = (certainty_y >= POS_TRUST_THRESHOLD) || use_odometry;

    Position.Certainty = (certainty_x + certainty_y) * 0.5f;

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

        float delta  = Util.AMap(sensorB - sensorA);
        line_depth = 1-abs(sin((abs(delta)/2)*DEG_TO_RAD));
        float mitte1 = Util.AMap(sensorA + (delta * 0.5f));
        float mitte2 = Util.AMap(mitte1 + 180.0f);

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
    IR_Distance = IR_rawDistance;
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
    ir_ring.read_raw_values(IR_rawSensorData);

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
    uint8_t Port_List[8];
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

    for (int i = 0; i < 8; i++)
    {
        if (Port_List[i] == 1)
        {
            out |= (1 << (7 - i));
        }
    }

    Motor_EXP.write(out);

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
    SW_EXP.read();
    Button_EXP.read();

    SWI[0] = SW_EXP.readPin(4);
    SWI[1] = SW_EXP.readPin(3);
    SWI[2] = SW_EXP.readPin(2);
    SWI[3] = SW_EXP.readPin(1);
    SWI[4] = SW_EXP.readPin(0);

    startSwitch = SW_EXP.readPin(5);
    RCJ = Button_EXP.readPin(5);
    RCJ_Disable = SW_EXP.readPin(6);
    RCJ_present = SW_EXP.readPin(7);

    BTN1.update(SW_EXP.readPin(0));
    BTN2.update(SW_EXP.readPin(6));
    BTN3.update(SW_EXP.readPin(7));

    fancyBTNstates[0] = BTN1.getStatusCode();
    fancyBTNstates[1] = BTN2.getStatusCode();
    fancyBTNstates[2] = BTN3.getStatusCode();
    fancyBTNstates[3] = BTN4.getStatusCode();
    fancyBTNstates[4] = BTN5.getStatusCode();
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
        cfgA.order = ADRCOrder::SECOND;
        cfgA.Ts = TS_NOMINAL;
        cfgA.dtMin = 1e-5f;
        cfgA.dtMax = 0.03f;
        cfgA.wc = 15.0f;
        cfgA.wo = 80.0f;
        cfgA.b0 = 8.0f;
        cfgA.uMin = -1.0f;
        cfgA.uMax = 1.0f;
        cfgA.angleMode = true;
        cfgA.outputRateLimit = 6.0f;

        headingControllerA = new ADRC(cfgA);
        headingControllerA->reset(DEG_TO_RAD * BNO_OrbitDir);
        lastMicrosContrl = micros();
    }
    else
    {
        cfgP.Ts = TS_NOMINAL;
        cfgP.dtMin = 1e-5f;
        cfgP.dtMax = 0.03f;

        cfgP.Kp = 0.2;
        cfgP.Ki = 0;
        cfgP.Kd = 0.016f;

        cfgP.uMin = -1.0f;
        cfgP.uMax = 1.0f;
        cfgP.angleMode = true;
        cfgP.derivativeOnMeasurement = true;
        cfgP.Tf = 0.02f;
        cfgP.outputRateLimit = 6.0f;

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
    float dt = (micros() - lastMicrosContrl) * 1e-6f;
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
    Speed = Speed*constrain(distanzCm / 40.0f, 0.0f, 1.0f);

    if(abs(distanzCm)<5){
        Speed=0;
    }

    setHeading(Andrehwinkel*DEG_TO_RAD);
    setOmniWheelDir(Util.AMap(atan2f(dx, dy) * (180.0f / PI)),Speed,giveHeadingController()*100);
}

void systemlib::moveToPoint(uint16_t X, uint16_t Y, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, int16_t Andrehwinkel){

}

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
    Serial.println("omega : "+String(omega));

    float directionRad = directionDeg * DEG_TO_RAD;
    float speedNorm     = speedPercent / 100.0f;

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

    if (posX < 0.0f || posY < 0.0f)
    {
        return -1;
    }

    if (posX < MARGIN_CM || posX > (FIELD_X_CM - MARGIN_CM) ||
        posY < MARGIN_CM || posY > (FIELD_Y_CM - MARGIN_CM))
    {
        return 180;
    }

    return 0;
}

void systemlib::moveDir(int16_t Dir,uint8_t Speed,float Andrehwinkel){
    setHeading(Andrehwinkel*DEG_TO_RAD);
    Dir = Util.AMap(checkWallDistance(Position.X, Position.Y)+Dir);
    setOmniWheelDir(Dir,Speed,giveHeadingController()*100);
}
