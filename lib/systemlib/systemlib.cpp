#include <systemlib.h>

/* -------------- Instanzen -------------- */

BNO055 BNO(BNO_ADDRESS, &I2C_BNO);
// Zwei Busmaster am ESP-Bus (Teensy ueber die Jumper und ESP) stoeren sich: Quellen nicht mischen
#if ((HEADING_SOURCE == SENSOR_SOURCE_TEENSY) || (RADAR_SOURCE == SENSOR_SOURCE_TEENSY) || (IR_SOURCE == SENSOR_SOURCE_TEENSY)) && SENSORS_ANY_ON_ESP && !defined(ALLOW_MIXED_SENSOR_SOURCES)
#warning "Sensorquellen gemischt (HEADING_SOURCE/RADAR_SOURCE/IR_SOURCE): Teensy und ESP waeren am ESP-Bus beide Master. Nur gewollt, wenn die Teensy-Geraete am Teensy-Stecker J13 (Wire1) stecken: dann -DALLOW_MIXED_SENSOR_SOURCES setzen."
#endif

SensorLink EspLink; // Link zum ESP32-S3 (Winkel, Radar, IR-Ring, Tasten)
StreamLinkPort EspLinkPort(UART_ESP);
Pathfinder AStar; // ~130 kB: nur global anlegen. begin() in systemlib::initialize()
// PowerDebugger PDS;

elapsedMillis cycleTimer;
elapsedMillis tempTimer;  // CPU-Temperatur 1x pro Sekunde
elapsedMillis powerTimer; // drosselt die INA219-Abfrage fuer die PDS-Kanaele
elapsedMicros cycleTimer_us;
elapsedMillis lastvaliddata_Radar_X;
elapsedMillis lastvaliddata_Radar_Y;

commlib communication;

#if IR_SOURCE == IR_SOURCE_TEENSY
I2C_bus i2cIR(I2C_IR);
Device_handle ir_ring_handle = {i2cIR, I2C_target{0x0A}};
IR_ring ir_ring(ir_ring_handle);
#endif

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

/* -------------- System Worker -------------- */

void systemlib::initialize()
{
    Wire1.begin();
    Wire1.setClock(I2C_WIRE1_CLOCK_HZ);
#if (HEADING_SOURCE == SENSOR_SOURCE_TEENSY) || (RADAR_SOURCE == SENSOR_SOURCE_TEENSY) || (IR_SOURCE == SENSOR_SOURCE_TEENSY)
    // Nur wenn der Teensy am ESP-Bus (Wire, Pins 18/19) selbst Sensoren liest. Sonst bleibt Wire unberuehrt,
    // damit der Teensy den Bus des ESP nie treibt.
    Wire.begin();
    Wire.setClock(I2C_WIRE_CLOCK_HZ);
#endif

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

    static uint8_t espRxBuffer[LINK_RX_BUFFER_BYTES]; // keine Flusssteuerung: grosser Empfangspuffer
    static uint8_t espTxBuffer[256];
    UART_ESP.addMemoryForRead(espRxBuffer, sizeof(espRxBuffer));
    UART_ESP.addMemoryForWrite(espTxBuffer, sizeof(espTxBuffer));
    UART_ESP.begin(LINK_BAUD);
    EspLink.begin(&EspLinkPort, micros());
#if SENSORS_ANY_ON_ESP
    // Beim Start auf den ESP warten (Handshake), aber nicht ewig: danach laeuft der Roboter ohne dessen Sensoren weiter
    // (kein Winkel: der Kursregler dreht nicht; kein Radar: die Fusion faellt auf die Odometrie zurueck).
    while (!EspLink.giveHandshakeDone() && !EspLink.giveBootTimeout(micros()))
    {
        EspLink.update(micros());
        delay(1);
    }
#endif

#if HEADING_SOURCE == HEADING_SOURCE_DIRECT
    BNO.begin(); // Uebergang: Teensy liest den BNO055 selbst (nur mit Jumpern am ESP-Bus, ESP darf dort nicht senden)
#endif
    AStar.begin();
    Profile.begin();
    Planner.begin(AStar);
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

#if RADAR_SOURCE == RADAR_SOURCE_TEENSY
    Radar.begin(I2C_RADAR); // richtet alle Sensoren ein und startet die erste Messrunde
#endif

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

    EspLink.update(micros());
    updateLinkDiagnostics();
    readBNO();
    readRadar();


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
    PDS.bind("Heading_Alter", &Heading_Age_ms, 31, "ms");
    PDS.bind("Link_Status", &Link_State, 32);
    PDS.bind("ESP_Loop_max", &ESP_MaxLoop_us, 33, "us");
    PDS.bind("ESP_I2C_Fehler", &ESP_I2cErr, 34);
    PDS.bind("Link_CRC", &Link_CrcErr, 35);
    PDS.bind("Link_Luecken", &Link_Lost, 36);
    PDS.bind("Link_Laufzeit", &Link_Delay_us, 37, "us");

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
    constexpr uint32_t latencyMs = (RADAR_SOURCE == RADAR_SOURCE_ESP) ? FUS_RADAR_LATENCY_LINK_MS : FUS_RADAR_LATENCY_MS;
    const uint32_t tMeas = (m.t > latencyMs) ? (m.t - latencyMs) : 0;
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
#if RADAR_SOURCE == RADAR_SOURCE_TEENSY
    Radar.calib();
#else
    EspLink.sendCmd(LinkCmdRadarCalib); // der ESP kalibriert (blockiert dort rund eine Sekunde je Sensor)
#endif
}

void systemlib::readRadar(void)
{
#if RADAR_SOURCE == RADAR_SOURCE_ESP
    // Werte vom ESP: eine Runde ist komplett, wenn der ESP sie schickt. Die Zeit ist die Mitte der Messrunde.
    const uint32_t nowUs = micros();
    RadarMsg rm;
    uint32_t localUs = 0;
    if (EspLink.takeRadar(rm, localUs))
    {
        int32_t ageUs = (int32_t)(nowUs - localUs); // Zeitabgleich ungenau: nie in die Zukunft datieren
        if (ageUs < 0)
            ageUs = 0;
        const uint32_t timeMs = millis() - (uint32_t)ageUs / 1000u;
        for (uint8_t k = 0; k < RADAR_NUM_SENSORS; k++)
            Radar.feedRemote(k, rm.Dist[k], rm.DistRaw[k], rm.Strength[k], (rm.PeakBits >> k) & 1u, (rm.ConBits >> k) & 1u, timeMs);
    }
    if (!EspLink.isUsable(nowUs))
        Radar.markRemoteLost(); // Link ausgefallen: Werte altern, die Fusion faellt auf die Odometrie zurueck
#else
    Radar.update();
#endif
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
#if HEADING_SOURCE == HEADING_SOURCE_LINK
    // Der Winkel kommt vom ESP: gilt ab der naechsten gueltigen Messung, falls jetzt keine vorliegt
    _headingCalibPending = true;
    if (_headingValid)
    {
        BNO_calibration = BNO_eulerZ;
        _headingCalibPending = false;
    }
#else
    BNO_calibration = BNO_eulerZ;
#endif
}

void systemlib::readBNO(void)
{
#if HEADING_SOURCE == HEADING_SOURCE_LINK
    const uint32_t nowUs = micros();
    _headingValid = EspLink.giveHeadingValid(nowUs);
    Heading_Age_ms = EspLink.giveHeadingAgeUs(nowUs) * 1e-3f;
    if (_headingValid)
    {
        BNO_eulerZ = EspLink.giveHeading(nowUs); // 0 ... 360, mit der Drehrate auf jetzt weitergerechnet
        if (_headingCalibPending)
        {
            BNO_calibration = BNO_eulerZ;
            _headingCalibPending = false;
        }
        BNO_OrbitDir = Util.wrapAngle(BNO_eulerZ - BNO_calibration);
    }
#else
    BNO_eulerZ = BNO.eulHeading();
    BNO_OrbitDir = Util.wrapAngle(BNO_eulerZ - BNO_calibration);
    _headingValid = true;
#endif
}

/* -------------- IR -------------- */
void systemlib::readIR(void)
{
#if IR_SOURCE == IR_SOURCE_ESP
    // Die Werte kommen vom ESP; der Aufruf kostet keine Buszeit und darf jeden Zyklus laufen.
    IrMsg im;
    uint32_t localUs = 0;
    if (EspLink.takeIr(im, localUs))
    {
        IR_rawAngle = (float)im.AngleRaw / 180.0f; // wie IR_ring::read_ball_angle()
        IR_rawDistance = im.Dist;
    }
#else
    IR_rawAngle = ir_ring.read_ball_angle();
    IR_rawDistance = ir_ring.read_ball_distance();

#if false
    ir_ring.read_raw_values(IR_rawSensorData);
    ir_ring.read_tssp(IR_TSSP);
#endif
#endif

    IR_Angle = Util.wrapAngle(IR_rawAngle - BNO_OrbitDir);
    IR_Distance = IR_rawDistance; // Hier muss noch Mathe hin
}

// Kalibrierung des IR-Rings: nur mit dem Ring am Teensy (IR_SOURCE_TEENSY). Am ESP laeuft sie ueber dessen Firmware.
void systemlib::writeIRCalibration(uint8_t sensorNumber, int16_t offset, int16_t gain)
{
#if IR_SOURCE == IR_SOURCE_TEENSY
    ir_ring.write_offset(sensorNumber, offset);
    ir_ring.write_gain(sensorNumber, gain);
#else
    (void)sensorNumber;
    (void)offset;
    (void)gain;
#endif
}

void systemlib::writeIRCalibration(const int16_t (&offsets)[16], const uint16_t (&gains)[16])
{
#if IR_SOURCE == IR_SOURCE_TEENSY
    ir_ring.write_offsets(offsets);
    ir_ring.write_gains(gains);
#else
    (void)offsets;
    (void)gains;
#endif
}

void systemlib::calibIR(void)
{
#if IR_SOURCE == IR_SOURCE_TEENSY
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
#endif
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

SensorLink &systemlib::giveEspLink(void)
{
    return EspLink;
}

LinkState systemlib::giveLinkState(void) const
{
    const uint32_t nowUs = micros();
    if (EspLink.giveParamsMismatch())
        return LinkStateMismatch;
#if !SENSORS_ANY_ON_ESP
    if (!EspLink.giveHandshakeDone())
        return LinkStateNotUsed;
#endif
    return EspLink.isUsable(nowUs) ? LinkStateOk : LinkStateNoEsp;
}

bool systemlib::giveSwitch2(void) const
{
    return EspLink.giveSwitch2();
}

void systemlib::updateLinkDiagnostics(void)
{
    Link_State = (float)giveLinkState();
    const HealthMsg &hl = EspLink.giveHealth();
    ESP_MaxLoop_us = hl.MaxLoopUs;
    ESP_I2cErr = hl.I2cErrors;
    Link_CrcErr = (float)EspLink.giveCrcErrors();
    Link_Lost = (float)EspLink.giveLostFrames();
    Link_Delay_us = (float)EspLink.giveClockDelayUs();
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
    digitalWrite(BLDC_DO1, !MoDataDR.Dir); // Dribbler-Richtung (ESCON DI2), wie bei den Motoren: 0 bei Dir = true

    // Ausgabebytes der Expander (Bit n = Pin Pn). Eingangspins (SWITCH1, BUTTON1) bleiben auf 1, ihr Wert ist ohne Wirkung.
    // Richtung (DI2) 0 bei Dir = true, Enable (DIO3) 1 bei Enable, Bremse (DIO4) 1 bei Break.
    uint8_t outA = 0xFF;
    uint8_t outB = 0xFF;
    setBit(outA, MotorVL_DIR_BIT, !MoDataVL.Dir);
    setBit(outA, MotorVL_EN_BIT, MoDataVL.Enable);
    setBit(outA, MotorVL_BRK_BIT, MoDataVL.Break);
    setBit(outA, MotorHL_DIR_BIT, !MoDataHL.Dir);
    setBit(outA, MotorHL_EN_BIT, MoDataHL.Enable);
    setBit(outA, MotorHL_BRK_BIT, MoDataHL.Break);
    setBit(outA, MotorDR_EN_BIT, MoDataDR.Enable);
    setBit(outA, MotorDR_BRK_BIT, MoDataDR.Break);
    setBit(outB, MotorVR_DIR_BIT, !MoDataVR.Dir);
    setBit(outB, MotorVR_EN_BIT, MoDataVR.Enable);
    setBit(outB, MotorVR_BRK_BIT, MoDataVR.Break);
    setBit(outB, MotorHR_DIR_BIT, !MoDataHR.Dir);
    setBit(outB, MotorHR_EN_BIT, MoDataHR.Enable);
    setBit(outB, MotorHR_BRK_BIT, MoDataHR.Break);
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

    if (!_headingValid)
    {
        // Kein gueltiger Winkel (Link ausgefallen): nicht drehen, Regler auf den letzten Winkel setzen
        ADRC_Out = 0;
        PID_Out = 0;
        resetHeadingController();
        return;
    }

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

/* ========================================================================
 * Fahren: Adapter zu MotionProfile und RoutePlanner
 *
 * Die Fahrlogik liegt in lib/MotionProfile (Speed-Pattern, Zielanfahrt,
 * Punkt-Liste) und lib/RoutePlanner (A* auf cm-Ebene). Beide kennen weder
 * Sensoren noch Motoren: systemlib gibt ihnen Position, Winkel und Zeit und
 * setzt den zurueckgegebenen Fahrbefehl ueber setOmniWheelDir() um.
 *
 * Der Kursregler bleibt hier -- er ist Roboter-Wissen (HDG_*), keine Geometrie.
 * ===================================================================== */

/// Position, Winkel und Zeit dieses Zyklus, so wie MotionProfile sie braucht.
MotionInput systemlib::giveMotionInput(void) const
{
    MotionInput in;
    in.NowMs = millis();
    in.NowUs = micros();
    in.X = Position.X;
    in.Y = Position.Y;
    in.PosValid = (Position.X >= 0.0f && Position.Y >= 0.0f);
    in.HeadingDeg = giveOrbitDir();
    return in;
}

/// Fahrbefehl ausfuehren: Sollwinkel an den Kursregler, Richtung und Tempo an die Raeder.
void systemlib::applyMotionCommand(const MotionCommand &cmd)
{
    if (!cmd.Active)
    {
        return; // Punkt-Liste ist Idle: die Motoren gehoeren der uebrigen Logik
    }

    setHeading(cmd.Andrehwinkel * DEG_TO_RAD);
    setOmniWheelDir(cmd.DirectionDeg, cmd.SpeedPct, giveHeadingController() * 100);
}

void systemlib::moveToPoint(uint16_t X, uint16_t Y, uint8_t Speed, int16_t Andrehwinkel)
{
    applyMotionCommand(Profile.moveToPointSimple(giveMotionInput(), X, Y, Speed, Andrehwinkel));
}

void systemlib::moveToPoint(uint16_t X, uint16_t Y, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, int16_t Andrehwinkel)
{
    applyMotionCommand(Profile.moveToPoint(giveMotionInput(), X, Y, SpeedPattern, MaxSpeed, MinSpeed, Andrehwinkel));
}

void systemlib::resetMoveProfile(void) { Profile.resetProfile(); }

bool systemlib::giveMoveArrived(void) const { return Profile.giveArrived(); }

float systemlib::giveMoveProgress(void) const { return Profile.giveProgress(); }

/* -------------- Punkt-Liste -------------- */

bool systemlib::setPointArray(const ArrayPoint *points, uint8_t count, bool loop)
{
    return Profile.setPointArray(points, count, loop);
}

bool systemlib::addPointToArray(uint16_t X, uint16_t Y, int16_t Andrehwinkel, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, uint16_t HoldMs)
{
    return Profile.addPointToArray(X, Y, Andrehwinkel, SpeedPattern, MaxSpeed, MinSpeed, HoldMs);
}

bool systemlib::addPointToArray(const ArrayPoint &point) { return Profile.addPointToArray(point); }

void systemlib::resetArray(void) { Profile.resetArray(); }

void systemlib::startArray(bool restart) { Profile.startArray(restart); }

void systemlib::pauseArray(void) { Profile.pauseArray(); }

void systemlib::resumeArray(void) { Profile.resumeArray(); }

void systemlib::updateArray(void) { applyMotionCommand(Profile.updateArray(giveMotionInput())); }

ArrayState systemlib::giveArrayState(void) const { return Profile.giveArrayState(); }
uint8_t systemlib::giveArrayIndex(void) const { return Profile.giveArrayIndex(); }
uint8_t systemlib::giveArrayCount(void) const { return Profile.giveArrayCount(); }

/* -------------- Pfadplanung -------------- */

/**
 * Hindernisse fuer die Pfadplanung [cm, Feldkoordinaten]. -1 = kein Hindernis.
 * Zyklisch aufrufbar, vor moveToPointAvoid(). Ball = Ball, Roboter = Partner (oder Gegner).
 */
void systemlib::setPathObstacles(int16_t ballX, int16_t ballY, int16_t robotX, int16_t robotY)
{
    Planner.setObstacles(ballX, ballY, robotX, robotY);
}

/**
 * Wie moveToPoint(), aber um Hindernisse herum. Zyklisch aufrufen.
 * Alle Parameter wie bei moveToPoint(); das Speed-Pattern gilt fuer die ganze Strecke.
 *
 * Kein Weg gefunden: PATH_FALLBACK_DIRECT = true faehrt geradeaus per moveToPoint(),
 * sonst bleibt der Roboter stehen und haelt den Winkel. Status: givePathFound().
 *
 * ACHTUNG: Solange moveToPointAvoid() benutzt wird, gehoert ihm die Punkt-Liste.
 */
void systemlib::moveToPointAvoid(uint16_t X, uint16_t Y, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, int16_t Andrehwinkel)
{
    const MotionInput in = giveMotionInput();

    // Position unbekannt: moveToPoint() haelt an und regelt den Winkel
    if (!in.PosValid)
    {
        applyMotionCommand(Profile.moveToPoint(in, X, Y, SpeedPattern, MaxSpeed, MinSpeed, Andrehwinkel));
        return;
    }

    RouteRequest req;
    req.NowMs = in.NowMs;
    req.FromX = in.X;
    req.FromY = in.Y;
    req.ToX = X;
    req.ToY = Y;
    req.Arrived = Profile.giveArrived(); // am Ziel: nicht dauernd neu planen

    Route route;
    switch (Planner.update(req, route))
    {
    case RouteNew:
        // Tempo je Teilstueck rechnet MotionProfile; unveraenderte Wegpunkte starten nicht neu
        Profile.setRoute(route, SpeedPattern, MaxSpeed, MinSpeed, Andrehwinkel);
        break;

    case RouteFailed:
        Profile.resetArray();
        break;

    default: // RouteUnchanged: die bisherige Route gilt weiter
        break;
    }

    if (Planner.giveFound())
    {
        Profile.setArrayHeading(Andrehwinkel); // Winkel darf sich ohne Neuplanung aendern
        Profile.startArray();
        applyMotionCommand(Profile.updateArray(in));
    }
    else if (PATH_FALLBACK_DIRECT)
    {
        applyMotionCommand(Profile.moveToPoint(in, X, Y, SpeedPattern, MaxSpeed, MinSpeed, Andrehwinkel));
    }
    else
    {
        setHeading(Andrehwinkel * DEG_TO_RAD);
        setOmniWheelDir(0, 0, giveHeadingController() * 100);
    }
}

/**
 * Verwirft Planung, Punkt-Liste und Fahrprofil (Speed 0). Aufrufen beim Wechsel in einen
 * anderen Modus, damit die naechste Fahrt ohne Reste der vorigen beginnt.
 */
void systemlib::resetPathPlan(void)
{
    Planner.reset();
    Profile.resetArray();
    Profile.stopProfile();
}

bool systemlib::givePathFound(void) const { return Planner.giveFound(); }
uint8_t systemlib::givePathWaypoints(void) const { return Planner.giveWaypoints(); }
uint32_t systemlib::givePathTimeUs(void) const { return Planner.giveTimeUs(); }

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

/**
 * Freie Fahrt in eine Feldrichtung, mit Wandschutz.
 *
 * Steht der Roboter naeher als MOVE_WALL_MARGIN_CM an einer Wand, faehrt er nur
 * noch mit MOVE_WALL_SPEED_PCT weiter. Bei unbekannter Position greift der
 * Schutz nicht, dann ist der Wandabstand nicht bekannt.
 */
void systemlib::moveDir(int16_t Dir, uint8_t Speed, float Andrehwinkel)
{
    setHeading(Andrehwinkel * DEG_TO_RAD);

    if (MotionProfile::isNearWall(Position.X, Position.Y))
    {
        Speed = MOVE_WALL_SPEED_PCT;
    }

    setOmniWheelDir(Dir, Speed, giveHeadingController() * 100);
}
