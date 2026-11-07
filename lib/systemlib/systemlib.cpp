#include <systemlib.h>
#include <comProtokoll.h>
#include <Cam.h>

// A*-Pathfinder (lib/pathfinder). Fehlt die Bibliothek, faehrt moveToPointAvoid() geradeaus.
#ifndef PF_AVAILABLE // kann auch per build_flags gesetzt werden: -DPF_AVAILABLE=1 / 0
#if __has_include(<pathfinder.h>)
#define PF_AVAILABLE 1
#else
#define PF_AVAILABLE 0
#endif
#endif
#if PF_AVAILABLE
#include <pathfinder.h>
#endif

/* -------------- Instanzen -------------- */

DataSync datasync;
Timer timer;

I2C_bus i2cIR(I2C_IR);
Device_handle ir_ring_handle = {i2cIR, I2C_target{IR_ADDRESS}};
IR_ring ir_ring(ir_ring_handle);

INA219 INA219_Main(INA_MAIN_ADDRESS);
INA219 INA219_Motor(INA_MOTOR_ADDRESS);

TCA9534DW SW_EXP(&I2C_SW, SW_SWITCH_ADDRESS);
TCA9534DW Button_EXP(&I2C_SW, SW_BUTTON_ADDRESS);
TCA9534DW Motor12_EXP(&I2C_MAIN, EXP_MO12_ADDRESS); // IC9: ESCON-Bits 0..7
TCA9534DW Motor34_EXP(&I2C_MAIN, EXP_MO34_ADDRESS); // IC7: ESCON-Bits 8..15
TCA9534DW Power_EXP(&I2C_MAIN, EXP_POWER_ADDRESS);  // IC8

// IC8 im Betrieb, wie CheckSystems::hardwareInit(): VMOT, VDRIB, Lichtschranke
// und Linien-LEDs ein; I2C_INTERCON_3V3_EN bleibt LOW (= Intercon versorgt).
constexpr uint8_t POWER_EXP_RUN = (1 << VDRIB_EN) | (1 << VMOT_EN) | (1 << LIGHT_BARRIER_EN) | (1 << LINE_EN);

ADC108S102 ADC_Motor(&SPI, ADC108S102_Motor_CS);
ADC108S102 ADC_Line1(&SPI, SPI2_DEC_A, SPI2_DEC_B, SPI2_DEC_C, 0);
ADC108S102 ADC_Line2(&SPI, SPI2_DEC_A, SPI2_DEC_B, SPI2_DEC_C, 1);
ADC108S102 ADC_Line3(&SPI, SPI2_DEC_A, SPI2_DEC_B, SPI2_DEC_C, 2);
ADC108S102 ADC_Line4(&SPI, SPI2_DEC_A, SPI2_DEC_B, SPI2_DEC_C, 3);
ADC108S102 ADC_Line5(&SPI, SPI2_DEC_A, SPI2_DEC_B, SPI2_DEC_C, 4);
ADC108S102 ADC_Line6(&SPI, SPI2_DEC_A, SPI2_DEC_B, SPI2_DEC_C, 5);

constexpr float TS_NOMINAL = 0.005f; // nur Fallback fuer cfg.Ts, siehe loop()
PID *headingControllerP;
ADRC *headingControllerA;
uint32_t lastMicrosContrl = 0;

fancyBTN BTN1(50, 4, 8, false);
fancyBTN BTN2(50, 4, 8, false);
fancyBTN BTN3(50, 4, 8, false);
fancyBTN BTN4(50, 4, 8, false);
fancyBTN BTN5(50, 4, 8, false);

CamC Cam1;
CamC Cam2;

/* -------------- System Worker -------------- */

void systemlib::initialize()
{
    // Sollwerte zuerst auf 0, bevor VMOT/VDRIB freigegeben werden
    const uint8_t pwmPins[5] = {MotorVL_PWM, MotorVR_PWM, MotorHL_PWM, MotorHR_PWM, MotorDR_PWM};
    for (uint8_t i = 0; i < 5; i++)
    {
        pinMode(pwmPins[i], OUTPUT);
        analogWrite(pwmPins[i], 0);
    }
    pinMode(KICKER_PORT, OUTPUT);
    digitalWrite(KICKER_PORT, LOW);
    pinMode(SPI_TNS_CS, OUTPUT); // Bottom-Platine waehlt ueber den Decoder, CS bleibt inaktiv
    digitalWrite(SPI_TNS_CS, HIGH);
    pinMode(ROBO_ID, INPUT);

    Wire1.begin();
    Wire1.setClock(1000000);

// Ausgangsregister VOR der Richtung setzen: nach dem Einschalten steht es
// auf 0xFF, ein setup(false) als erstes zoege alle ESCON-Freigaben auf HIGH.
#if SYS_EN_MOTORS
    Motor12_EXP.write(0x00);
    Motor12_EXP.setup(false);
    Motor34_EXP.write(0x00);
    Motor34_EXP.setup(false, false, false, false, false, false, true, true); // P6/P7 = SWITCH1/BUTTON1
#endif
    Power_EXP.write(POWER_EXP_RUN);
    Power_EXP.setup(false);
    enableVMOT();
    enableVDRIB();
    disableLIGHTBARRIER();
    enableLINE();
    enableI2C();
#if SYS_EN_SWITCHBOARD
    SW_EXP.setup(true);
    Button_EXP.setup(true);
#endif
    readExpanders();

    SPI.begin();

    USBC_UART.begin(115200);

#if SYS_EN_POWER
    INA219_Main.begin(Wire1);
    INA219_Motor.begin(Wire1);

    INA219_Main.setMaxCurrentShunt(15);
    INA219_Motor.setMaxCurrentShunt(15);
#endif

#if DPDS_Game == D_Mode_Test
    PDS.init();
    setupPDS();
#endif

    MoDataVL.ID = MotorVL_ID;
    MoDataVR.ID = MotorVR_ID;
    MoDataHL.ID = MotorHL_ID;
    MoDataHR.ID = MotorHR_ID;
    MoDataDR.ID = MotorDR_ID;

    // ir_ring.read_gains(IR_Gains);
    // ir_ring.read_offsets(IR_Offsets);

    setDebugTimeout(0);

#if SYS_EN_MOTORS
    ADC_Motor.init();
#endif
#if SYS_EN_LINE
    ADC_Line1.init();
    ADC_Line2.init();
    ADC_Line3.init();
    ADC_Line4.init();
    ADC_Line5.init();
    ADC_Line6.init();
#endif

    setupHeadingController();

    datasync.begin();

    Cam1.init(UART_CAM1);
    Cam1.setSign(false);
    Cam2.init(UART_CAM2);
    Cam2.setSign(false);
}

int16_t systemlib::giveOrbitDir()
{
    return BNO_OrbitDir;
}

void systemlib::readSensors(void)
{
    // Zyklische Messung der Laufzeit in ms (read -> read)
    timer.cycle = 0;
    timer.cycle_us = 0;

    delay(debugTimeout);
    

    Cam1.Update(BNO_OrbitDir); // <1µs
    Cam2.Update(BNO_OrbitDir); // <1µs

    

    readIR(); // 260µs

    updateArray(); // <1µs

    
    updateLine(); // 250µs

    timer.cycle = 0;
    timer.cycle_us = 0;

    readExpanders(); //200µs

    

    datasync.update(); // 2-4µs

    

    Position.X = datasync.PositionX;
    Position.Y = datasync.PositionY;
    Position.XValid = datasync.Position_XVal;
    Position.YValid = datasync.Position_YVal;
    Position.AtGoal = datasync.Position_AtGoal;
    Position.Certainty = datasync.Position_Cert;

    datasync.Motor_SpeedVL = MoDataVL.Speed_real;
    datasync.Motor_SpeedVR = MoDataVR.Speed_real;
    datasync.Motor_SpeedHL = MoDataHL.Speed_real;
    datasync.Motor_SpeedHR = MoDataHR.Speed_real;

    BNO_OrbitDir = datasync.BNO_Heading;

    #if DPDS_Game == D_Mode_Test
    // Werte fuer die PDS-Kanaele nachfuehren. Die INA219 haengen am I2C und
    // kosten je Abfrage spuerbar Zeit -- Akkuspannung aendert sich nicht im
    // 100-Hz-Takt, 10 Hz reichen voellig.
    uptime_s = millis() / 1000.0f;
    if ((timer.power >= 1000))
    {
        timer.cycle = 0;
        timer.cycle_us = 0;
        timer.power = 0;
        Akku_V = giveMainVoltage();
        Akku_I = giveMainCurrent();
        Motor_V = giveMotorVoltage();
        Motor_I = giveMotorCurrent();
    }

    PDS.update();
    #endif

    updateHeadingController();

    
    updateMotors(); // 70-160µs
    

    if ((timer.BNOCalib > 200))
    {
        datasync.BNO_Calib = false;
    }

    cycleTime = timer.cycle;
    cycleTime_us = timer.cycle_us;
}

float systemlib::giveCam2Angle()
{
    return Cam2.give_Angle();
}

float systemlib::giveCam1Angle()
{
    return Cam1.give_Angle();
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

    /* ---- Ball (IR) ----------------------------------------------------- */
    // readIR() ist in readSensors() derzeit auskommentiert -- die Kanaele
    // stehen aber schon bereit und zeigen bis dahin 0.
    PDS.bind("Ball_Winkel", &IR_Angle, 8, "°");
    PDS.bind("Ball_Dist", &IR_Distance, 11);

    /* ---- Zykluszeit ---------------------------------------------------- */
    PDS.bind("Zykluszeit", &cycleTime, 9, "ms");
    PDS.bind("Zykluszeit_us", &cycleTime_us, 10, "us");

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

    // 26..31 bleiben frei fuer weitere Systemwerte.

#endif // Game_mode
}

/* -------------- ADC/Line -------------- */

uint16_t systemlib::giveADC(uint8_t Port, uint8_t Pin)
{
#if !SYS_EN_LINE
    return 0;
#endif
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
    uint16_t line_depth = 0;
    int8_t line_Angle = 0;
    bool line_touching;

// Ohne Linienplatinen bleibt line_array auf 0 -> keine Linie erkannt
#if SYS_EN_LINE
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
#endif

    for (int i = 0; i < 48; i++)
    {
        line_bool_array[i] = line_array[i] > LineSensorSchwelle[i];
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
                aktiveSensoren[count] = Util.AMap((i * 7.5));
                count++;
            }
        }

        for (int i = 0; i < line_summe; i++)
        {
            for (int j = i + 1; j < line_summe; j++)
            {
                float dist = fabsf(Util.AMap(aktiveSensoren[i] - aktiveSensoren[j]));
                if (dist > maxDistanz)
                {
                    maxDistanz = dist;
                    sensorA = aktiveSensoren[i];
                    sensorB = aktiveSensoren[j];
                }
            }
        }

        // 3) Mittelwinkel zwischen A und B bestimmen (2 Möglichkeiten)
        float delta = Util.AMap(sensorB - sensorA);
        line_depth = 1 - abs(sin((abs(delta) / 2) * DEG_TO_RAD));
        float mitte1 = Util.AMap(sensorA + (delta * 0.5f));
        float mitte2 = Util.AMap(mitte1 + 180.0f);

        // 4) Mitte wählen, die näher liegt
        line_Angle = (fabsf(Util.AMap(mitte1 - mitte2))) < (fabsf(Util.AMap(mitte2 - sensorA))) ? mitte1 : mitte2;
        line_touching = true;
    }
    else if (line_summe == 1)
    {
        line_depth = 1;
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
        line_depth = 1;
        line_touching = false;
    }

    LineDepth = line_array[0];
    LineWinkel = line_Angle;
    LinePresent = line_touching;
}

// Ball Modell: y = a1*exp(-k1*x) + a2*exp(-k2*x)
float yVonX(float x) {
  return 467.19 * exp(-0.0136558 * x) + 833.60 * exp(-0.00260057 * x);
}

// Ball Umkehrung: gibt zu einem y-Wert das passende x zurück
float xAusY(float y) {
    if(y>53){
        float lo = 0;
        float hi = 4000;

        if (y >= yVonX(lo)) return lo;   // über dem Startwert (ca. 1301)
        if (y <= yVonX(hi)) return hi;   // praktisch 0

        for (int i = 0; i < 30; i++) {   // 30 Halbierungen reichen locker
            float mid = (lo + hi) / 2;
            if (yVonX(mid) > y) lo = mid;
            else hi = mid;
        }
        return (lo + hi) / 2;
    }
    else if(y>1){
        return logf(164.14f / y) * 943.4f;
    }
    else{
        return 0;
    }
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
    
    if(IR_rawDistance > 25){
        IR_Angle = Util.AMap(IR_rawAngle + BNO_OrbitDir);
        IR_Distance = xAusY(IR_rawDistance-25);
    }
    else{
        IR_Distance = 0;
    }    

    Ball_Position.X = (IR_Distance * cosf(DEG_TO_RAD*IR_Angle)) + Position.X;
    Ball_Position.Y = (IR_Distance * sinf(DEG_TO_RAD*IR_Angle)) + Position.Y;
    if(IR_Distance>1){
        Ball_Position.Certainty = Position.Certainty;
        Ball_Position.XValid = Position.XValid;
        Ball_Position.YValid = Position.YValid;
    }
    else{
        Ball_Position.Certainty = 0;
        Ball_Position.XValid = 0;
        Ball_Position.YValid = 0;
    }
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

void systemlib::calibBNO(void)
{
    datasync.BNO_Calib = true;
    timer.BNOCalib = 0;
}

/* -------------- Aktor --------------*/

float systemlib::giveMainVoltage(void)
{
#if !SYS_EN_POWER
    return 0.0f;
#endif
    return (float)INA219_Main.getBusVoltage();
}

float systemlib::giveMotorVoltage(void)
{
#if !SYS_EN_POWER
    return 0.0f;
#endif
    return (float)INA219_Motor.getBusVoltage();
}

float systemlib::giveMainCurrent(void)
{
#if !SYS_EN_POWER
    return 0.0f;
#endif
    return (float)INA219_Main.getCurrent_mA();
}

float systemlib::giveMotorCurrent(void)
{
#if !SYS_EN_POWER
    return 0.0f;
#endif
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

/// Freigabe- und Richtungsbit eines ESCON im Expanderwort setzen (Bitnummern aus enum.h).
static void setEsconBits(uint16_t &out, const MotorData &m, uint8_t enBit, uint8_t dirBit)
{
    if (m.Enable)
        out |= (1u << enBit);
    if (!m.Dir)
        out |= (1u << dirBit);
}

int motorSampleUs = 0;

void systemlib::updateMotors(void)
{
// Ohne Motorplatine bleiben PWM (in initialize() auf 0 gesetzt) und
// Speed_real/Current auf 0 -- MoData* koennen trotzdem beschrieben werden.
#if !SYS_EN_MOTORS
    return;
#endif
    uint16_t out = 0;
    if (MoDataVL.Speed == 0 || giveExpander(0, 3))
    {
        MoDataVL.Speed = 0;
        MoDataVL.Enable = false;
        MoDataVL.Break = true;
    }
    if (MoDataVR.Speed == 0 || giveExpander(0, 3))
    {
        MoDataVR.Speed = 0;
        MoDataVR.Enable = false;
        MoDataVR.Break = true;
    }
    if (MoDataHL.Speed == 0 || giveExpander(0, 3))
    {
        MoDataHL.Speed = 0;
        MoDataHL.Enable = false;
        MoDataHL.Break = true;
    }
    if (MoDataHR.Speed == 0 || giveExpander(0, 3))
    {
        MoDataHR.Speed = 0;
        MoDataHR.Enable = false;
        MoDataHR.Break = true;
    }
    if (MoDataDR.Speed == 0 || giveExpander(0, 3))
    {
        MoDataDR.Speed = 0;
        MoDataDR.Enable = false;
        MoDataDR.Break = true;
    }

    analogWrite(MotorVL_PWM, abs(MoDataVL.Speed) * 2.55);
    analogWrite(MotorVR_PWM, abs(MoDataVR.Speed) * 2.55);
    analogWrite(MotorHR_PWM, abs(MoDataHR.Speed) * 2.55);
    analogWrite(MotorHL_PWM, abs(MoDataHL.Speed) * 2.55);
    analogWrite(MotorDR_PWM, abs(MoDataDR.Speed) * 2.55);

    // ESCON-Steuerleitungen: Bit 0..7 -> IC9, Bit 8..15 -> IC7 (siehe enum.h).
    // Dir true -> Richtungsleitung LOW, Break -> Stopp-Leitung HIGH.
    setEsconBits(out, MoDataVL, MotorVL_EN, MotorVL_DIR);
    setEsconBits(out, MoDataVR, MotorVR_EN, MotorVR_DIR);
    setEsconBits(out, MoDataHL, MotorHL_EN, MotorHL_DIR);
    setEsconBits(out, MoDataHR, MotorHR_EN, MotorHR_DIR);
    setEsconBits(out, MoDataDR, MotorDR_EN, MotorDR_DIR);

    if (MoDataVL.Break)
        out |= (1u << MotorVL_STOP);
    if (MoDataVR.Break)
        out |= (1u << MotorVR_STOP);
    if (MoDataHL.Break)
        out |= (1u << MotorHL_STOP);
    if (MoDataHR.Break)
        out |= (1u << MotorHR_STOP);

    Motor12_EXP.write(out & 0xFF);
    Motor34_EXP.write(out >> 8);

    /*for(int i = 0 ; i<8 ; i++){
        Serial.print("channel ");
        Serial.print(i);
        Serial.print(" : ");
        Serial.println(ADC_Motor.read(i));
    }

    Serial.println(ADC_Motor.read(0)-ADC_Motor.read(1));
    */

    motorSampleUs = micros(); // Zeitstempel der Drehzahlmessung (fuer Totzeitausgleich in calculatePosition)
    MoDataVL.Speed_real = ADC_Motor.read(MotorVL_RPM, true, true);
    MoDataVL.Current = ADC_Motor.read(MotorVL_CUR, true, false);
    MoDataVR.Speed_real = ADC_Motor.read(MotorVR_RPM, true, true);
    MoDataVR.Current = ADC_Motor.read(MotorVR_CUR, true, false);
    MoDataHL.Speed_real = ADC_Motor.read(MotorHL_RPM, true, true);
    MoDataHL.Current = ADC_Motor.read(MotorHL_CUR, true, false);
    MoDataHR.Speed_real = ADC_Motor.read(MotorHR_RPM, true, true);
    MoDataHR.Current = ADC_Motor.read(MotorHR_CUR, true, false);
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
void systemlib::enableVMOT(bool enable)
{
    Power_EXP.writePin(VMOT_EN, enable);
}

void systemlib::enableVDRIB(bool enable)
{
    Power_EXP.writePin(VDRIB_EN, enable);
}

void systemlib::enableLIGHTBARRIER(bool enable)
{
    Power_EXP.writePin(LIGHT_BARRIER_EN, !enable); // Pulldown um zu aktivieren
}

void systemlib::enableLINE(bool enable)
{
    Power_EXP.writePin(LINE_EN, enable);
}

void systemlib::enableI2C(bool enable)
{
    Power_EXP.writePin(I2C_INTERCON_3V3_EN, !enable); // Pulldown um zu aktivieren
}

void systemlib::readExpanders(void)
{
// Ohne SwitchBoard bleiben Schalter, Start, RCJ und Taster auf false
#if !SYS_EN_SWITCHBOARD
    return;
#endif
    // Je Expander ein Buszugriff, danach nur noch aus dem gelesenen Byte
    // (readPin() wuerde jedes Mal neu ueber I2C lesen).
    SW_EXP.read();     // SwitchBoard IC1: P0 SW5 ... P4 SW1, P5 Start, P6 RCJ-Disable, P7 RCJ erkannt
    Button_EXP.read(); // SwitchBoard IC2: P0 BTN1 ... P4 BTN5, P5 RCJ-Out1, P6 RCJ-Out2, P7 PX2_DIO

    SWI[0] = SW_EXP.getPinState(4);
    SWI[1] = SW_EXP.getPinState(3);
    SWI[2] = SW_EXP.getPinState(2);
    SWI[3] = SW_EXP.getPinState(1);
    SWI[4] = SW_EXP.getPinState(0);

    startSwitch = SW_EXP.getPinState(5);
    RCJ = Button_EXP.getPinState(5);
    RCJ_Disable = SW_EXP.getPinState(6);
    RCJ_present = SW_EXP.getPinState(7);

    BTN1.update(Button_EXP.getPinState(0));
    BTN2.update(Button_EXP.getPinState(1));
    BTN3.update(Button_EXP.getPinState(2));
    BTN4.update(Button_EXP.getPinState(3));
    BTN5.update(Button_EXP.getPinState(4));

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
#if !SYS_EN_SWITCHBOARD
    return false;
#endif
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
        ADRC_Out = headingControllerA->update(_OrbitHeading, DEG_TO_RAD * BNO_OrbitDir, dt);
    }
    else
    {
        PID_Out = headingControllerP->update(_OrbitHeading, DEG_TO_RAD * BNO_OrbitDir, dt);
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
    setOmniWheelDir(Util.AMap(atan2f(-dx, dy) * (180.0f / PI)), Speed, giveHeadingController() * 100);
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

    const float tx = constrain((float)X, 0.0f, feld_x);
    const float ty = constrain((float)Y, 0.0f, feld_y);
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
    const float headingErr = fabsf(Util.AMap(p.Andrehwinkel - BNO_OrbitDir));
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
 * Verknuepft den A*-Pathfinder (lib/pathfinder) mit dem Punkt-Array:
 *
 *   Position, Ziel, Ball, Partner  ->  pathfinder::findPath()  (Raster 90 x 120)
 *     ->  Zellpfad  ->  Linie-fuer-Linie vereinfachen (wenige Wegpunkte, in cm)
 *     ->  Wegpunkte als ArrayPoints  ->  setPointArray() / updateArray()
 *
 * Das Speed-Pattern gilt fuer die GANZE Strecke: Zwischenpunkte bekommen je
 * ein konstantes Tempo, das aus dem Pattern an der Stelle des Teilstuecks
 * berechnet wird; sie werden ohne Halt durchfahren (PassCm). Das letzte
 * Teilstueck faehrt das Pattern selbst, inkl. Zielkreis und Andrehwinkel.
 *
 * Neu geplant wird bei: neuem Ziel / Pattern / Speed, bewegtem Hindernis
 * (> PF_OBST_MOVE_CM) und zyklisch alle PF_REPLAN_MS (nur wenn noch unterwegs).
 *
 * Ohne die Bibliothek (kein <pathfinder.h>) faehrt moveToPointAvoid() einfach
 * geradeaus per moveToPoint().
 *
 * ACHTUNG: Waehrend moveToPointAvoid() benutzt wird, gehoert ihm das Punkt-Array.
 * ===================================================================== */
#ifndef PF_REPLAN_MS
#define PF_REPLAN_MS 300 // Neuplanung alle ... ms, solange der Roboter unterwegs ist
#endif
#ifndef PF_REPLAN_FAIL_MS
#define PF_REPLAN_FAIL_MS 1000 // nach einer gescheiterten Planung seltener erneut versuchen
#endif
#ifndef PF_OBST_MOVE_CM
#define PF_OBST_MOVE_CM 8.0f // Hindernis hat sich so weit bewegt -> sofort neu planen
#endif
#ifndef PF_PASS_CM
#define PF_PASS_CM 12 // Zwischenpunkte gelten ab diesem Abstand als durchfahren
#endif
#ifndef PF_LOS_MAX_CELLS
#define PF_LOS_MAX_CELLS 80 // so weit (in Zellen) wird beim Vereinfachen vorausgeschaut
#endif
#ifndef PF_FALLBACK_DIRECT
#define PF_FALLBACK_DIRECT 1 // 1 = ohne Weg geradeaus fahren, 0 = stehen bleiben
#endif

#if PF_AVAILABLE

// Gewichtskarte (weights.cpp): enthaelt nach findPath() auch die Masken von Ball und Partner
extern uint8_t field[pathfinder::kFieldWidth][pathfinder::kFieldHeight];

namespace
{
    constexpr uint8_t PF_NO_OBSTACLE = 255; // Zelle ausserhalb des Rasters: Maske wird nicht gesetzt

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
        uint8_t path[pathfinder::kPathMaxLength][2];
    };
    PathPlan pf;

    uint8_t cmToCell(float cm, float fieldCm, uint8_t cells)
    {
        const int c = (int)(cm / fieldCm * cells);
        return (uint8_t)constrain(c, 0, (int)cells - 1);
    }

    float cellToCm(uint8_t cell, float fieldCm, uint8_t cells)
    {
        return (cell + 0.5f) * fieldCm / cells;
    }

    /// true, wenn auf der Linie (Bresenham) nichts blockiert ist und keine Zelle teurer als costLimit ist
    bool lineClear(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t costLimit)
    {
        int x = x0;
        int y = y0;
        const int dx = abs((int)x1 - (int)x0);
        const int dy = abs((int)y1 - (int)y0);
        const int sx = (x0 < x1) ? 1 : -1;
        const int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        while (true)
        {
            const uint8_t w = field[x][y];
            if (w >= 255 || w > costLimit)
            {
                return false;
            }
            if (x == x1 && y == y1)
            {
                return true;
            }
            const int e2 = 2 * err;
            if (e2 > -dy)
            {
                err -= dy;
                x += sx;
            }
            if (e2 < dx)
            {
                err += dx;
                y += sy;
            }
        }
    }

    /**
     * Zellpfad auf wenige Wegpunkte reduzieren: von jedem Ankerpunkt so weit voraus,
     * wie eine freie Gerade moeglich ist. Die Gerade darf nie durch teurere Zellen laufen
     * als der A*-Weg selbst (die Gewichte der Karte bleiben also wirksam).
     *
     * @return Anzahl der Wegpunkte in wp (ohne Startzelle, der letzte ist die Zielzelle)
     */
    uint8_t simplifyPath(uint16_t len, uint8_t (&wp)[ARRAY_MAX_POINTS][2])
    {
        uint8_t runMax[PF_LOS_MAX_CELLS + 1];
        uint8_t n = 0;
        uint16_t i = 0;

        while (i + 1 < len && n < ARRAY_MAX_POINTS)
        {
            const uint16_t jMax = (i + PF_LOS_MAX_CELLS < len - 1) ? (i + PF_LOS_MAX_CELLS) : (len - 1);

            uint8_t m = 0;
            for (uint16_t j = i + 1; j <= jMax; j++)
            {
                const uint8_t w = field[pf.path[j][0]][pf.path[j][1]];
                if (w > m)
                    m = w;
                runMax[j - i] = m;
            }

            uint16_t best = i + 1; // mindestens ein Schritt: der A*-Weg ist begehbar
            for (uint16_t j = jMax; j > i + 1; j--)
            {
                if (lineClear(pf.path[i][0], pf.path[i][1], pf.path[j][0], pf.path[j][1], runMax[j - i]))
                {
                    best = j;
                    break;
                }
            }

            wp[n][0] = pf.path[best][0];
            wp[n][1] = pf.path[best][1];
            n++;
            i = best;
        }
        return n;
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
        return sqrtf(dx * dx + dy * dy) > PF_OBST_MOVE_CM;
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
        const uint8_t W = pathfinder::kFieldWidth;
        const uint8_t H = pathfinder::kFieldHeight;

        const uint8_t scx = cmToCell(sx, fieldX, W), scy = cmToCell(sy, fieldY, H);
        const uint8_t tcx = cmToCell(tx, fieldX, W), tcy = cmToCell(ty, fieldY, H);

        const bool hasBall = (pf.ballIn[0] >= 0 && pf.ballIn[1] >= 0);
        const bool hasRob = (pf.robIn[0] >= 0 && pf.robIn[1] >= 0);
        const uint8_t bx = hasBall ? cmToCell(pf.ballIn[0], fieldX, W) : PF_NO_OBSTACLE;
        const uint8_t by = hasBall ? cmToCell(pf.ballIn[1], fieldY, H) : PF_NO_OBSTACLE;
        const uint8_t rx = hasRob ? cmToCell(pf.robIn[0], fieldX, W) : PF_NO_OBSTACLE;
        const uint8_t ry = hasRob ? cmToCell(pf.robIn[1], fieldY, H) : PF_NO_OBSTACLE;

        uint16_t len = 0;
        bool ok = false;
        for (uint8_t attempt = 0; attempt < 3 && !ok; attempt++)
        {
            if (attempt == 1 && !hasBall)
                continue;
            if (attempt == 2 && !hasRob)
                continue;

            const bool useBall = (attempt == 0);
            const bool useRob = (attempt <= 1);
            ok = pathfinder::findPath(scx, scy, tcx, tcy,
                                      useBall ? bx : PF_NO_OBSTACLE, useBall ? by : PF_NO_OBSTACLE,
                                      useRob ? rx : PF_NO_OBSTACLE, useRob ? ry : PF_NO_OBSTACLE,
                                      pf.path, len);
        }
        if (!ok)
        {
            return false;
        }

        // ---- Wegpunkte (cm) -------------------------------------------------
        uint8_t wp[ARRAY_MAX_POINTS][2];
        uint8_t n = (len >= 2) ? simplifyPath(len, wp) : 0;

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
        if (restartPattern || total > pf.total0)
            pf.total0 = total;

        float cum = 0.0f;
        for (uint8_t i = 0; i < n; i++)
        {
            const float pStart = constrain(1.0f - (total - cum) / pf.total0, 0.0f, 1.0f);
            const float pMid = constrain(1.0f - (total - cum - 0.5f * seg[i]) / pf.total0, 0.0f, 1.0f);
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
                a.PassCm = PF_PASS_CM;
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

#endif // PF_AVAILABLE

/**
 * Hindernisse fuer die Pfadplanung [cm, Feldkoordinaten]. -1 = kein Hindernis.
 * Zyklisch aufrufbar, vor moveToPointAvoid(). Ball = Ball, Roboter = Partner (oder Gegner).
 */
void systemlib::setPathObstacles(int16_t ballX, int16_t ballY, int16_t robotX, int16_t robotY)
{
#if PF_AVAILABLE
    pf.ballIn[0] = ballX;
    pf.ballIn[1] = ballY;
    pf.robIn[0] = robotX;
    pf.robIn[1] = robotY;
#endif
}

/**
 * Wie moveToPoint(), aber um Hindernisse herum. Zyklisch aufrufen.
 * Alle Parameter wie bei moveToPoint(); das Speed-Pattern gilt fuer die ganze Strecke.
 *
 * Kein Weg gefunden: PF_FALLBACK_DIRECT = 1 faehrt geradeaus per moveToPoint(),
 * sonst bleibt der Roboter stehen und haelt den Winkel. Status: givePathFound().
 */
void systemlib::moveToPointAvoid(uint16_t X, uint16_t Y, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, int16_t Andrehwinkel)
{
#if PF_AVAILABLE
    // Position unbekannt: moveToPoint() haelt an und regelt den Winkel
    if (Position.X < 0.0f || Position.Y < 0.0f)
    {
        moveToPoint(X, Y, SpeedPattern, MaxSpeed, MinSpeed, Andrehwinkel);
        return;
    }

    const uint32_t nowMs = millis();

    const bool settingsChanged = !pf.valid || X != pf.tx || Y != pf.ty || SpeedPattern != pf.pattern || MaxSpeed != pf.maxS || MinSpeed != pf.minS;
    const bool moved = obstacleMoved(pf.ballIn, pf.ballUsed) || obstacleMoved(pf.robIn, pf.robUsed);
    const bool settled = pf.found && giveMoveArrived(); // am Ziel: nicht dauernd neu planen
    const uint32_t interval = pf.found ? PF_REPLAN_MS : PF_REPLAN_FAIL_MS;
    const bool due = !settled && (uint32_t)(nowMs - pf.lastPlanMs) >= interval;

    if (settingsChanged || moved || due)
    {
        pf.lastPlanMs = nowMs;
        pf.valid = true;
        pf.tx = X;
        pf.ty = Y;
        pf.pattern = SpeedPattern;
        pf.maxS = MaxSpeed;
        pf.minS = MinSpeed;
        for (uint8_t i = 0; i < 2; i++)
        {
            pf.ballUsed[i] = pf.ballIn[i];
            pf.robUsed[i] = pf.robIn[i];
        }

        ArrayPoint route[ARRAY_MAX_POINTS];
        uint8_t count = 0;

        const uint32_t t0 = micros();
        pf.found = buildRoute(Position.X, Position.Y, feld_x, feld_y, X, Y,
                              SpeedPattern, MaxSpeed, MinSpeed, Andrehwinkel, settingsChanged, route, count);
        pf.planUs = micros() - t0;

        if (pf.found)
        {
            pf.wpCount = count;
            setPointArray(route, count); // unveraenderte Wegpunkte: kein Neustart
        }
        else
        {
            pf.wpCount = 0;
            resetArray();
        }
    }

    if (pf.found)
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
#if PF_FALLBACK_DIRECT
        moveToPoint(X, Y, SpeedPattern, MaxSpeed, MinSpeed, Andrehwinkel);
#else
        setHeading(Andrehwinkel * DEG_TO_RAD);
        setOmniWheelDir(0, 0, giveHeadingController() * 100);
#endif
    }
#else
    moveToPoint(X, Y, SpeedPattern, MaxSpeed, MinSpeed, Andrehwinkel);
#endif
}

/**
 * Verwirft Planung, Punkt-Array und Fahrprofil (Speed 0). Aufrufen beim Wechsel in einen anderen
 * Modus, damit die naechste Fahrt ohne Reste der vorigen beginnt.
 */
void systemlib::resetPathPlan(void)
{
#if PF_AVAILABLE
    pf.valid = false;
    pf.found = false;
    pf.wpCount = 0;
    pf.total0 = 1.0f;
#endif
    resetArray();
    mv.lastSpeed = 0.0f;
}

#if PF_AVAILABLE
bool systemlib::givePathFound(void) const { return pf.found; }
uint8_t systemlib::givePathWaypoints(void) const { return pf.wpCount; }
uint32_t systemlib::givePathTimeUs(void) const { return pf.planUs; }
#else
bool systemlib::givePathFound(void) const { return false; }
uint8_t systemlib::givePathWaypoints(void) const { return 0; }
uint32_t systemlib::givePathTimeUs(void) const { return 0; }
#endif

void systemlib::setOmniWheelVek(float vx, float vy, float omega)
{
    float w[4];
    static const float wheelAngles[4] = {45, 315, 135, 225};
    for (int i = 0; i < 4; ++i)
    {
        w[i] = -sinf(wheelAngles[i] * DEG_TO_RAD) * -vx + cosf(wheelAngles[i] * DEG_TO_RAD) * vy + omega;
        w[i] = constrain(w[i], -100, 100);
    }
    setMotorData(MoDataVL.ID, w[0], true, false);
    setMotorData(MoDataVR.ID, w[1], true, false);
    setMotorData(MoDataHL.ID, w[2], true, false);
    setMotorData(MoDataHR.ID, w[3], true, false);
}

void systemlib::setOmniWheelDir(float directionDeg, uint8_t speedPercent, float omega)
{
    directionDeg = directionDeg - BNO_OrbitDir;

    directionDeg = Util.AMap(directionDeg);

    float w[4];
    static const float wheelAngles[4] = {45, 315, 135, 225};
    // Serial.println("omega : "+String(omega));  // deaktiviert: blockierendes USB-Serial im 100-Hz-Takt verfaelscht dt

    float directionRad = directionDeg * DEG_TO_RAD;
    float speedNorm = speedPercent / 100.0f; // 0..1

    float vx = cosf(directionRad) * speedNorm * 100;
    float vy = sinf(directionRad) * speedNorm * 100;

    for (int i = 0; i < 4; ++i)
    {
        w[i] = -sinf(wheelAngles[i] * DEG_TO_RAD) * -vx + cosf(wheelAngles[i] * DEG_TO_RAD) * vy + omega;
        w[i] = constrain(w[i], -100, 100);
    }

    setMotorData(MoDataVL.ID, w[0], true, false);
    setMotorData(MoDataVR.ID, w[1], true, false);
    setMotorData(MoDataHL.ID, w[2], true, false);
    setMotorData(MoDataHR.ID, w[3], true, false);
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

void systemlib::moveDir(int16_t Dir, uint8_t Speed, float Andrehwinkel)
{
    setHeading(Andrehwinkel * DEG_TO_RAD);
    if (checkWallDistance(Position.X, Position.Y) == 1)
    {
        setOmniWheelDir(Dir, 5, giveHeadingController() * 100);
    }
    else
    {
        setOmniWheelDir(Dir, Speed, giveHeadingController() * 100);
    }
}