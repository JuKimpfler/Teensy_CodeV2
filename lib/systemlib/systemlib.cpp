#include <systemlib.h>

/* -------------- Instanzen -------------- */

BNO055 BNO(BNO_ADDRESS, &I2C_BNO);
PowerDebugger PDS;
elapsedMillis cycleTimer;
elapsedMicros cycleTimer_us;

commlib communication;

I2C_bus i2cIR(I2C_IR);
Device_handle ir_ring_handle = {i2cIR, I2C_target{0x0A}};
IR_ring ir_ring(ir_ring_handle);

INA219 INA219_Main(0x40);
INA219 INA219_Motor(0x41);

SRF08Sensor SRF08_vo(SRF08_ADDR(0xE2), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor SRF08_re(SRF08_ADDR(0xE4), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor SRF08_hi(SRF08_ADDR(0xE6), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor SRF08_li(SRF08_ADDR(0xE8), SRF08_RANGE_2M, SRF08_GAIN_MID);

TCA9534DW SW_EXP(&I2C_SW, 0x22);
TCA9534DW Motor_EXP(&Wire1, 0x20);
TCA9534DW Button_EXP(&Wire1, 0x23);
TCA9534DW Rand_EXP(&Wire1, 0x21);

ADC108S102 ADC_Motor(&SPI1,ADC108S102_Motor_CS);


constexpr float TS_NOMINAL = 0.005f; // nur Fallback fuer cfg.Ts, siehe loop()
PIDConfig cfgP;
PID* headingControllerP;
ADRCConfig cfgA;
ADRC* headingControllerA;
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
    Wire1.setClock(400000);
    Wire.begin();
    Wire.setClock(400000);
    USBC_UART.begin(115200);

    INA219_Main.begin(Wire1);
    INA219_Motor.begin(Wire1);

    INA219_Main.setMaxCurrentShunt(15);
    INA219_Motor.setMaxCurrentShunt(15);

    BNO.init();
    PDS.init();

    MotorVL_D.ID = MotorVL_ID;
    MotorVR_D.ID = MotorVR_ID;
    MotorHL_D.ID = MotorHL_ID;
    MotorHR_D.ID = MotorHR_ID;
    MotorDR_D.ID = MotorDR_ID;

    pinMode(START_SWITCH, INPUT);

    // -- US -----
    SRF08_vo.setEMAAlpha(0.25f);
    SRF08_hi.setEMAAlpha(0.25f);
    SRF08_re.setEMAAlpha(0.25f);
    SRF08_li.setEMAAlpha(0.25f);

    SRF08_vo.setJumpThreshold(50);
    SRF08_hi.setJumpThreshold(50);
    SRF08_re.setJumpThreshold(50);
    SRF08_li.setJumpThreshold(50);

    SRF08_vo.begin(I2C_US);
    SRF08_hi.begin(I2C_US);
    SRF08_re.begin(I2C_US);
    SRF08_li.begin(I2C_US);

    SRF08_vo.startRanging();
    SRF08_hi.startRanging();
    SRF08_re.startRanging();
    SRF08_li.startRanging();

    Motor_EXP.setup(false);
    Rand_EXP.setup(false);
    SW_EXP.setup(true);
    Button_EXP.setup(true);

    ir_ring.read_gains(IR_Gains);
    ir_ring.read_offsets(IR_Offsets);

    setDebugTimeout(5);

    ADC_Motor.init();

    setupHeadingController(true);
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
    readIR();
    //readUS();

    readExpanders();

    PDS.update();

    updateHeadingController();
    updateMotors();
}

/* -------------- Debugger -------------- */
void systemlib::setDBGSys()
{
    PDS.Channel(21, BNO_OrbitDir);
    PDS.Channel(22, BNO.eulHeading());
    PDS.Channel(23, BNO.calibStatus());
    PDS.Channel(24, BNO.systemStatus());
    PDS.Channel(25, 0);
    PDS.Channel(26, 0);
    PDS.Channel(27, 0);
    PDS.Channel(28, 0);
    PDS.Channel(29, 0);
    PDS.Channel(30, 0);
    PDS.Channel(31, 0);
    PDS.Channel(32, 0);
    PDS.Channel(33, 0);
    PDS.Channel(34, 0);
    PDS.Channel(35, digitalRead(START_SWITCH));
    PDS.Channel(36, 0);
    PDS.Channel(37, 0);
    PDS.Channel(38, analogRead(LIGHT_BARRIER));
    PDS.Channel(39, 0);
    PDS.Channel(40, 0);
}

void systemlib::setDBGChannel(u_int8_t Channel, float Value)
{
    if (Util.Range(Channel, 0, 20))
    {
        PDS.Channel(Channel, Value);
    }
}

/* -------------- BNO -------------- */
void systemlib::calibBNO(void)
{
    BNO_calibration = BNO_eulerZ;
}

void systemlib::readBNO(void)
{
    BNO_eulerZ = Util.AMap(BNO.eulHeading());
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
    for(int i = 0; i<6 ; i++){Summe+=INA_filter[i];}
    Summe = Summe/6;
    return Summe;
}

/* -------------- US -------------- */

void systemlib::readUS(void)
{
    if (SRF08_vo.update())
        US_vo = SRF08_vo.getDistance(), SRF08_vo.startRanging();

    if (SRF08_hi.update())
        US_hi = SRF08_hi.getDistance(), SRF08_hi.startRanging();

    if (SRF08_re.update())
        US_re = SRF08_re.getDistance(), SRF08_re.startRanging();

    if (SRF08_li.update())
        US_li = SRF08_li.getDistance(), SRF08_li.startRanging();
}

/* -------------- Aktors ------------- */
void systemlib::setMotorData(MotorData Data){
    if(Data.ID == MotorVR_D.ID){MotorVR_D = Data;}
    else if(Data.ID == MotorHR_D.ID){MotorHR_D = Data;}
    else if(Data.ID == MotorVL_D.ID){MotorVL_D = Data;}
    else if(Data.ID == MotorHL_D.ID){MotorHL_D = Data;}
    else if(Data.ID == MotorDR_D.ID){MotorDR_D = Data;}

    updateMotors();
}

void systemlib::updateMotors(void){
    uint8_t Port_List[8];
    byte out = 0;

    analogWrite(MotorVL_PWM,abs(MotorVL_D.Speed)*2.55);
    analogWrite(MotorVR_PWM,abs(MotorVR_D.Speed)*2.55);
    analogWrite(MotorHR_PWM,abs(MotorHR_D.Speed)*2.55);
    analogWrite(MotorHL_PWM,abs(MotorHL_D.Speed)*2.55);

    if(MotorVL_D.Dir){Port_List[MotorVL_P1]=0;}else{Port_List[MotorVL_P1]=1;}
    if(MotorVR_D.Dir){Port_List[MotorVR_P1]=0;}else{Port_List[MotorVR_P1]=1;}
    if(MotorHL_D.Dir){Port_List[MotorHL_P1]=0;}else{Port_List[MotorHL_P1]=1;}
    if(MotorHR_D.Dir){Port_List[MotorHR_P1]=0;}else{Port_List[MotorHR_P1]=1;}

    if(MotorVL_D.Enable){Port_List[MotorVL_P2]=1;}else{Port_List[MotorVL_P2]=0;}
    if(MotorVR_D.Enable){Port_List[MotorVR_P2]=1;}else{Port_List[MotorVR_P2]=0;}
    if(MotorHL_D.Enable){Port_List[MotorHL_P2]=1;}else{Port_List[MotorHL_P2]=0;}
    if(MotorHR_D.Enable){Port_List[MotorHR_P2]=1;}else{Port_List[MotorHR_P2]=0;}

    // Konvertieren von Liste zu byte
    for (int i = 0; i < 8; i++) {
        if (Port_List[i] == 1) {
            out |= (1 << (7 - i));
        }
    }

    Motor_EXP.write(out);

    MotorVL_D.Speed_real = ADC_Motor.read(MotorVL_AP1,true,true);
    MotorVL_D.Current = ADC_Motor.read(MotorVL_AP2,true,false);
    MotorVR_D.Speed_real = ADC_Motor.read(MotorVR_AP1,true,true);
    MotorVR_D.Current = ADC_Motor.read(MotorVR_AP2,true,false);
    MotorHL_D.Speed_real = ADC_Motor.read(MotorHL_AP1,true,true);
    MotorHL_D.Current = ADC_Motor.read(MotorHL_AP2,true,false);
    MotorHR_D.Speed_real = ADC_Motor.read(MotorHR_AP1,true,true);
    MotorHR_D.Current = ADC_Motor.read(MotorHR_AP2,true,false);
}

MotorData systemlib::giveMotorData(uint8_t ID){
    MotorData Error_Data;
    Error_Data.ID = 100;

    if(ID == MotorVR_D.ID){return MotorVR_D;}
    else if(ID == MotorHR_D.ID){return MotorHR_D;}
    else if(ID == MotorVL_D.ID){return MotorVL_D;}
    else if(ID == MotorHL_D.ID){return MotorHL_D;}
    else if(ID == MotorDR_D.ID){return MotorDR_D;}
    else {return Error_Data;}
}

/* -------------- Expanders -------------- */
void systemlib::readExpanders(void)
{
    SW_EXP.read(); // Update the internal state of the port expander
    Button_EXP.read(); // Update the internal state of the port expander

    BTN1.update(Button_EXP.readPin(0));
    BTN2.update(Button_EXP.readPin(1));
    BTN3.update(Button_EXP.readPin(2));
    BTN4.update(Button_EXP.readPin(3));
    BTN5.update(Button_EXP.readPin(4));

    fancyBTNstates[0] = BTN1.getStatusCode();
    fancyBTNstates[1] = BTN2.getStatusCode();
    fancyBTNstates[2] = BTN3.getStatusCode();
    fancyBTNstates[3] = BTN4.getStatusCode();
    fancyBTNstates[4] = BTN5.getStatusCode();

    // Read the state of each pin from the port expander
    // Example: bool pin0State = portExpander.getPinState(0);
}


void systemlib::setupHeadingController(bool useADRC){
    _useADRC = useADRC;
    if(_useADRC){
        cfgA.order = ADRCOrder::SECOND;   // Winkel = Strecke 2. Ordnung
        cfgA.Ts    = TS_NOMINAL;          // nur Fallback, siehe loop()
        cfgA.dtMin = 1e-5f;               // 10 us -- gegen dt=0
        cfgA.dtMax = 0.03f;               // 30 ms -- deckelt Ausreisser, falls das
                                        // Hauptprogramm mal laenger braucht
        cfgA.wc    = 15.0f;               // Reglerbandbreite: wie schnell ausgerichtet wird
        cfgA.wo    = 80.0f;               // Beobachterbandbreite: ca. 5x wc als Startwert
        cfgA.b0    = 8.0f;                // grobe Schaetzung: wie stark omega_cmd auf
                                        // die Winkelbeschleunigung wirkt -> per Test anpassen
        cfgA.uMin  = -1.0f;
        cfgA.uMax  =  1.0f;
        cfgA.angleMode = true;            // WICHTIG: sorgt fuer kuerzesten Drehweg (kein +-pi Sprung)
        cfgA.outputRateLimit = 6.0f;      // begrenzt Stellgroessen-Aenderungsrate (sanfter Anlauf)

        headingControllerA = new ADRC(cfgA);
        headingControllerA->reset(DEG_TO_RAD*BNO_OrbitDir);
        lastMicrosContrl = micros();
    }
    else{
        cfgP.Ts    = TS_NOMINAL;          // nur Fallback, siehe loop()
        cfgP.dtMin = 1e-5f;               // 10 us -- gegen dt=0
        cfgP.dtMax = 0.03f;               // 30 ms -- deckelt Ausreisser, falls das
                                        // Hauptprogramm mal laenger braucht

        cfgP.Kp = 2.0f;                   // Start: nur P, dann I/D schrittweise zuschalten
        cfgP.Ki = 0.5f;
        cfgP.Kd = 0.05f;

        cfgP.uMin = -1.0f;
        cfgP.uMax =  1.0f;
        cfgP.angleMode = true;            // WICHTIG: kuerzester Drehweg, kein +-pi Sprung
        cfgP.derivativeOnMeasurement = true; // kein Kick bei Sollwinkel-Spruengen
        cfgP.Tf = 0.02f;                  // D-Anteil filtern (Winkelmessung ist meist verrauscht)
        cfgP.outputRateLimit = 6.0f;      // sanfter Anlauf / Ruckbegrenzung

        headingControllerP = new PID(cfgP);
        headingControllerP->reset(DEG_TO_RAD*BNO_OrbitDir);
        lastMicrosContrl = micros();
    }
}

float systemlib::giveHeadingController(){
    if(_useADRC){
        return ADRC_Out;
    }
    else{
        return PID_Out;
    }
}

void systemlib::setHeading(float Heading){
    _OrbitHeading = Heading;
}

void systemlib::updateHeadingController(){
    float dt = (micros() - lastMicrosContrl) * 1e-6f; // us -> s
    lastMicrosContrl = micros();

    if(_useADRC){
        ADRC_Out = headingControllerA->update(_OrbitHeading, DEG_TO_RAD*BNO_OrbitDir, dt);
    }
    else{
        PID_Out = headingControllerP->update(_OrbitHeading, DEG_TO_RAD*BNO_OrbitDir, dt);
    }
}