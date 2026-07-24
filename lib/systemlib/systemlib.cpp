#include <systemlib.h>

/* -------------- Instanzen -------------- */

BNO055 BNO(BNO_ADDRESS, &I2C_BNO);
PowerDebugger PDS;
elapsedMillis cycleTimer;
elapsedMicros cycleTimer_us;

commlib communication;

Motor motor;

I2C_bus i2c1(Wire1);
Device_handle ir_ring_handle = {i2c1, I2C_target{0x0A}};
IR_ring ir_ring(ir_ring_handle);

INA219 INA219_Main(0x40);
INA219 INA219_Motor(0x41);

SRF08Sensor SRF08_vo(SRF08_ADDR(0xE2), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor SRF08_re(SRF08_ADDR(0xE4), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor SRF08_hi(SRF08_ADDR(0xE6), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor SRF08_li(SRF08_ADDR(0xE8), SRF08_RANGE_2M, SRF08_GAIN_MID);

TCA9534DW portExpander(&Wire1, 0x20);

ADC108S102 ADC108S102_1(&SPI1,ADC108S102_CS1);
/* -------------- System Worker -------------- */

void systemlib::initialize()
{
    Wire1.begin();
    Wire1.setClock(1000000);
    USBC_UART.begin(115200);

    INA219_Main.begin();
    INA219_Motor.begin();

    INA219_Main.setMaxCurrentShunt(15);
    INA219_Motor.setMaxCurrentShunt(15);

    BNO.init();
    PDS.init();


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

    SRF08_vo.begin(Wire1);
    SRF08_hi.begin(Wire1);
    SRF08_re.begin(Wire1);
    SRF08_li.begin(Wire1);

    SRF08_vo.startRanging();
    SRF08_hi.startRanging();
    SRF08_re.startRanging();
    SRF08_li.startRanging();

    portExpander.setup(true);
    ir_ring.read_gains(IR_Gains);
    ir_ring.read_offsets(IR_Offsets);

    setDebugTimeout(5);
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

    readUS();
    PDS.update();

    Serial.println("Fresh = " + String(PDS.fastParamsAreFresh()) + " , Param1 = " + String(PDS.getFastParam(0)));
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

/* --------------  Motors   --------------*/
void systemlib::setMotor(u_int8_t MotorID, int speed){
    motor.setMotor(MotorID, speed);
}

void systemlib::setAllMotors(int Motor1, int Motor2, int Motor3, int Motor4){
    motor.setMotor(1, Motor1);
    motor.setMotor(2, Motor2);
    motor.setMotor(3, Motor3);
    motor.setMotor(4, Motor4);
}

void systemlib::move(float direction, int speed){
    //TODO
    int Motor1 = 1;
    int Motor2 = 1;
    int Motor3 = 1;
    int Motor4 = 1;

    int max = fmax(fmax(fmax(Motor1, Motor2), Motor3), Motor4);
    float mapping = speed/max;

    Motor1 *= mapping;
    Motor2 *= mapping;
    Motor3 *= mapping;
    Motor4 *= mapping;

    setAllMotors(Motor1, Motor2, Motor3, Motor4);
}

void systemlib::move(float direction, int speed, float turn){
    //TODO
}

/* -------------- Expanders -------------- */
void systemlib::readExpanders(void)
{
    portExpander.read(); // Update the internal state of the port expander

    // Read the state of each pin from the port expander
    // Example: bool pin0State = portExpander.getPinState(0);
}