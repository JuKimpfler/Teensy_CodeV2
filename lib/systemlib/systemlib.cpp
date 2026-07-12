#include <systemlib.h>

/* -------------- Instanzen -------------- */

BNO055 BNO(BNO_ADDRESS, &I2C_BNO);
PowerDebugger PDS;
elapsedMillis cycleTimer;
elapsedMicros cycleTimer_us;

commlib communication;

I2C_bus i2c1(Wire1);
Device_handle ir_ring_handle = {i2c1, I2C_target{0x0A}};
IR_ring ir_ring(ir_ring_handle);

/* -------------- System Worker -------------- */

void systemlib::initialize()
{
    Wire1.begin();
    Wire1.setClock(1000000);
    USBC_UART.begin(115200);

    BNO.init();
    PDS.init();

    ir_ring.read_gains(IR_Gains);
    ir_ring.read_offsets(IR_Offsets);

    pinMode(START_SWITCH, INPUT);
    setDebugTimeout(5);
}

void systemlib::readSensors(void)
{
    // Zyklische Messung der Laufzeit in ms
    cycleTime = cycleTimer;
    cycleTime_us = cycleTimer_us;
    delay(debugTimeout);
    cycleTimer = 0;
    cycleTimer_us = 0;

    readBNO();
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