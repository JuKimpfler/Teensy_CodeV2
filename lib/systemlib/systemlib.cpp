#include <systemlib.h>

/* -------------- Instanzen -------------- */

BNO055 BNO( BNO_ADDRESS, &I2C_BNO);
PowerDebugger PDS;
elapsedMillis cycleTimer;
elapsedMicros cycleTimer_us;

commlib communication;

SRF08Sensor SRF08_vo(SRF08_ADDR(0xE2), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor SRF08_re(SRF08_ADDR(0xE4), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor SRF08_hi(SRF08_ADDR(0xE6), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor SRF08_li(SRF08_ADDR(0xE8), SRF08_RANGE_2M, SRF08_GAIN_MID);

/* -------------- System Worker -------------- */

void systemlib::initialize()
{
    Wire1.begin();
    Wire1.setClock(1000000);
    USBC_UART.begin(115200);

    BNO.init();
    PDS.init();

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

    pinMode(START_SWITCH,INPUT);
    setDebugTimeout(5);

}

void systemlib::readSensors( void ){
    // Zyklische Messung der Laufzeit in ms
    cycleTime = cycleTimer;
    cycleTime_us = cycleTimer_us;
    delay(debugTimeout);
    cycleTimer = 0;
    cycleTimer_us = 0;

    readBNO();
    readUS();
    PDS.update();

    Serial.println("Fresh = "+String(PDS.fastParamsAreFresh())+" , Param1 = "+String(PDS.getFastParam(0)));
}

/* -------------- Debugger -------------- */
void systemlib::setDBGSys(){
    PDS.Channel(21,BNO_OrbitDir);
    PDS.Channel(22,BNO.eulHeading());
    PDS.Channel(23,BNO.calibStatus());
    PDS.Channel(24,BNO.systemStatus());
    PDS.Channel(25,0);
    PDS.Channel(26,0);
    PDS.Channel(27,0);
    PDS.Channel(28,0);
    PDS.Channel(29,0);
    PDS.Channel(30,0);
    PDS.Channel(31,0);
    PDS.Channel(32,0);
    PDS.Channel(33,0);
    PDS.Channel(34,0);
    PDS.Channel(35,digitalRead(START_SWITCH));
    PDS.Channel(36,0);
    PDS.Channel(37,0);
    PDS.Channel(38,analogRead(LIGHT_BARRIER));
    PDS.Channel(39,0);
    PDS.Channel(40,0);

}

void systemlib::setDBGChannel( u_int8_t Channel , float Value ){
    if(Util.Range(Channel,0,20)){
        PDS.Channel(Channel, Value);
    }
}

/* -------------- BNO -------------- */
void systemlib::calibBNO( void )
{
    BNO_calibration = BNO_eulerZ;
}

void systemlib::readBNO( void )
{
    BNO_eulerZ = Util.AMap(BNO.eulHeading());
    BNO_OrbitDir = Util.AMap(BNO_eulerZ - BNO_calibration);
}

/* ---------- Communication -----------*/

// send

void systemlib::communicationSend( void )
{
    communication.sendOwnFlush();
}

void systemlib::communicationSend( float positionX, float positionY, float ballAngle, float ballDistance, float goalAngle, float goalDistance ){
    communication.setOwnFlush(positionX, positionY, ballAngle, ballDistance, goalAngle, goalDistance);
    communication.sendOwnFlush();
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