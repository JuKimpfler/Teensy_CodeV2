#include <systemlib.h>

/* -------------- Instanzen -------------- */

BNO055 BNO( 0x28, &I2C_BNO);
elapsedMillis cycleTimer;
elapsedMicros cycleTimer_us;

commlib communication;


/* -------------- System Worker -------------- */

void systemlib::initialize()
{
    Wire1.begin();
    Wire1.setClock(1000000);
    USBC_UART.begin(115200);

    setDebugTimeout(10);

    BNO.init();


}

void systemlib::readSensors( void ){
    // Zyklische Messung der Laufzeit in ms
    cycleTime = cycleTimer;
    cycleTime_us = cycleTimer_us;
    delay(debugTimeout);
    cycleTimer = 0;
    cycleTimer_us = 0;

    readBNO();
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

void systemlib::send( void )
{
    communication.sendOwnFlush();
}

void systemlib::send( float positionX, float positionY, float ballAngle, float ballDistance, float goalAngle, float goalDistance ){
    communication.setOwnFlush(positionX, positionY, ballAngle, ballDistance, goalAngle, goalDistance);
    communication.sendOwnFlush();
}

// give

RobotDataCom systemlib::giveOtherRobotData() const{
    return communication.getOtherFlush();
}