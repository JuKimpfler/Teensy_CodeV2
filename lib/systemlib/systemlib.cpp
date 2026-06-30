#include <systemlib.h>

/* -------------- Instanzen -------------- */

BNO055 BNO( BNO_ADDRESS, &I2C_BNO);
PowerDebugger PDS;
elapsedMillis cycleTimer;
elapsedMicros cycleTimer_us;


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

/* -------------- Debugger -------------- */
void systemlib::setDBGSys(){
    PDS.Channel(21,BNO_OrbitDir);
    PDS.Channel(21,BNO_OrbitDir);
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
