#include <systemlib.h>

/* -------------- Instanzen -------------- */

BNO055 BNO( BNO_ADDRESS, &I2C_BNO);
PowerDebugger PDS;
elapsedMillis cycleTimer;
elapsedMicros cycleTimer_us;
//int cycleTimer_us =0;
//int cycleTimer =0;

/* -------------- System Worker -------------- */

void systemlib::initialize()
{
    Wire1.begin();
    Wire1.setClock(1000000);
    USBC_UART.begin(115200);

    BNO.init();
    PDS.init();

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
