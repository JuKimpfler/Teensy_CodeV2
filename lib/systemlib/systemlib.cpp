#include <systemlib.h>

/* -------------- Instanzen -------------- */

BNO055 BNO( 0x28, &I2C_BNO);
elapsedMillis cycleTimer;


/* -------------- System Worker -------------- */

void systemlib::initialize()
{
    Wire1.begin();
    Wire1.setClock(1000000);
    USBC_UART.begin(115200);

    BNO.init();
}

void systemlib::readSensors( void ){
    // Zyklische Messung der Laufzeit in ms
    cycleTime = cycleTimer;
    cycleTimer = 0;

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
