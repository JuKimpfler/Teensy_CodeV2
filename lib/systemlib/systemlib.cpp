#include <systemlib.h>


BNO055 BNO( 0x28, &I2C_BNO);
BNO055 BNO1( 0x29, &I2C_BNO);


void systemlib::initialize()
{
    Wire1.begin();
    Wire1.setClock(1000000);
    USBC_UART.begin(115200);

    BNO.init();
    BNO1.init();
}

void systemlib::calibrateBNO( void )
{
    readBNO();
    BNO_calibration = BNO_eulerZ;
}

void systemlib::readSensors( void ){
    readBNO();
}

float systemlib::getOrbitDir( void )
{
    return Util.AMap(BNO_eulerZ - BNO_calibration);
}

void systemlib::readBNO( void )
{
    BNO_eulerZ = Util.AMap(BNO.eulHeading());

    BNO_eulerZ_test = Util.AMap(BNO1.eulHeading());
}