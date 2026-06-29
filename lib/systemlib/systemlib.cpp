#include <systemlib.h>

void systemlib::initialize()
{
    USBC_UART.begin(9600);

    BNO.init();
}

void systemlib::calibrateBNO( void )
{
    readBNO();
    BNO_calibration = BNO_eulerZ;
}

float systemlib::getOrbitDirection( void )
{
    return BNO_eulerZ - BNO_calibration;
}

void systemlib::readBNO( void )
{
    BNO_eulerZ = BNO.eulHeading();
}