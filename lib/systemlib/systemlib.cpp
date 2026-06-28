#include <systemlib.h>

void systemlib::initialize()
{
    USBC_UART.begin(9600);
}