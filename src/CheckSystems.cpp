#include <Arduino.h>

#include <systemlib.h>

systemlib sys; // IMAS1

RobotDataCom IMAS2;

void setup()
{
    sys.initialize();
}

void loop()
{
    sys.readSensors();
    sys.communicationSend();

    sys.setDBGSys();
    sys.setDBGChannel(0, sys.giveCycleTime_us());

    USBC_UART.println("CycleTime = " + String(sys.giveCycleTime()) + " ms (" + String(sys.giveCycleTime_us()) + " us)");
    USBC_UART.println("BNO_OrbitDir = " + String(sys.giveOrbitDir()) + " °");
    USBC_UART.print("US_vo = " + String(sys.giveUS_vo()) + " cm , ");
    USBC_UART.print("US_hi = " + String(sys.giveUS_hi()) + " cm , ");
    USBC_UART.print("US_re = " + String(sys.giveUS_re()) + " cm , ");
    USBC_UART.println("US_li = " + String(sys.giveUS_li()) + " cm");
}