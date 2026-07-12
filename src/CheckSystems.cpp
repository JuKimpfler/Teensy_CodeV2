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
    USBC_UART.println("Ball_Angle= " + String(sys.giveBallAngle()) + " °");
    USBC_UART.println("Ball_Distance= " + String(sys.giveBallDistance()) + " cm (hoffentlich)");
}