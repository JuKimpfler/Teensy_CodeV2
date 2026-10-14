#include <Arduino.h>

#include <systemlib.h>

systemlib sys;

RobotDataCom IMAS2;

void setup()
{
    sys.initialize();
}

void loop()
{
    sys.readSensors();

    //if start switch
    if (false)
    {
        MotorVR_D.Speed = sys.giveHeadingController();
        MoHR.Speed = sys.giveHeadingController();
        MoVL.Speed = sys.giveHeadingController();
        MoVR.Speed = sys.giveHeadingController();
        sys.setMotorData(sys.giveMotorData(MotorDR_ID));
    }
    else
    {
        sys.resetHeadingController();
    }
    USBC_UART.println("alive");

    sys.setDBGSys();
    sys.setDBGChannel(0, sys.giveCycleTime_us());
}
