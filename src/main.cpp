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
  if (false){
    
  }
  USBC_UART.println("alive");

  USBC_UART.print(sys.giveCycleTime_us());
  USBC_UART.print(" , bno_main : ");
  USBC_UART.println(sys.giveOrbitDir());
}