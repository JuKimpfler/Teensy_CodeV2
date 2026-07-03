#include <Arduino.h>
#include "roboterDaten.h"
#include "commlib.h"

#include <systemlib.h>

systemlib sys;

void setup()
{
  sys.initialize();
}

void loop()
{
  sys.readSensors();

  //if start switch
  if (false){
    commlibC.setOwnFlush(0,0,0,0,0,0);

    roboterDaten otherRobot = commlibC.getOtherFlush();
  }
  USBC_UART.println("alive");

  USBC_UART.print(sys.giveCycleTime_us());
  USBC_UART.print(" , bno_main : ");
  USBC_UART.println(sys.giveOrbitDir());
}