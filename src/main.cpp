#include <Arduino.h>

#include <systemlib.h>

systemlib sys;

void setup()
{
  sys.initialize();
}

void loop()
{

  sys.readSensors();
  USBC_UART.println("alive");

  USBC_UART.print(sys.giveCycleTime_us());
  USBC_UART.print(" , bno_main : ");
  USBC_UART.println(sys.giveOrbitDir());

  sys.setDBGSys();
  sys.setDBGChannel(0,sys.giveCycleTime_us());
}