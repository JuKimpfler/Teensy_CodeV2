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

  USBC_UART.print(sys.giveCycleTime());
  USBC_UART.print(" , bno_main : ");
  USBC_UART.println(sys.giveOrbitDir());

  

  delay(1000);
}