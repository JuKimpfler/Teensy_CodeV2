#include <Arduino.h>

#include <systemlib.h>

systemlib sys;

void setup()
{
  sys.initialize();
}

void loop()
{
  USBC_UART.println("alive");
  delay(1000);
}