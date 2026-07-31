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

  USBC_UART.print("main_voltage : ");
  USBC_UART.print(sys.giveMainVoltage());
  USBC_UART.print(" , main_current : ");
  USBC_UART.print(sys.giveMainCurrent());
  USBC_UART.print(" , motor_voltage : ");
  USBC_UART.print(sys.giveMotorVoltage());
  USBC_UART.print(" , motor_current : ");
  USBC_UART.println(sys.giveMotorCurrent());

  sys.setDBGSys();
  sys.setDBGChannel(0,sys.giveCycleTime_us());
}