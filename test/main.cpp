#include <Arduino.h>
#include <PDS.h>
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

  // if start switch
  if (sys.giveStart()) // Game Loop
  {
  }
  else if (sys.giveExpStartSwitch()) // gestoppt aber homing etc. erlaubt
  {
  }
  else // Motor aus
  {
  }
}