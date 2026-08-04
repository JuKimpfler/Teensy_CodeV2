#include <Arduino.h>
#include <PDS.h>
#include <systemlib.h>

systemlib sys;

RobotDataCom IMAS2;

PowerDebugger PDS;

void PDSbind()
{
  // Alle standard binds hier eintragen
}

void setup()
{
  sys.initialize();
  PDSbind();
}

void loop()
{
  sys.readSensors();

  // if start switch
  if (false)
  {
  }
}