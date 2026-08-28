#include <Arduino.h>
#include <PDS.h>
#include <systemlib.h>

systemlib sys;

RobotDataCom IMAS2;

bool identity = false;

void setup()
{
  sys.initialize();
  pinMode(21, INPUT);
  identity = digitalRead(21);
}

void loop()
{
  sys.readSensors();
  uint8_t buttons[5];
  sys.giveButtonStates(buttons);

  // if start switch
  if (sys.giveStart()) // Game Loop
  {
  }
  else if (sys.giveExpStartSwitch()) // gestoppt aber homing etc. erlaubt
  {
    if (buttons[0] == fancy_CLICKED) // move to point
    {
      if (identity)
        sys.moveToPoint(50, 50, 20, 0);
      else
        sys.moveToPoint(50, 30, 20, 0);
    }

    if (buttons[0] == fancy_DOUBLE_CLICK)
      ; // ir Calibration

    if (buttons[0] == fancy_LONG_PRESS)
      ; // line Calibration

    if (buttons[1] == fancy_CLICKED)
      ; // Cam CalibrationW

    if (buttons[2] == fancy_CLICKED) // cycle config
      ;
  }
  else // Motor aus
  {
    sys.moveDir(0, 0, 0); // Motoren aus
  }
}