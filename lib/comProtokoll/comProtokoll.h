#pragma once
#include <Arduino.h>
#include "SerialSync.h"

// Gleiche Schnittstelle auf ESP32 und Teensy.
// Die Implementierung liegt je Geraet in einer eigenen .cpp
class DataSync {
public:
  void begin();
  void update();
  bool isConnected();

  bool Position_AtGoal;
  bool Position_YVal;
  bool Position_XVal;
  float Position_Cert;
  float PositionY;
  float PositionX;

  bool BNO_Calib;
  int32_t BNO_Heading;
  float Motor_SpeedVL;
  float Motor_SpeedVR;
  float Motor_SpeedHL;
  float Motor_SpeedHR;
};