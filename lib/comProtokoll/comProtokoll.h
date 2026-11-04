#pragma once
#include <Arduino.h>
#include "SerialSync.h"   // Dateiname exakt so schreiben (Gross-/Kleinschreibung, Linux/CI)

// Gleiche Schnittstelle auf ESP32 und Teensy.
// Die Implementierung liegt je Geraet in einer eigenen .cpp
class DataSync {
public:
  void begin();
  void update();
  bool isConnected();

  // Diagnose
  uint32_t rxBytes();
  uint32_t rxOk();
  uint32_t rxBad();
  uint32_t uartErrors();     // UART-Fehlerflags (Overrun/Framing/Noise), vom Treiber nie geloescht
  uint32_t uartOverruns();   // davon Overruns (Empfaenger ist danach gesperrt, bis das Flag geloescht wird)

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

  float radar_test;
};