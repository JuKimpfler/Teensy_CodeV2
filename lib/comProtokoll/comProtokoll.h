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

  bool test_bool;
  int32_t test_int;
  float test_float;

  bool rc_bool;
  int32_t rc_int;
  float rc_float;
};