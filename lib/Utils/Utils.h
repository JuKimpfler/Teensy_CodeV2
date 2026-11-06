#pragma once
#include <Arduino.h>
#include "elapsedMillis.h"

static class Timer{
    public:
        elapsedMillis cycle;
        elapsedMillis power;   // drosselt die INA219-Abfrage fuer die PDS-Kanaele
        elapsedMicros cycle_us;
        elapsedMillis BNOCalib;
        elapsedMillis debug;
}

class Utilslib
{
    public:
        bool Range(float Value , float min , float max); // is in Range
        float AMap( float Angle ); // Angle maping to +- 180°
};

extern Utilslib Util;