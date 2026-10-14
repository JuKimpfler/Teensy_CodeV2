#pragma once
#include <Arduino.h>

class Utils
{
public:
    bool inRange(float Value, float min, float max); // is in Range
    float wrapAngle(float Angle);                    // Angle maping to +- 180°
};

extern Utils Util;
