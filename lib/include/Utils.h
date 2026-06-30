#pragma once
#include <Arduino.h>

class Utilslib
{
    public:
        bool Range(float Value , float min , float max); // is in Range
        float AMap( float Angle ); // Angle maping to +- 180°
};

extern Utilslib Util;