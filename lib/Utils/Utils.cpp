#include "Utils.h"
#include <math.h>

Utils Util;

bool Utils::inRange(float Value, float min, float max)
{
    return (Value >= min && Value <= max);
}

float Utils::wrapAngle(float Angle)
{
    // Normalfall: Winkel liegt schon in (-180, 180] -> unveraendert zurueck, kein fmodf.
    // (Unterschied zur Modulo-Rechnung unten: nur deren Rundungsrauschen, < 7e-5 Grad.)
    if (Angle > -180.0f && Angle <= 180.0f)
    {
        return Angle;
    }

    //switch to 0, 360 for modulo operation, then back to +- 180
    return 180.0f - fmodf(fmodf(180.0f - Angle, 360.0f) + 360.0f, 360.0f);
}
