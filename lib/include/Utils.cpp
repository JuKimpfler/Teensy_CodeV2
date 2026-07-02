#include "Utils.h"
#include <math.h>

Utilslib Util;

bool Utilslib::Range(float Value , float min , float max){
    return (Value >= min && Value <= max);
}

float Utilslib::AMap( float Angle ){
    //switch to 0, 360 for modulo operation, then back to +- 180
    return fmod(Angle + 180.0f, 360.0f) - 180.0f;
}