#pragma once
#include <Arduino.h>

#include "enum.h"
#include "params.h"

#include <BNO055.h>

class systemlib
{
public:
    void initialize();

    void calibrateBNO( void );
    float getOrbitDirection( void );

private:

    BNO055 BNO = BNO055(BNO_ADDRESS, &I2C);
    float BNO_calibration = 0.0f;
    float BNO_eulerZ = 0.0f;

    void readBNO( void );

    
};
