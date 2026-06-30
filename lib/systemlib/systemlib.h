#pragma once
#include <Arduino.h>

#include "enum.h"
#include "params.h"
#include "Utils.h"

#include <BNO055.h>

extern BNO055 BNO;
extern BNO055 BNO1;

class systemlib
{
public:
    void initialize();

    void readSensors( void );

    void calibrateBNO( void );
    float getOrbitDir( void );



    float BNO_eulerZ_test = 0.0f;

private:

    
    //BNO055 BNO1 = BNO055( 0x29, &I2C_BNO);
    float BNO_calibration = 0.0f;
    float BNO_eulerZ = 0.0f;
    

    void readBNO( void );

    
};
