#pragma once
#include <Arduino.h>

#include "enum.h"
#include "params.h"
#include "Utils.h"

#include <BNO055.h>

extern BNO055 BNO;

class systemlib
{
public:

    /* -------------- Ssytem Worker -------------- */
    void initialize();
    void readSensors( void );

    /* -------------- Sensors -------------- */
    void calibBNO( void );

    /* -------------- Aktors -------------- */

    /* -------------- give -------------- */
    float giveOrbitDir( void ) const { return BNO_OrbitDir;  }; // Orbital Ausrichtung
    u_int16_t giveCycleTime( void ) const { return cycleTime;  }; // Zyklische Messung der Laufzeit in ms

    /* -------------- set -------------- */

private:

    /* -------------- Sensor Var -------------- */
    float BNO_calibration = 0.0f;
    float BNO_eulerZ = 0.0f;
    float BNO_OrbitDir = 0.0f;

    /* -------------- System -------------- */
    u_int16_t cycleTime = 0;
    
    /* -------------- Sensor Funktion -------------- */
    void readBNO( void );
};
