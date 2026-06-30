#pragma once
#include <Arduino.h>

#include "enum.h"
#include "params.h"
#include "Utils.h"

#include <BNO055.h>
#include <PDS.h>

class systemlib
{
private:

    /* -------------- Sensor Var -------------- */
    float BNO_calibration = 0.0f;
    float BNO_eulerZ = 0.0f;
    float BNO_OrbitDir = 0.0f;

    /* -------------- System -------------- */
    u_int16_t cycleTime = 0;
    u_int16_t cycleTime_us = 0;
    u_int16_t debugTimeout = 0;
    
    /* -------------- Sensor Funktion -------------- */
    void readBNO( void );


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
    u_int16_t giveCycleTime_us( void ) const { return cycleTime_us;  }; // Zyklische Messung der Laufzeit in us

    /* -------------- set -------------- */
    void setDebugTimeout( u_int16_t timeout_ms ) { debugTimeout = timeout_ms; }
    void setDBGSys(); // Setzen der System Channels des DBG Systems
    void setDBGChannel( u_int8_t Channel , float Value ); // Setzen der verfügbaren DBG Channels (0-20)
};
