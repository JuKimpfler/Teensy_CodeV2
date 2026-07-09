#pragma once
#include <Arduino.h>

#include "enum.h"
#include "params.h"
#include <Utils.h>

#include <BNO055.h>

#include <commlib.h>
#include <PDS.h>


extern BNO055 BNO;

class systemlib
{
private:
    RobotDataCom ownRobot;
    RobotDataCom otherRobot;

    /* -------------- Sensor Var -------------- */
    float BNO_calibration = 0.0f;
    float BNO_eulerZ = 0.0f;
    float BNO_OrbitDir = 0.0f;

    /* -------------- System -------------- */
    u_int16_t cycleTime = 0;
    u_int16_t cycleTime_us = 0;
    u_int16_t debugTimeout = 0;

    /* -------------- Sensor Funktion -------------- */
    void readBNO(void);

public:
    /* -------------- Ssytem Worker -------------- */
    void initialize();
    void readSensors(void);

    /* -------------- Communication -------------- */
    void send(void);
    void send(float positionX, float positionY, float ballAngle, float ballDistance, float goalAngle, float goalDistance);

    /* -------------- Sensors -------------- */
    void calibBNO(void);

    /* -------------- Aktors -------------- */

    /* -------------- give -------------- */
    float giveOrbitDir(void) const { return BNO_OrbitDir; };            // Orbital Ausrichtung
    u_int16_t giveCycleTime(void) const { return cycleTime; };          // Zyklische Messung der Laufzeit in ms
    u_int16_t giveCycleTime_us(void) const { return cycleTime_us; };    // Zyklische Messung der Laufzeit in us
    RobotDataCom giveOtherRobotData(void) const { return otherRobot; }; // Daten des anderen Roboter

    /* -------------- set -------------- */
    void setDebugTimeout(u_int16_t timeout_ms) { debugTimeout = timeout_ms; }
    void setDBGSys(void);
    void setDBGChannel(u_int8_t Channel, float Value);
    void setCommunication(void);
};
