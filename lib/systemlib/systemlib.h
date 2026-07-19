#pragma once
#include <Arduino.h>

#include "enum.h"
#include "params.h"
#include <Utils.h>

#include <BNO055.h>
#include <INA219.h>
#include <SRF08.h>
#include <ADC108S102.h>
#include <TCA9534DW.h>

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
    uint16_t US_vo = 0;
    uint16_t US_re = 0;
    uint16_t US_hi = 0;
    uint16_t US_li = 0;

    /* -------------- System -------------- */
    u_int16_t cycleTime = 0;
    u_int16_t cycleTime_us = 0;
    u_int16_t debugTimeout = 0;

    /* -------------- Sensor Funktion -------------- */
    void readBNO(void);
    void readUS(void);
    void readExpanders(void);

    float INA_filter[6] = {0, 0, 0, 0, 0, 0};

public:
    /* -------------- Ssytem Worker -------------- */
    void initialize();
    void readSensors(void);

    /* -------------- Communication -------------- */
    void communicationSend(void);                                                                                                       // setzt die Daten Automatisch und sendet sie
    void communicationSend(float positionX, float positionY, float ballAngle, float ballDistance, float goalAngle, float goalDistance); // sendet Benutzerdefinierte Daten

    /* -------------- Sensors -------------- */
    void calibBNO(void);

    /* -------------- Aktors -------------- */

    /* -------------- give -------------- */
    float giveOrbitDir(void) const { return BNO_OrbitDir; };            // Orbital Ausrichtung
    u_int16_t giveCycleTime(void) const { return cycleTime; };          // Zyklische Messung der Laufzeit in ms
    u_int16_t giveCycleTime_us(void) const { return cycleTime_us; };    // Zyklische Messung der Laufzeit in us
    RobotDataCom giveOtherRobotData(void) const { return otherRobot; }; // Daten des anderen Roboter
    float giveMainVoltage(void);
    float giveMotorVoltage(void);
    float giveMainCurrent(void); 
    float giveMotorCurrent(void); 
    uint16_t giveUS_vo(void) const { return US_vo; };
    uint16_t giveUS_re(void) const { return US_re; };
    uint16_t giveUS_hi(void) const { return US_hi; };
    uint16_t giveUS_li(void) const { return US_li; };

    /* -------------- set -------------- */
    void setDebugTimeout(u_int16_t timeout_ms) { debugTimeout = timeout_ms; }
    void setDBGSys(void);
    void setDBGChannel(u_int8_t Channel, float Value); // 0-20 Frei für eigenen Quatsch
};
