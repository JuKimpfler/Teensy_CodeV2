#pragma once
#include <Arduino.h>

#include "enum.h"
#include "params.h"
#include <Utils.h>

#include <BNO055.h>
#include <INA219.h>
#include <SRF08.h>
#include <ir_ring.h>

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
    float IR_Angle = 0.0f;
    float IR_Distance = 0.0f;
    float IR_rawAngle = 0.0f;
    float IR_rawDistance = 0.0f;
    uint16_t IR_rawSensorData[16] = {0.0f};
    bool IR_TSSP[8] = {false};
    uint16_t IR_Gains[16] = {0};
    int16_t IR_Offsets[16] = {9999};


    /* -------------- System -------------- */
    uint16_t cycleTime = 0;
    uint16_t cycleTime_us = 0;
    uint16_t debugTimeout = 0;

    /* -------------- Sensor Funktion -------------- */
    void readBNO(void);
    void readIR(void);
    void readUS(void);

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
    void writeIRCalibration(uint8_t sensorNumber, int16_t offset, int16_t gain);
    void writeIRCalibration(const int16_t (&offsets)[16], const uint16_t (&gains)[16]);
    void calibIR(void);

    /* -------------- Aktors -------------- */

    /* -------------- give -------------- */
    float giveOrbitDir(void) const { return BNO_OrbitDir; };            // Orbital Ausrichtung
    uint16_t giveCycleTime(void) const { return cycleTime; };          // Zyklische Messung der Laufzeit in ms
    uint16_t giveCycleTime_us(void) const { return cycleTime_us; };    // Zyklische Messung der Laufzeit in us
    RobotDataCom giveOtherRobotData(void) const { return otherRobot; }; // Daten des anderen Roboter
    float giveMainVoltage(void);
    float giveMotorVoltage(void);
    float giveMainCurrent(void); 
    float giveMotorCurrent(void); 
    uint16_t giveUS_vo(void) const { return US_vo; };
    uint16_t giveUS_re(void) const { return US_re; };
    uint16_t giveUS_hi(void) const { return US_hi; };
    uint16_t giveUS_li(void) const { return US_li; };
    float giveBallAngle(void) const { return IR_Angle; }; // TODO: Implementieren
    float giveBallDistance(void) const { return IR_Distance; };  // TODO: Implementieren
    float giveRawBallAngle(void) const { return IR_rawAngle; }; // TODO: Implementieren
    float giveRawBallDistance(void) const { return IR_rawDistance; }; // TODO: Implementieren
    uint16_t giveRawSensorData(uint8_t sensorNumber) const { return IR_rawSensorData[sensorNumber]; }; // currently disabled -> change in readIR() to enable it
    void giveRawSensorData(uint16_t (&rawSensorData)[16]) const { for (uint8_t i = 0; i < 16; i++) rawSensorData[i] = IR_rawSensorData[i]; };// currently disabled -> change in readIR() to enable it
    void giveTSSPData(bool (&TSSPData)[8]) const { for (uint8_t i = 0; i < 8; i++) TSSPData[i] = IR_TSSP[i]; }; // currently disabled -> change in readIR() to enable it
    bool giveTSSPData(uint8_t sensorNumber) const { return IR_TSSP[sensorNumber]; }; // currently disabled -> change in readIR() to enable it

    /* -------------- set -------------- */
    void setDebugTimeout(uint16_t timeout_ms) { debugTimeout = timeout_ms; }
    void setDBGSys(void);
    void setDBGChannel(uint8_t Channel, float Value); // 0-20 Frei für eigenen Quatsch
};
