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
#include <ir_ring.h>
#include "SPI.h"
#include <fancyBTN.h>
#include <RadarArray.h>

#include <commlib.h>
#include <PDS.h>

#include <PID/PID.h>
#include <ADRC/ADRC.h>

extern BNO055 BNO;

struct MotorData
{
    uint8_t ID;

    // Param 
    float Speed;
    bool Dir;
    bool Enable;
    bool Break;

    // Meassurements
    uint16_t Current;
    float Speed_real;
    bool Dir_real;

    float Speed_diff;
    bool is_running;
};

class systemlib
{
private:
    RobotDataCom ownRobot;
    RobotDataCom otherRobot;

    /* -------------- Sensor Var -------------- */
    float Akku_V = 0.0f;
    float Akku_I = 0.0f;
    float Motor_V = 0.0f;
    float Motor_I = 0.0f;
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
    uint16_t IR_rawSensorData[16] = {0};
    bool IR_TSSP[8] = {false};
    uint16_t IR_Gains[16] = {0};
    int16_t IR_Offsets[16] = {9999};
    float INA_filter[6] = {0, 0, 0, 0, 0, 0};

    bool SWI[5] = {false};
    bool RCJ = false;
    bool RCJ_Disable = false;
    bool RCJ_present = false;
    bool startSwitch = false;

    bool _useADRC = false;
    float _OrbitHeading = 0;
    float ADRC_Out = 0;
    float PID_Out = 0;

    uint8_t fancyBTNstates[5] = {0};
    /* -------------- System -------------- */
    uint16_t cycleTime = 0;
    uint16_t cycleTime_us = 0;
    uint16_t debugTimeout = 0;

    /* -------------- Sensor Funktion -------------- */
    void readBNO(void);
    void readIR(void);
    void readExpanders(void);

    /* -------------- Motor ----------------- */
    void updateMotors(void);

public:

    /// Alle Radarsensoren. Zugriff ueber die Einbauposition: Radar[RadarVo].Dist
    RadarArray Radar;

    MotorData MoDataVL;
    MotorData MoDataVR;
    MotorData MoDataHL;
    MotorData MoDataHR;
    MotorData MoDataDR;

    ADRCConfig cfgA;
    PIDConfig cfgP;

    float Regler_in = 1;

    /* -------------- Ssytem Worker -------------- */
    void initialize();
    void readSensors(void);

    /* -------------- Communication -------------- */
    void communicationSend(void);                                                                                                       // setzt die Daten Automatisch und sendet sie
    void communicationSend(float positionX, float positionY, float ballAngle, float ballDistance, float goalAngle, float goalDistance); // sendet Benutzerdefinierte Daten

    /* -------------- Calibrate -------------- */
    void calibBNO(void);
    void writeIRCalibration(uint8_t sensorNumber, int16_t offset, int16_t gain);
    void writeIRCalibration(const int16_t (&offsets)[16], const uint16_t (&gains)[16]);
    void calibIR(void);
    void calibRadar(void); ///< alle Radarsensoren neu konfigurieren + kalibrieren (blockiert)

    /* -------------- Aktors -------------- */
    void setMotorData(MotorData Data);
    MotorData giveMotorData(uint8_t ID);
    void setupHeadingController(bool useADRC = false);
    void resetHeadingController(void);
    void moveToPoint(uint16_t X, uint16_t Y, uint8_t Speed,int16_t Andrehwinkel);
    void moveToPoint(uint16_t X, uint16_t Y, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, int16_t Andrehwinkel);
    void moveDir(int16_t Dir,uint8_t Speed,float Andrehwinkel);
    void setMotorData(uint8_t ID, uint8_t Speed, bool Dir, bool Enable, bool Break);
    void setMotorData(uint8_t ID, int8_t Speed, bool Enable, bool Break);
    void setOmniWheelVek(float vx, float vy, float omega);
    void setOmniWheelDir(float directionDeg, uint8_t speedPercent, float omega);

    /* -------------- Calculate -------------- */
    void updateHeadingController(void);

    /* -------------- give -------------- */
    float giveOrbitDir(void) const { return BNO_OrbitDir; };            // Orbital Ausrichtung
    uint16_t giveCycleTime(void) const { return cycleTime; };           // Zyklische Messung der Laufzeit in ms
    uint16_t giveCycleTime_us(void) const { return cycleTime_us; };     // Zyklische Messung der Laufzeit in us
    RobotDataCom giveOtherRobotData(void) const { return otherRobot; }; // Daten des anderen Roboter
    float giveMainVoltage(void);
    float giveMotorVoltage(void);
    float giveMainCurrent(void);
    float giveMotorCurrent(void);
    float giveBallAngle(void) const { return IR_Angle; };                                              // TODO: Implementieren
    float giveBallDistance(void) const { return IR_Distance; };                                        // TODO: Implementieren
    float giveRawBallAngle(void) const { return IR_rawAngle; };                                        // TODO: Implementieren
    float giveRawBallDistance(void) const { return IR_rawDistance; };                                  // TODO: Implementieren
    uint16_t giveRawSensorData(uint8_t sensorNumber) const { return IR_rawSensorData[sensorNumber]; }; // currently disabled -> change in readIR() to enable it
    void giveRawSensorData(uint16_t (&rawSensorData)[16]) const {for (uint8_t i = 0; i < 16; i++){rawSensorData[i] = IR_rawSensorData[i];}}; // currently disabled -> change in readIR() to enable it
    void giveTSSPData(bool (&TSSPData)[8]) const {for (uint8_t i = 0; i < 8; i++){TSSPData[i] = IR_TSSP[i];}};                                                                               // currently disabled -> change in readIR() to enable it
    bool giveTSSPData(uint8_t sensorNumber) const { return IR_TSSP[sensorNumber]; }; // currently disabled -> change in readIR() to enable it
    float giveHeadingController(void); // BNO has to be setup allready
    void giveButtonStates(uint8_t (&buttonStates)[5]) const {for (uint8_t i = 0; i < 5; i++){buttonStates[i] = fancyBTNstates[i];}}; // currently disabled -> change in readExpanders() to enable it
    RadarData giveRadar(uint8_t pos) const { return Radar[pos]; };       // Radar-Messwerte, pos = RadarVo/Li/Hi/Re
    bool giveRadarCon(uint8_t pos) const { return Radar.giveCon(pos); }; // antwortet der Radarsensor?
    bool giveExpander(uint8_t expnum, uint8_t portnum);
    bool giveExpSwitch(uint8_t switchNumber) const { return SWI[switchNumber]; };
    bool giveExpStartSwitch(void) const { return startSwitch; };
    bool giveRCJ(void) const { return RCJ; };
    bool giveRCJpresent(void) const { return RCJ_present; };
    bool giveStart(void);

    /* -------------- set -------------- */
    void setDebugTimeout(uint16_t timeout_ms) { debugTimeout = timeout_ms; }
    void setHeading(float Heading);
    void setHeadingControllerConfig(void);
};
