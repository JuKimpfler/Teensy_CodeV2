#pragma once
#include <Arduino.h>

#include "enum.h"
#include "params.h"
#include <Utils.h>

#include <INA219.h>
#include <ADC108S102.h>
#include <TCA9534DW.h>
#include <ir_ring.h>
#include "SPI.h"
#include <fancyBTN.h>
#include <PDS.h>

#include <PID/PID.h>
#include <ADRC/ADRC.h>

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

/// Ergebnis von systemlib::calculatePosition() — Feldposition mit Guetemass.
struct PositionData {
    float X;
    float Y;
    float Certainty;
    bool  XValid;
    bool  YValid;
    bool  AtGoal;
};

struct RobotVelocity {
    float vx_field; // cm/s
    float vy_field; // cm/s
};

/// Ein Zielpunkt der Punkt-Liste, siehe systemlib::setPointArray().
struct ArrayPoint {
    uint16_t X;            ///< Ziel X [cm]
    uint16_t Y;            ///< Ziel Y [cm]
    int16_t  Andrehwinkel; ///< Soll-Heading [Grad]
    uint8_t  SpeedPattern; ///< SPattern_* aus params.h
    uint8_t  MaxSpeed;     ///< Speed [%] bei Pattern-Wert 1
    uint8_t  MinSpeed;     ///< Speed [%] bei Pattern-Wert 0
    uint16_t HoldMs;       ///< Wartezeit nach Erreichen des Punkts [ms]
    uint8_t  PassCm;       ///< >0: Durchfahrpunkt, gilt ab diesem Abstand [cm] als erreicht (kein Halt); 0 = anhalten
};

/// Zustand der Punkt-Liste, siehe systemlib::updateArray().
enum ArrayState : uint8_t {
    ArrayIdle = 0,  ///< nicht gestartet / leer
    ArrayRunning,   ///< faehrt die Punkte ab
    ArrayPaused,    ///< angehalten, Fortschritt bleibt
    ArrayFinished   ///< letzter Punkt erreicht
};

class systemlib
{
private:

    /* -------------- Sensor Var -------------- */
    float Akku_V = 0.0f;
    float Akku_I = 0.0f;
    float Motor_V = 0.0f;
    float Motor_I = 0.0f;
    float IR_Angle = 0.0f;
    float IR_Distance = 0.0f;
    float IR_rawAngle = 0.0f;
    float IR_rawDistance = 0.0f;
    uint16_t IR_rawSensorData[16] = {0};
    bool IR_TSSP[8] = {false};
    uint16_t IR_Gains[16] = {0};
    int16_t IR_Offsets[16] = {9999};
    float INA_filter[6] = {0, 0, 0, 0, 0, 0};

    // Esp Data 
    PositionData Position;
    PositionData Ball_Position; 
    int16_t BNO_OrbitDir=0;

    int8_t LineWinkel = 0;
    int16_t LineDepth = 0;
    bool LinePresent = 0;

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
    float uptime_s = 0.0f; ///< Laufzeit seit dem Start [s], PDS-Kanal 0

    /* -------------- Sensor Funktion -------------- */
    void readIR(void);
    void readExpanders(void);

    /* -------------- Debug -------------- */
    /// Bindet die Systemwerte an die festen PDS-Kanaele 0..25.
    /// Wird von initialize() aufgerufen, siehe docs/PDS_Belegung.md.
    void setupPDS(void);

    /* -------------- Motor ----------------- */
    void updateMotors(void);

    float feld_x = POS_FIELD_X_CM; ///< Feldbreite [cm], siehe params.h
    float feld_y = POS_FIELD_Y_CM; ///< Feldlaenge [cm], siehe params.h

public:

    MotorData MoDataVL;
    MotorData MoDataVR;
    MotorData MoDataHL;
    MotorData MoDataHR;
    MotorData MoDataDR;

    ADRCConfig cfgA;
    PIDConfig cfgP;

    uint16_t line_array[48];
    bool line_bool_array[48];

    /* -------------- Ssytem Worker -------------- */
    void initialize();
    void readSensors(void);

    /* -------------- Calibrate -------------- */
    void writeIRCalibration(uint8_t sensorNumber, int16_t offset, int16_t gain);
    void writeIRCalibration(const int16_t (&offsets)[16], const uint16_t (&gains)[16]);
    void calibIR(void);
    void calibBNO(void);

    /* -------------- enable -------------- */
    void enableVMOT(bool enable = true);
    void disableVMOT() {enableVMOT(false);};
    void enableVDRIB(bool enable = true);
    void disableVDRIB() {enableVDRIB(false);};
    void enableLIGHTBARRIER(bool enable = true);
    void disableLIGHTBARRIER() {enableLIGHTBARRIER(false);};
    void enableLINE(bool enable = true);
    void disableLINE() {enableLINE(false);};
    void enableI2C(bool enable = true);
    void disableI2C() {enableI2C(false);};

    /* -------------- Aktors -------------- */
    void setMotorData(MotorData Data);
    MotorData giveMotorData(uint8_t ID);
    void setupHeadingController(bool useADRC = false);
    void resetHeadingController(void);
    void moveToPoint(uint16_t X, uint16_t Y, uint8_t Speed,int16_t Andrehwinkel);
    void moveToPoint(uint16_t X, uint16_t Y, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, int16_t Andrehwinkel);
    void resetMoveProfile(void);             ///< naechster moveToPoint()-Aufruf startet das Speed-Pattern von vorn
    bool giveMoveArrived(void) const;        ///< true = Ziel von moveToPoint() erreicht (Zielkreis)
    float giveMoveProgress(void) const;      ///< Fortschritt 0..1 des aktuellen Speed-Patterns

    /* -------------- Punkt-Array -------------- */
    bool setPointArray(const ArrayPoint *points, uint8_t count, bool loop = false); ///< Liste ersetzen (unveraendert = kein Neustart)
    template <size_t N>
    bool setPointArray(const ArrayPoint (&points)[N], bool loop = false) { return setPointArray(points, (uint8_t)N, loop); } ///< Anzahl automatisch
    bool addPointToArray(uint16_t X, uint16_t Y, int16_t Andrehwinkel, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, uint16_t HoldMs = 0);
    bool addPointToArray(const ArrayPoint &point);                                  ///< Punkt hinten anhaengen
    void resetArray(void);                                                          ///< Liste leeren, Zustand Idle
    void startArray(bool restart = false);                                          ///< beim ersten Punkt starten (laeuft sie schon: nichts)
    void pauseArray(void);                                                          ///< anhalten
    void resumeArray(void);                                                         ///< nach pauseArray() fortsetzen
    void updateArray(void);                                                         ///< jeden Zyklus aufrufen
    ArrayState giveArrayState(void) const;
    uint8_t giveArrayIndex(void) const;                                             ///< aktueller Zielpunkt (0-basiert)
    uint8_t giveArrayCount(void) const;                                             ///< Anzahl Punkte

    /* -------------- Pfadplanung (A*) -------------- */
    void setPathObstacles(int16_t ballX, int16_t ballY, int16_t robotX, int16_t robotY); ///< Hindernisse [cm], -1 = keins
    void moveToPointAvoid(uint16_t X, uint16_t Y, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, int16_t Andrehwinkel);
    void resetPathPlan(void);                                                       ///< Planung, Punkt-Array und Fahrprofil verwerfen
    bool givePathFound(void) const;                                                 ///< A* hat einen Weg gefunden
    uint8_t givePathWaypoints(void) const;                                          ///< Anzahl Wegpunkte der Route
    uint32_t givePathTimeUs(void) const;                                            ///< Rechenzeit der letzten Planung [us]
    void moveDir(int16_t Dir,uint8_t Speed,float Andrehwinkel);
    void setMotorData(uint8_t ID, uint8_t Speed, bool Dir, bool Enable, bool Break);
    void setMotorData(uint8_t ID, int8_t Speed, bool Enable, bool Break);
    void setOmniWheelVek(float vx, float vy, float omega);
    void setOmniWheelDir(float directionDeg, uint8_t speedPercent, float omega);

    /* -------------- Calculate -------------- */
    void updateLine();
    void updateHeadingController(void);

    /* -------------- give -------------- */
    int16_t giveOrbitDir();
    uint16_t giveCycleTime(void) const { return cycleTime; };           // Zyklische Messung der Laufzeit in ms
    uint16_t giveCycleTime_us(void) const { return cycleTime_us; };     // Zyklische Messung der Laufzeit in us
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
    void giveButtonStates(uint8_t (&buttonStates)[5]) const {for (uint8_t i = 0; i < 5; i++){buttonStates[i] = fancyBTNstates[i];}}; // BTN1..BTN5 vom SwitchBoard, fancyBTN-Statuscodes
    bool giveExpander(uint8_t expnum, uint8_t portnum);
    bool giveExpSwitch(uint8_t switchNumber) const { return SWI[switchNumber]; };
    bool giveExpStartSwitch(void) const { return startSwitch; };
    bool giveRCJ(void) const { return RCJ; };
    bool giveRCJpresent(void) const { return RCJ_present; };
    bool giveStart(void);
    uint16_t giveADC(uint8_t Port, uint8_t Pin);
    int8_t giveLineAngle() { return LineWinkel; }
    int16_t giveLineDepth() { return LineDepth; }
    bool giveLinePresent() { return LinePresent; }
    float giveCam1Angle();
    float giveCam2Angle();

    /* -------------- set -------------- */
    void setDebugTimeout(uint16_t timeout_ms) { debugTimeout = timeout_ms; }
    void setHeading(float Heading);
    void setHeadingControllerConfig(void);
};
