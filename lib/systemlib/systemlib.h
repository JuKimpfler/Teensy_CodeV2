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
#include <Pathfinder.h>
#include <MotionProfile.h>
#include <RoutePlanner.h>
#include <SensorLink.h>

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

/// Ergebnis von systemlib::calculatePosition() — Feldposition mit Guetemass.
struct PositionData
{
    float X;
    float Y;
    float Certainty;
    bool XValid;
    bool YValid;
    bool AtGoal;
};

struct RobotVelocity
{
    float vx_field; // cm/s
    float vy_field; // cm/s
};

// ArrayPoint und ArrayState stehen in lib/MotionProfile/MotionProfile.h

/// Zustand des Links zum ESP32-S3
enum LinkState : uint8_t
{
    LinkStateOk = 0,       ///< ESP lebt, Handshake fertig, Parameter passen
    LinkStateNoEsp = 1,    ///< ESP meldet sich nicht (oder nicht mehr)
    LinkStateMismatch = 2, ///< Version oder Parameter-Pruefsumme passen nicht: ESP-Daten werden nicht benutzt
    LinkStateNotUsed = 3   ///< kein Geraet am ESP (Quelle Teensy) und kein ESP gesehen
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
    float Drib_I = 0.0f;               ///< Dribbler-Strom [mA], 10 Hz
    bool _headingValid = true;         ///< Winkel liegt vor (Link lebt bzw. BNO055 direkt gelesen)
    bool _headingCalibPending = false; ///< Nullpunkt wird bei der naechsten gueltigen Winkelmessung gesetzt
    float Link_State = 0.0f;           ///< LinkState als Zahl (PDS)
    float ESP_MaxLoop_us = 0.0f;       ///< groesste Schleifenzeit des ESP [us]
    float ESP_I2cErr = 0.0f;           ///< I2C-Fehler des ESP
    float Link_CrcErr = 0.0f;          ///< CRC-Fehler am Teensy
    float Link_Lost = 0.0f;            ///< verlorene Rahmen (Luecken der Sequenznummer)
    float Link_Delay_us = 0.0f;        ///< Laufzeit des besten Zeitabgleichs [us]
    float Heading_Age_ms = 0.0f;       ///< Alter der Winkelmessung [ms] (Quelle Link)
    bool _motorPowerEnabled = true;    ///< VMOT_EN; false = Motorversorgung aus, alle Antriebe stehen
    float CPU_Temp = 0.0f;             ///< Chiptemperatur [Grad C], 1 Hz
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

    int8_t LineWinkel = 0;
    int8_t LineDepth = 0;
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
    void readBNO(void);
    void readIR(void);
    void readRadar(void); ///< Quelle Teensy: Radar.update(); Quelle ESP: Werte vom Link uebernehmen
    void updateLinkDiagnostics(void);
    void readExpanders(void);

    /* -------------- Debug -------------- */
    /// Bindet die Systemwerte an die festen PDS-Kanaele 0..25.
    /// Wird von initialize() aufgerufen, siehe docs/PDS_Belegung.md.
    void setupPDS(void);

    /* -------------- Motor ----------------- */
    void updateMotors(void);

    /* -------------- Position -------------- */
    float goalOffset(float x_cm, float &sureness) const;

    /* -------------- Fahren -------------- */
    MotionInput giveMotionInput(void) const;            ///< Position, Winkel und Zeit dieses Zyklus
    void applyMotionCommand(const MotionCommand &cmd);  ///< Fahrbefehl -> Kursregler und Raeder

public:
    /// Zielanfahrt, Speed-Pattern und Punkt-Liste, siehe lib/MotionProfile/MotionProfile.md
    MotionProfile Profile;

    /// Pfadplanung auf cm-Ebene ueber AStar, siehe lib/RoutePlanner/RoutePlanner.md
    RoutePlanner Planner;

    /// Alle Radarsensoren. Zugriff ueber die Einbauposition: Radar[RadarVo].Dist
    RadarArray Radar;

    MotorData MoDataVL;
    MotorData MoDataVR;
    MotorData MoDataHL;
    MotorData MoDataHR;
    MotorData MoDataDR;

    ADRCConfig cfgA;
    PIDConfig cfgP;

    uint16_t line_array[LINE_NUM_SENSORS];
    bool line_bool_array[LINE_NUM_SENSORS];

    /// Letztes Ergebnis von calculatePosition(), wird in readSensors() aktualisiert.
    PositionData Position;

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
    void moveToPoint(uint16_t X, uint16_t Y, uint8_t Speed, int16_t Andrehwinkel);
    void moveToPoint(uint16_t X, uint16_t Y, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, int16_t Andrehwinkel);
    void resetMoveProfile(void) { Profile.resetProfile(); };        ///< naechster moveToPoint()-Aufruf startet das Speed-Pattern von vorn
    bool giveMoveArrived(void) const { return Profile.giveArrived(); };   ///< true = Ziel von moveToPoint() erreicht (Zielkreis)
    float giveMoveProgress(void) const { return Profile.giveProgress(); }; ///< Fortschritt 0..1 des aktuellen Speed-Patterns

    /* -------------- Punkt-Array -------------- */
    bool setPointArray(const ArrayPoint *points, uint8_t count, bool loop = false); ///< Liste ersetzen (unveraendert = kein Neustart)
    template <size_t N>
    bool setPointArray(const ArrayPoint (&points)[N], bool loop = false) { return setPointArray(points, (uint8_t)N, loop); } ///< Anzahl automatisch
    bool addPointToArray(uint16_t X, uint16_t Y, int16_t Andrehwinkel, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, uint16_t HoldMs = 0);
    bool addPointToArray(const ArrayPoint &point) { return Profile.addPointToArray(point); } ///< Punkt hinten anhaengen
    void resetArray(void){ Profile.resetArray();}; ///< Liste leeren, Zustand Idle
    void startArray(bool restart = false){ Profile.startArray(restart); }; ///< beim ersten Punkt starten (laeuft sie schon: nichts)
    void pauseArray(void){ Profile.pauseArray(); }; ///< anhalten
    void resumeArray(void){ Profile.resumeArray(); }; ///< nach pauseArray() fortsetzen
    void updateArray(void){ applyMotionCommand(Profile.updateArray(giveMotionInput())); }; ///< jeden Zyklus aufrufen
    ArrayState giveArrayState(void) const { return Profile.giveArrayState(); };
    uint8_t giveArrayIndex(void) const { return Profile.giveArrayIndex(); }; ///< aktueller Zielpunkt (0-basiert)
    uint8_t giveArrayCount(void) const { return Profile.giveArrayCount(); }; ///< Anzahl Punkte

    /* -------------- Pfadplanung (A*) -------------- */
    void setPathObstacles(int16_t ballX, int16_t ballY, int16_t robotX, int16_t robotY); ///< Hindernisse [cm], -1 = keins
    void moveToPointAvoid(uint16_t X, uint16_t Y, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, int16_t Andrehwinkel);
    void resetPathPlan(void);              ///< Planung, Punkt-Array und Fahrprofil verwerfen
    bool givePathFound(void) const { return Planner.giveFound(); };        ///< A* hat einen Weg gefunden
    uint8_t givePathWaypoints(void) const { return Planner.giveWaypoints(); }; ///< Anzahl Wegpunkte der Route
    uint32_t givePathTimeUs(void) const { return Planner.giveTimeUs(); };   ///< Rechenzeit der letzten Planung [us]
    void moveDir(int16_t Dir, uint8_t Speed, float Andrehwinkel);
    void setMotorData(uint8_t ID, uint8_t Speed, bool Dir, bool Enable, bool Break);
    void setMotorData(uint8_t ID, int8_t Speed, bool Enable, bool Break);
    void setOmniWheelVek(float vx, float vy, float omega);
    void setOmniWheelDir(float directionDeg, uint8_t speedPercent, float omega);

    /* -------------- Calculate -------------- */
    void updateLine();
    void updateHeadingController(void);
    PositionData calculatePosition(uint32_t current_time_ms,
                                   float vx_field,
                                   float vy_field,
                                   bool use_radar = true,
                                   bool use_odometry = true);
    void getOmniWheelVek(float wVL, float wVR, float wHL, float wHR, float &vx, float &vy, float &omega);
    float convertMotorRpmToWheelSpeedCmS(float motor_rpm);
    RobotVelocity calculateFieldVelocityFromMotorRpm(const float motor_rpm[4], float heading_deg);

    /* -------------- give -------------- */
    float giveOrbitDir(void) const { return BNO_OrbitDir; };            // Orbital Ausrichtung
    uint16_t giveCycleTime(void) const { return cycleTime; };           // Zyklische Messung der Laufzeit in ms
    uint16_t giveCycleTime_us(void) const { return cycleTime_us; };     // Zyklische Messung der Laufzeit in us
    RobotDataCom giveOtherRobotData(void) const { return otherRobot; }; // Daten des anderen Roboter
    float giveCpuTemp(void) const { return CPU_Temp; }                  ///< Chiptemperatur [Grad C], 1 Hz nachgefuehrt
    float giveMainVoltage(void);
    float giveMotorVoltage(void);
    float giveMainCurrent(void);
    float giveMotorCurrent(void);
    float giveDribblerCurrent(void);
    void setMotorPower(bool on) { _motorPowerEnabled = on; } ///< Motorversorgung (VMOT_EN) und Freigabe aller Antriebe
    bool giveMotorPower(void) const { return _motorPowerEnabled; }
    bool giveHeadingValid(void) const { return _headingValid; } ///< false: kein Winkel, der Kursregler dreht nicht
    LinkState giveLinkState(void) const;                        ///< Zustand des ESP-Links
    bool giveSwitch2(void) const;                               ///< SWITCH2 am ESP (nur mit laufendem Link)
    bool setServo(uint16_t microseconds) { return giveEspLink().sendServo(microseconds); }
    bool playPiezo(uint16_t hz, uint16_t ms) { return giveEspLink().sendPiezo(hz, ms); }
    SensorLink &giveEspLink(void);                                                                     ///< Zustand des Links zum ESP32-S3
    float giveBallAngle(void) const { return IR_Angle; };                                              // TODO: Implementieren
    float giveBallDistance(void) const { return IR_Distance; };                                        // TODO: Implementieren
    float giveRawBallAngle(void) const { return IR_rawAngle; };                                        // TODO: Implementieren
    float giveRawBallDistance(void) const { return IR_rawDistance; };                                  // TODO: Implementieren
    uint16_t giveRawSensorData(uint8_t sensorNumber) const { return IR_rawSensorData[sensorNumber]; }; // currently disabled -> change in readIR() to enable it
    void giveRawSensorData(uint16_t (&rawSensorData)[16]) const
    {
        for (uint8_t i = 0; i < 16; i++)
        {
            rawSensorData[i] = IR_rawSensorData[i];
        }
    }; // currently disabled -> change in readIR() to enable it
    void giveTSSPData(bool (&TSSPData)[8]) const
    {
        for (uint8_t i = 0; i < 8; i++)
        {
            TSSPData[i] = IR_TSSP[i];
        }
    }; // currently disabled -> change in readIR() to enable it
    bool giveTSSPData(uint8_t sensorNumber) const { return IR_TSSP[sensorNumber]; }; // currently disabled -> change in readIR() to enable it
    float giveHeadingController(void);                                               // BNO has to be setup allready
    void giveButtonStates(uint8_t (&buttonStates)[5]) const
    {
        for (uint8_t i = 0; i < 5; i++)
        {
            buttonStates[i] = fancyBTNstates[i];
        }
    }; // currently disabled -> change in readExpanders() to enable it
    RadarData giveRadar(uint8_t pos) const { return Radar[pos]; };           // Radar-Messwerte, pos = RadarVo/Li/Hi/Re
    bool giveRadarCon(uint8_t pos) const { return Radar.giveCon(pos); };     // antwortet der Radarsensor?
    uint32_t giveRadarAge(uint8_t pos) const { return Radar.giveAge(pos); }; // Alter des Radarwerts in ms
    PositionData givePosition(void) const { return Position; };              // zuletzt berechnete Feldposition + Certainty
    float givePositionCertainty(void) const { return Position.Certainty; };  // 0 = unbrauchbar ... 1 = sicher
    bool giveExpander(uint8_t expnum, uint8_t portnum);
    bool giveExpSwitch(uint8_t switchNumber) const { return SWI[switchNumber]; };
    bool giveExpStartSwitch(void) const { return startSwitch; };
    bool giveRCJ(void) const { return RCJ; };
    bool giveRCJpresent(void) const { return RCJ_present; };
    bool giveStart(void);
    uint16_t giveADC(uint8_t Port, uint8_t Pin);
    int8_t giveLineAngle() { return LineWinkel; }
    int8_t giveLineDepth() { return LineDepth; }
    bool giveLinePresent() { return LinePresent; }

    /* -------------- set -------------- */
    void setDebugTimeout(uint16_t timeout_ms) { debugTimeout = timeout_ms; }
    void setHeading(float Heading);
    void setHeadingControllerConfig(void);
};
