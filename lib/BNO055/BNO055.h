#ifndef _BNO055_H_
#define _BNO055_H_

#include "../Wire/Wire.h"

// in Taktik oä. auslagern evtl.
#define EEPROM_BNO_CALIB 250
#define EEPROM_BNO_CALIB_SIZE 22

//#define CALIBRATION_ENABLED
#define BNO055_SENSOR_FUSION_ALGO 5

//
enum e_bnoSysStatus {
  systemIdle,
  systemError,
  initializingPeripherals,
  systemInitialization,
  executingSelftest,
  sensorFusionAlgorithmRunning,
  systemRunningWithoutFusionAlgorithm
};

#define BNO055_CHIP_ID 0xA0

// Register
#define REG_CHIP_ID 0x00
#define REG_ACC_ID 0x01
#define REG_MAG_ID 0x02
#define REG_GYR_ID 0x03
#define REG_SW_REV_ID_LSB 0x04
#define REG_SW_REV_ID_MSB 0x05
#define REG_BL_REV_ID 0x06
#define REG_PAGE_ID 0x07
#define REG_ACC_DATA_X_LSB 0x08
// ...
// TODO: ergänzen
// ...
#define REG_TEMP 0x34
#define REG_CALIB_STAT 0x35
#define REG_ST_RESULT 0x36
#define REG_INT_STA 0x37
#define REG_SYS_CLK_STATUS 0x38
#define REG_SYS_STATUS 0x39
#define REG_SYS_ERR 0x3A
#define REG_UNIT_SEL 0x3B
#define REG_OPR_MODE 0x3D
#define REG_SYS_TRIGGER 0x3F

// Operation Modes
//Config Mode
#define OPR_MODE_CONFIGMODE 0x00

//Non-Fusion Mode
#define OPR_MODE_ACCONLY 0x01
#define OPR_MODE_MAGONLY 0x02
#define OPR_MODE_GYROONLY 0x03
#define OPR_MODE_ACCMAG 0x04
#define OPR_MODE_ACCGYRO 0x05
#define OPR_MODE_MAGGYRO 0x06
#define OPR_MODE_AMG 0x07

//Fusion Modes
#define OPR_MODE_IMU 0x08
#define OPR_MODE_COMPASS 0x09
#define OPR_MODE_M4G 0x0A
#define OPR_MODE_NDOF_FMC_OFF 0x0B
#define OPR_MODE_NDOF 0x0C

#define DEFAULT_OPR_MODE OPR_MODE_IMU

class BNO055 {
public:
  BNO055(uint8_t addr, TwoWire* i2c_class) {
    this->addr = addr;
    this->cls = i2c_class;
  }
  void init();
  void reset();
#ifdef CALIBRATION_ENABLED
  void updateCalibrationData();
#endif
  int16_t calibData[11];
  uint8_t calibStatus();
  e_bnoSysStatus systemStatus();
  uint8_t requestSystemStatus();
  uint8_t systemError();
  /* Von Thilo auskommentiert
  void readBytes(uint8_t *data, uint8_t cnt)
  {
    cls->read(data, cnt);
  }*/
  // Ausrichtung in Quaternionen
  void quaData(float* data);
  uint8_t requestQuaData();
  float quaDataW();
  float quaDataX();
  float quaDataY();
  float quaDataZ();
  // Beschleunigungs Vektoren
  int16_t accDataX();
  int16_t accDataY();
  int16_t accDataZ();
  // Magnetfeld Vektoren
  int16_t magDataX();
  int16_t magDataY();
  int16_t magDataZ();
  // Gyro Vektoren
  int16_t gyrDataX();
  int16_t gyrDataY();
  int16_t gyrDataZ();
  uint8_t requestGyrDataZ();
  // Ausrichtung in Euler-Winkeln
  float eulHeading();
  float eulRoll();
  float eulPitch();

private:
  uint8_t calibDataByte[22];
  inline void writeReg(uint8_t reg, uint8_t value);
  inline int16_t getWord(uint8_t address);
  inline void getWords(uint8_t start_address, int16_t* data, uint8_t numWords);
  uint8_t addr;
  TwoWire* cls;
};

#endif
