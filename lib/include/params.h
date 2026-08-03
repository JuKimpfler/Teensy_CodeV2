#pragma once
#include <Arduino.h>

#define I2C_BNO Wire
#define I2C_IR Wire
#define I2C_SW Wire
#define I2C_US Wire
#define I2C_RADAR Wire

#define BNO_ADDRESS 0x28

static constexpr uint8_t RADAR_NUM_SENSORS = 4;
static constexpr uint8_t RADAR_SENSOR_ADDRS[RADAR_NUM_SENSORS] = { 0x51, 0x52, 0x53, 0x54 };
static constexpr int8_t  RADAR_WAKE_PIN = -1;
static constexpr int8_t  RADAR_INT_PIN  = -1;

