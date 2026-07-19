/*
 * IR-Ring Library v0.2.0
 *
 * https://git.markdorf-robotics.de/libraries/teensy-40/ir-ring
 *
 * Author: Rafael Günther <mail@rafael-guenther.de> (2025-2026)
 */

#pragma once

#include <cstdint>

#include "bus-interface/i2c_interface.h"

class IR_ring
{
public:
  explicit IR_ring(const Device_handle& handle) : _handle(handle) {};

  struct Fw_version { int major; int minor; int patch; };
  enum class Hw_version : uint8_t {unknown = 0, v1_0 = 1, v1_1 = 2};

  [[nodiscard]] uint8_t read_sensor_id() const;
  [[nodiscard]] Fw_version read_fw_version() const;
  [[nodiscard]] Hw_version read_hw_version() const;

  [[nodiscard]] uint16_t read_error_code() const;

  [[nodiscard]] bool read_ball_visible() const;
  [[nodiscard]] float read_ball_angle() const;
  [[nodiscard]] uint16_t read_ball_distance() const;

  [[nodiscard]] uint16_t read_raw_value(uint8_t sensor_number) const;
  void read_raw_values(uint16_t (&raw)[16]) const;

  [[nodiscard]] uint16_t read_calibrated_value(uint8_t sensor_number) const;
  void read_calibrated_values(uint16_t (&calib)[16]) const;

  [[nodiscard]] uint16_t read_gain(uint8_t sensor_number) const;
  void read_gains(uint16_t (&gains)[16]) const;
  void write_gain(uint8_t sensor_number, uint16_t gain) const;
  void write_gains(const uint16_t (&gains)[16]) const;

  [[nodiscard]] int16_t read_offset(uint8_t sensor_number) const;
  void read_offsets(int16_t (&offsets)[16]) const;
  void write_offset(uint8_t sensor_number, int16_t offset) const;
  void write_offsets(const int16_t (&offsets)[16]) const;

  [[nodiscard]] uint16_t read_ball_visibility_threshold() const;
  void write_ball_visibility_threshold(uint16_t ball_visibility_threshold) const;

  [[nodiscard]] uint16_t read_ir_sensor_mask() const;
  void write_ir_sensor_mask(uint16_t ir_sensor_mask) const;

  void save_calibration(bool save_gains=true, bool save_offsets=true, bool save_visibility_th=true) const;
  void load_calibration(bool save_gains=true, bool save_offsets=true, bool save_visibility_th=true) const;
  void reset_calibration(bool save_gains=true, bool save_offsets=true, bool save_visibility_th=true) const;

  [[nodiscard]] uint8_t read_tssp() const;
  void read_tssp(bool (&tssp)[8]) const;

  [[nodiscard]] uint8_t read_user_input() const;

  [[nodiscard]] uint8_t read_sensor_power_supply() const;
  void write_sensor_power_supply(bool enable_ir_a, bool enable_ir_b, bool enable_tssp) const;

  enum class Cmd_opcode : uint8_t { control_eeprom = 0x10 };

private:
  const Device_handle _handle;
};