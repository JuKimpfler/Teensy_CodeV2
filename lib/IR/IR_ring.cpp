//
// Created by Rafael Günther on 23.10.25.
//

#include "ir_ring.h"

#include "ir_ring_registers.h"

uint8_t IR_ring::read_sensor_id() const
{
  uint8_t sensor_id {};
  _handle.read_register<uint8_t, uint8_t>(static_cast<uint8_t>(Ir_ring_registers::sensor_id),sensor_id);

  return sensor_id;
}

IR_ring::Fw_version IR_ring::read_fw_version() const
{
  Fw_version v {};

  uint8_t buf[3];
  _handle.read_bytes (buf, 3);
  _handle.read_registers<uint8_t, uint8_t>(static_cast<uint8_t>(Ir_ring_registers::fw_version_major), buf, 3);

  v.major = buf[0];
  v.minor = buf[1];
  v.patch = buf[2];

  return v;
}

IR_ring::Hw_version IR_ring::read_hw_version() const
{
  uint8_t buf;
  _handle.read_register<uint8_t, uint8_t>(static_cast<uint8_t>(Ir_ring_registers::hardware_rev), buf);

  return static_cast<Hw_version>(buf);
}

uint16_t IR_ring::read_error_code() const
{
  uint16_t error_code {};
  _handle.read_register<uint8_t, uint16_t>(static_cast<uint8_t>(Ir_ring_registers::error_msb),error_code);

  return error_code;
}

bool IR_ring::read_ball_visible() const
{
  uint8_t buf;
  _handle.read_register<uint8_t, uint8_t>(static_cast<uint8_t>(Ir_ring_registers::ball_visible), buf);

  return buf;
}

float IR_ring::read_ball_angle() const
{
  int16_t ball_angle;
  _handle.read_register<uint8_t, int16_t>(static_cast<uint8_t>(Ir_ring_registers::ball_angle_msb),ball_angle);

  return static_cast<float>(ball_angle) / 180.F;
}

uint16_t IR_ring::read_ball_distance() const
{
  uint16_t ball_distance;
  _handle.read_register<uint8_t, uint16_t>(static_cast<uint8_t>(Ir_ring_registers::ball_distance_msb),ball_distance);

  return ball_distance;
}

uint16_t IR_ring::read_raw_value(uint8_t sensor_number) const
{
  const uint8_t i = (sensor_number - 1) % 16;
  const uint8_t addr = static_cast<uint8_t>(Ir_ring_registers::ir1_data_raw_msb) + 2 * i;
  uint16_t raw {};
  _handle.read_register<uint8_t, uint16_t>(addr,raw);

  return raw;
}

void IR_ring::read_raw_values(uint16_t (&raw)[16]) const
{
  _handle.read_registers<uint8_t, uint16_t>(static_cast<uint8_t>(Ir_ring_registers::ir1_data_raw_msb),raw,16);
}

uint16_t IR_ring::read_calibrated_value(uint8_t sensor_number) const
{
  const uint8_t i = (sensor_number - 1) % 16;
  const uint8_t addr = static_cast<uint8_t>(Ir_ring_registers::ir1_data_calib_msb) + 2 * i;
  uint16_t calib {};
  _handle.read_register<uint8_t, uint16_t>(addr,calib);

  return calib;
}

void IR_ring::read_calibrated_values(uint16_t (&calib)[16]) const
{
  _handle.read_registers<uint8_t, uint16_t>(static_cast<uint8_t>(Ir_ring_registers::ir1_data_calib_msb),calib,16);
}

uint16_t IR_ring::read_gain(uint8_t sensor_number) const
{
  const uint8_t i = (sensor_number - 1) % 16;
  const uint8_t addr = static_cast<uint8_t>(Ir_ring_registers::ir1_gain_msb) + 2 * i;
  uint16_t gain {};
  _handle.read_register<uint8_t, uint16_t>(addr,gain);

  return gain;
}

void IR_ring::read_gains(uint16_t (&gains)[16]) const
{
  _handle.read_registers<uint8_t, uint16_t>(static_cast<uint8_t>(Ir_ring_registers::ir1_gain_msb),gains,16);
}

void IR_ring::write_gain(uint8_t sensor_number, uint16_t gain) const
{
  const uint8_t i = (sensor_number - 1) % 16;
  const uint8_t addr = static_cast<uint8_t>(Ir_ring_registers::ir1_gain_msb) + 2 * i;
  _handle.write_register<uint8_t, uint16_t>(addr, gain);
}

void IR_ring::write_gains(const uint16_t (&gains)[16]) const
{
  _handle.write_registers<uint8_t, uint16_t>(static_cast<uint8_t>(Ir_ring_registers::ir1_gain_msb), gains, 16);
}

int16_t IR_ring::read_offset(uint8_t sensor_number) const
{
  const uint8_t i = (sensor_number - 1) % 16;
  const uint8_t addr = static_cast<uint8_t>(Ir_ring_registers::ir1_offset_msb) + 2 * i;
  int16_t offset {};
  _handle.read_register<uint8_t, int16_t>(addr, offset);

  return offset;
}

void IR_ring::read_offsets(int16_t (&offsets)[16]) const
{
  _handle.read_registers<uint8_t, int16_t>(static_cast<uint8_t>(Ir_ring_registers::ir1_offset_msb), offsets,16);
}

void IR_ring::write_offset(uint8_t sensor_number, int16_t offset) const
{
  const uint8_t i = (sensor_number - 1) % 16;
  const uint8_t addr = static_cast<uint8_t>(Ir_ring_registers::ir1_offset_msb) + 2 * i;
  _handle.write_register<uint8_t, int16_t>(addr, offset);
}

void IR_ring::write_offsets(const int16_t (&offsets)[16]) const
{
  _handle.write_registers(static_cast<uint8_t>(Ir_ring_registers::ir1_offset_msb), offsets, 16);
}

uint16_t IR_ring::read_ball_visibility_threshold() const
{
  uint16_t buf;
  _handle.read_register<uint8_t, uint16_t>(static_cast<uint8_t>(Ir_ring_registers::ball_visible_thr_msb), buf);

  return buf;
}

void IR_ring::write_ball_visibility_threshold(uint16_t ball_visibility_threshold) const
{
  _handle.write_register<uint8_t, uint16_t>(static_cast<uint8_t>(Ir_ring_registers::ball_visible_thr_msb), ball_visibility_threshold);
}

uint16_t IR_ring::read_ir_sensor_mask() const
{
  uint16_t buf;
  _handle.read_register<uint8_t, uint16_t>(static_cast<uint8_t>(Ir_ring_registers::ir_sensor_mask_msb), buf);

  return buf;
}

void IR_ring::write_ir_sensor_mask(uint16_t ir_sensor_mask) const
{
  _handle.write_register<uint8_t, uint16_t>(static_cast<uint8_t>(Ir_ring_registers::ir_sensor_mask_msb), ir_sensor_mask);
}

void IR_ring::save_calibration(bool save_gains, bool save_offsets, bool save_visibility_th) const
{
  uint8_t buf[8] {};

  buf[0] = static_cast<uint8_t>(Cmd_opcode::control_eeprom);
  buf[1] = 0;
  buf[2] = save_gains | (save_offsets<<1) | (save_visibility_th<<2);

  _handle.write_registers<uint8_t, uint8_t>(static_cast<uint8_t>(Ir_ring_registers::cmd_opcode), buf, 8);
}

void IR_ring::load_calibration(bool save_gains, bool save_offsets, bool save_visibility_th) const
{
  uint8_t buf[8] {};

  buf[0] = static_cast<uint8_t>(Cmd_opcode::control_eeprom);
  buf[1] = 1;
  buf[2] = save_gains | (save_offsets<<1) | (save_visibility_th<<2);

  _handle.write_registers<uint8_t, uint8_t>(static_cast<uint8_t>(Ir_ring_registers::cmd_opcode), buf, 8);
}

void IR_ring::reset_calibration(bool save_gains, bool save_offsets, bool save_visibility_th) const
{
  uint8_t buf[8] {};

  buf[0] = static_cast<uint8_t>(Cmd_opcode::control_eeprom);
  buf[1] = 2;
  buf[2] = save_gains | (save_offsets<<1) | (save_visibility_th<<2);

  _handle.write_registers<uint8_t, uint8_t>(static_cast<uint8_t>(Ir_ring_registers::cmd_opcode), buf, 8);
}

uint8_t IR_ring::read_tssp() const
{
  uint8_t tssp_input {};
  _handle.read_register<uint8_t, uint8_t>(static_cast<uint8_t>(Ir_ring_registers::tssp_input), tssp_input);
  return tssp_input;
}

void IR_ring::read_tssp(bool (&tssp)[8]) const
{
  uint8_t tssp_input = read_tssp();
  for (uint_fast8_t i=0; i<8; i++)
    tssp[i] = tssp_input & (1 << i);
}

// bit0: DIP1; bit1: DIP2; bit2: DIP3; bit3: DIP4 (BOOT0); bit4: BTN1; bit5: BTN2
uint8_t IR_ring::read_user_input() const
{
  uint8_t buf;
  _handle.read_register<uint8_t, uint8_t>(static_cast<uint8_t>(Ir_ring_registers::user_input),buf);

  return buf;
}

// bit0: IR_A enable; bit1: IR_B enable; bit2: TSSP enable
uint8_t IR_ring::read_sensor_power_supply() const
{
  uint8_t buf;
  _handle.read_register<uint8_t, uint8_t>(static_cast<uint8_t>(Ir_ring_registers::sensor_enable), buf);

  return buf;
}

void IR_ring::write_sensor_power_supply(bool enable_ir_a, bool enable_ir_b, bool enable_tssp) const
{
  uint8_t buf = enable_ir_a | (enable_ir_b<<1) | (enable_tssp<<2);
  _handle.write_register<uint8_t, uint8_t>(static_cast<uint8_t>(Ir_ring_registers::sensor_enable), buf);
}
