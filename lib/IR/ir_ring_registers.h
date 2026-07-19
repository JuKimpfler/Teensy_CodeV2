//
// Created by Rafael Günther on 23.10.25.
//

#pragma once

enum class Ir_ring_registers : uint8_t
{
  sensor_id             = 0x00,
  fw_version_major      = 0x01,
  fw_version_minor      = 0x02,
  fw_version_patch      = 0x03,
  hardware_rev          = 0x04,
  harware_id            = 0x05,

  error_msb             = 0x0A,
  error_lsb             = 0x0B,
  user_input            = 0x0C,
  tssp_input            = 0x0D,

  ball_visible          = 0x10,
  ball_angle_msb        = 0x11,
  ball_angle_lsb        = 0x12,
  ball_distance_msb     = 0x13,
  ball_distance_lsb     = 0x14,

  sensor_enable         = 0x19,
  led_enable            = 0x1D,
  ball_visible_thr_msb  = 0x1E,
  ball_visible_thr_lsb  = 0x1F,
  ir_sensor_mask_msb    = 0x20,
  ir_sensor_mask_lsb    = 0x21,

  ir1_data_raw_msb      = 0x28,

  ir1_data_calib_msb    = 0x48,

  ir1_offset_msb        = 0x68,
  ir1_offset_lsb        = 0x69,

  ir1_gain_msb          = 0x88,
  ir1_gain_lsb          = 0x89,

  cmd_opcode            = 0xA8,
  cmd_arg1              = 0xA9,
  cmd_arg2              = 0xAA,
  cmd_arg3              = 0xAB,
  cmd_arg4              = 0xAC,
  cmd_arg5              = 0xAD,
  cmd_arg6              = 0xAE,
  cmd_arg7              = 0xAF,

};