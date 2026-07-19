//
// Created by Rafael Günther on 24.10.25.
//

#pragma once

#include <Wire.h>

#include "bus_interface.h"

class I2C_bus : public Bus_interface
{
public:
  explicit I2C_bus(TwoWire& i2c = Wire) : _i2c(i2c) {};

  Bus_error read_bytes(const Bus_target& target, uint8_t* data, std::size_t length) const override;
  Bus_error write_bytes(const Bus_target& target, const uint8_t* data, std::size_t length) const override;

  Bus_error start_transmission(const Bus_target& target, Transfer_direction dir) const override;
  Bus_error stop_transmission(const Bus_target& target, Transfer_direction dir) const override;

private:
  TwoWire& _i2c;
};