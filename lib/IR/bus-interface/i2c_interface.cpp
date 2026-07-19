//
// Created by Rafael Günther on 24.10.25.
//

#include "i2c_interface.h"

Bus_error I2C_bus::read_bytes(const Bus_target& target, uint8_t* data, std::size_t length) const
{
  if (!data || length == 0)
    return Bus_error::invalid_arg;

  const auto i2c_target = std::get_if<I2C_target>(&target);
  if (!i2c_target)
    return Bus_error::invalid_arg;

  _i2c.requestFrom(i2c_target->addr, length, true);
  for (std::size_t i=0; i<length; i++)
    data[i] = _i2c.read();

  return Bus_error::ok;
}

Bus_error I2C_bus::write_bytes(const Bus_target& target, const uint8_t* data, std::size_t length) const
{
  if (!data || length == 0)
  return Bus_error::invalid_arg;

  const auto i2c_target = std::get_if<I2C_target>(&target);
  if (!i2c_target)
    return Bus_error::invalid_arg;

  for (std::size_t i=0; i<length; i++)
    _i2c.write(data[i]);

  return Bus_error::ok;
}

Bus_error I2C_bus::start_transmission(const Bus_target& target, Transfer_direction dir) const
{
  if (dir == Transfer_direction::read)
    return Bus_error::ok;

  const auto i2c_target = std::get_if<I2C_target>(&target);
  if (!i2c_target)
    return Bus_error::invalid_arg;

  _i2c.beginTransmission(i2c_target->addr);

  return Bus_error::ok;
}

Bus_error I2C_bus::stop_transmission(const Bus_target& target, Transfer_direction dir) const
{
  if (dir == Transfer_direction::read)
    return Bus_error::ok;

  _i2c.endTransmission(true);

  return Bus_error::ok;
}

