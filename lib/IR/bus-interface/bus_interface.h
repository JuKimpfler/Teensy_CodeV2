//
// Created by Rafael Günther on 23.10.25.
//

#pragma once

#include <cstdint>
#include <variant>

struct I2C_target;
struct SPI_target;
struct UART_target;
using Bus_target = std::variant<I2C_target, SPI_target, UART_target>;

class Device_handle;

enum class Bus_error : uint8_t
{
  ok=0,
  invalid_arg,
  unknown_error
};

enum class Transfer_direction : uint8_t
{
  read=0, write
};

class Bus_interface
{
public:
  virtual ~Bus_interface() = default;

  virtual Bus_error read_bytes(const Bus_target& target, uint8_t* data, std::size_t len) const = 0;
  virtual Bus_error write_bytes(const Bus_target& target, const uint8_t* data, std::size_t len) const = 0;

  virtual Bus_error start_transmission(const Bus_target& target, Transfer_direction dir) const = 0;
  virtual Bus_error stop_transmission(const Bus_target& target, Transfer_direction dir) const = 0;

  Bus_error read_words(const Bus_target& target, uint16_t* data, std::size_t len) const;
  Bus_error write_words(const Bus_target& target, const uint16_t* data, std::size_t len) const;

  Bus_error read_byte(const Bus_target& target, uint8_t& in) const { return read_bytes(target, &in, 1); }
  Bus_error write_byte(const Bus_target& target, uint8_t out) const { return write_bytes(target, &out, 1); }

  Bus_error read_word(const Bus_target& target, uint16_t& in) const { return read_words(target, &in, 1); }
  Bus_error write_word(const Bus_target& target, uint16_t out) const { return write_words(target, &out, 1); }

  template<typename RegAddrT, typename RegValT>
  Bus_error read_registers(const Bus_target& target, RegAddrT reg_addr, RegValT* data, std::size_t len) const;
  template<typename RegAddrT, typename RegValT>
  Bus_error write_registers(const Bus_target& target, RegAddrT reg_addr, const RegValT* data, std::size_t len) const;

  template<typename RegAddrT, typename RegValT>
  Bus_error read_register(const Bus_target& target, RegAddrT reg_addr, RegValT& data) const
  {
    return read_registers(target, reg_addr, &data, 1);
  }
  template<typename RegAddrT, typename RegValT>
  Bus_error write_register(const Bus_target& target, RegAddrT reg_addr, RegValT& data) const
  {
    return write_registers(target, reg_addr, &data, 1);
  }
};

struct I2C_target
{
  uint8_t addr;
};

struct SPI_target
{
  uint8_t cs_pin;
  uint32_t clock_speed_hz;
  uint8_t mode;
};

struct UART_target
{
  uint32_t baudrate;
};

class Device_handle
{
public:
  Device_handle(Bus_interface& bus, Bus_target target) : _bus(bus), _target(target) {};

  Bus_error read_bytes(uint8_t* data, std::size_t length) const { return _bus.read_bytes(_target, data, length); }
  Bus_error write_bytes(const uint8_t* data, std::size_t length) const { return _bus.write_bytes(_target, data, length); }

  Bus_error read_words(uint16_t* data, std::size_t length) const { return _bus.read_words(_target, data, length); }
  Bus_error write_words(const uint16_t* data, std::size_t length) const { return _bus.write_words(_target, data, length); }

  Bus_error read_byte(uint8_t& in) const { return _bus.read_byte(_target, in); }
  Bus_error write_byte(uint8_t out) const { return _bus.write_byte(_target, out); }

  Bus_error read_word(uint16_t& in) const { return _bus.read_word(_target, in); }
  Bus_error write_word(uint16_t out) const { return _bus.write_word(_target, out); }

  Bus_error start_transmission(Transfer_direction dir) const { return _bus.start_transmission(_target, dir); }
  Bus_error stop_transmission(Transfer_direction dir) const { return _bus.stop_transmission(_target, dir); }

  template<typename RegAddrT, typename RegValT>
  Bus_error read_registers(RegAddrT reg_addr, RegValT* data, std::size_t len) const
  {
    return _bus.read_registers(_target, reg_addr, data, len);
  }

  template<typename RegAddrT, typename RegValT>
  Bus_error write_registers(RegAddrT reg_addr, const RegValT* data, std::size_t len) const
  {
    return _bus.write_registers(_target, reg_addr, data, len);
  }

  template<typename RegAddrT, typename RegValT>
  Bus_error read_register(RegAddrT reg_addr, RegValT& data) const
  {
    return _bus.read_register(_target, reg_addr, data);
  }

  template<typename RegAddrT, typename RegValT>
  Bus_error write_register(RegAddrT reg_addr, const RegValT& data) const
  {
    return _bus.write_register(_target, reg_addr, data);
  }

private:
  Bus_interface& _bus;
  Bus_target _target;
};

template <typename RegAddrT, typename RegValT>
Bus_error Bus_interface::read_registers(const Bus_target& target, RegAddrT reg_addr, RegValT* data, std::size_t len) const
{
  // Only 1 byte register address and 2 byte register size currently supported
  if (sizeof(RegAddrT) > 1 || sizeof(RegValT) > 2)
    return Bus_error::invalid_arg;

  start_transmission(target, Transfer_direction::write);
  write_byte(target, static_cast<uint8_t>(reg_addr));
  stop_transmission(target, Transfer_direction::write);

  start_transmission(target, Transfer_direction::read);
  if (sizeof(RegValT) == 1)
    read_bytes(target, reinterpret_cast<uint8_t*>(data), len);
  else if (sizeof(RegValT) == 2)
    read_words(target, reinterpret_cast<uint16_t*>(data), len);
  stop_transmission(target, Transfer_direction::read);

  return Bus_error::ok; // TODO: Implement error handling
}

template <typename RegAddrT, typename RegValT>
Bus_error Bus_interface::write_registers(const Bus_target& target, RegAddrT reg_addr, const RegValT* data, std::size_t len) const
{
  // Only 1 byte register address and 2 byte register size currently supported
  if (sizeof(RegAddrT) > 1 || sizeof(RegValT) > 2)
    return Bus_error::invalid_arg;

  start_transmission(target, Transfer_direction::write);
  write_byte(target, static_cast<uint8_t>(reg_addr));
  if (sizeof(RegValT) == 1)
    write_bytes(target, reinterpret_cast<const uint8_t*>(data), len);
  else if (sizeof(RegValT) == 2)
    write_words(target, reinterpret_cast<const uint16_t*>(data), len);
  stop_transmission(target, Transfer_direction::write);

  return Bus_error::ok; // TODO: Implement error handling
}

