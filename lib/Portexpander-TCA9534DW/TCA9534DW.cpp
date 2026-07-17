#include <TCA9534DW.h>

/**
 * @brief Initializes the TCA9534DW port expander by setting all pins as inputs.
 * @note This function needs the Wire to be already set up and initialized.
 */
void TCA9534DW::setup()
{
    _i2c->beginTransmission(_address);
    _i2c->write(0x03); // Configuration register
    _i2c->write(0xFF); // Set all pins as inputs
    _i2c->endTransmission();
}

/**
 * @brief Initializes the TCA9534DW port expander by setting the direction of each pin.
 * @note This function needs the Wire to be already set up and initialized.
 */
void TCA9534DW::setup(bool pin0, bool pin1, bool pin2, bool pin3, bool pin4, bool pin5, bool pin6, bool pin7)
{

    _i2c->beginTransmission(_address);
    _i2c->write(0x03); // Configuration register
    uint8_t config = (pin0) | (pin1 << 1) | (pin2 << 2) | (pin3 << 3) |
                     (pin4 << 4) | (pin5 << 5) | (pin6 << 6) | (pin7 << 7);
    _i2c->write(config);
    _i2c->endTransmission();
}

/**
 * @brief Writes a value(byte) to the output ports of the TCA9534DW.
 */
void TCA9534DW::write(uint8_t value)
{
    _i2c->beginTransmission(_address);
    _i2c->write(0x01); // Output port register
    _i2c->write(value);
    _i2c->endTransmission();
    _state = value;
}

/**
 * @brief Writes a value to a specific pin of the TCA9534DW.
 */
void TCA9534DW::writePin(uint8_t pin, bool value)
{
    if (value)
        _state |= (1 << pin);
    else
        _state &= ~(1 << pin);

    _i2c->beginTransmission(_address);
    _i2c->write(0x01); // Output port register
    _i2c->write(_state);
    _i2c->endTransmission();
}

/**
 * @brief Reads the current state of the input ports of the TCA9534DW.
 * @return The current state of the input ports as a byte, or -1 if no data is available.
 */
uint8_t TCA9534DW::read()
{
    _i2c->beginTransmission(_address);
    _i2c->write(0x00); // Input port register
    _i2c->endTransmission();
    _i2c->requestFrom(_address, (uint8_t)1);

    if (_i2c->available())
        return _i2c->read();

    return -1; // Return -1 if no data is available
}

/**
 * @brief Reads the current state of a specific pin of the TCA9534DW.
 * @return The state of the specified pin (0 or 1), or -1 if no data is available.
 */
uint8_t TCA9534DW::readPin(uint8_t pin)
{
    uint8_t inputState = read();
    if (inputState != (uint8_t)-1)
        return (inputState >> pin) & 0x01;

    return -1; // Return -1 if no data is available
}
