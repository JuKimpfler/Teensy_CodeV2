#include <Arduino.h>
#include <Wire.h>

class TCA9534DW
{
    private:
    TwoWire *_i2c;
    uint8_t _address;
    uint8_t _state;
public:
    TCA9534DW(TwoWire *i2c, uint8_t address) : _i2c(i2c), _address(address) {};

    void setup(bool Input);

    void setup(bool pin0, bool pin1, bool pin2, bool pin3, bool pin4, bool pin5, bool pin6, bool pin7);

    void write(uint8_t value);
    void writePin(uint8_t pin, bool value);

    uint8_t read();
    uint8_t readPin(uint8_t pin);

    bool getPinState(uint8_t pin) const { return (_state >> pin) & 0x01; }
    
};