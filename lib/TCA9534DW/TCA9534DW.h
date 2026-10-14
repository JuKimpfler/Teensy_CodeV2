#pragma once
#include <Arduino.h>
#include <Wire.h>

class TCA9534DW
{
private:
    TwoWire *_i2c;
    uint8_t _address;
    uint8_t _state = 0xFF;    ///< Zwischenspeicher der Eingaenge (read/update)
    uint8_t _outState = 0xFF; ///< zuletzt geschriebenes Ausgaberegister (write/writePin)

public:
    TCA9534DW(TwoWire *i2c, uint8_t address) : _i2c(i2c), _address(address) {};

    void begin(bool Input);

    void begin(bool pin0, bool pin1, bool pin2, bool pin3, bool pin4, bool pin5, bool pin6, bool pin7);

    void write(uint8_t value);
    void writePin(uint8_t pin, bool value);

    uint8_t read();
    void update(); ///< Eingangsport einmal lesen und merken; danach givePinState() ohne Bus
    uint8_t readPin(uint8_t pin);

    bool givePinState(uint8_t pin) const { return (_state >> pin) & 0x01; }
};
