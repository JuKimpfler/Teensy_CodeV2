#pragma once
#include <Arduino.h>
struct TwoWire
{
    bool begin(int sda = -1, int scl = -1) { (void)sda; (void)scl; return true; }
    void begin(uint8_t) {}
    void setClock(uint32_t) {}
    void setTimeOut(uint16_t) {}
    void beginTransmission(uint8_t) {}
    uint8_t endTransmission(bool = true) { return 0; }
    size_t write(uint8_t) { return 1; }
    size_t write(const uint8_t *, size_t n) { return n; }
    uint8_t requestFrom(int, int) { return 0; }
    uint8_t requestFrom(uint8_t, uint8_t) { return 0; }
    uint8_t requestFrom(uint8_t, size_t, bool) { return 0; }
    int read() { return 0; }
    size_t readBytes(uint8_t *, size_t) { return 0; }
    int available() { return 0; }
};
extern TwoWire Wire;
