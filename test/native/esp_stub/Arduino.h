#pragma once
// Grobes Abbild der Teile des Arduino-ESP32-Cores, die esp/src/main.cpp benutzt. NUR fuer einen Syntaxlauf auf dem
// Rechner (test/native/check_esp_syntax.sh): prueft Typen und Aufrufe von main.cpp und den Libs gegen diese Annahmen,
// nicht gegen den echten Core.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#define ARDUINO 10819
#define DEG_TO_RAD 0.017453292519943295
#define SERIAL_8N1 0x800001c
#define INPUT 0x01
using std::abs;
using std::min;
static const uint8_t D0 = 1, D1 = 2, D2 = 3, D3 = 4, D4 = 5, D5 = 6, D6 = 43, D7 = 44, D8 = 7, D9 = 8, D10 = 9;
inline void delay(uint32_t) {}
inline void yield() {}
inline void delayMicroseconds(unsigned int) {}
inline uint32_t millis() { return 0; }
inline uint32_t micros() { return 0; }
inline void pinMode(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return 0; }
inline void digitalWrite(uint8_t, uint8_t) {}
#define HIGH 1
#define LOW 0
#define OUTPUT 0x03
#define constrain(a, l, h) ((a) < (l) ? (l) : ((a) > (h) ? (h) : (a)))
// LEDC: arduino-esp32 2.x (Kanalnummer) und 3.x (Pin) -- beide Faelle deklariert, main.cpp waehlt per ESP_ARDUINO_VERSION_MAJOR
bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution);
bool ledcDetach(uint8_t pin);
bool ledcWrite(uint8_t pinOrChannel, uint32_t duty);
double ledcSetup(uint8_t channel, double freq, uint8_t resolutionBits);
void ledcAttachPin(uint8_t pin, uint8_t channel);
void ledcDetachPin(uint8_t pin);
inline void tone(uint8_t, unsigned int, unsigned long = 0) {}
struct Print
{
    virtual ~Print() {}
    virtual size_t write(uint8_t) = 0;
    virtual size_t write(const uint8_t *b, size_t n)
    {
        size_t k = 0;
        while (n--)
            k += write(*b++);
        return k;
    }
    virtual int availableForWrite() { return 64; }
};
struct Stream : Print
{
    virtual int read() = 0;
    virtual int available() = 0;
};
struct HardwareSerial : Stream
{
    size_t write(uint8_t) override { return 1; }
    int read() override { return -1; }
    int available() override { return 0; }
    void begin(unsigned long) {}
    void begin(unsigned long, uint32_t, int8_t, int8_t) {}
    size_t setRxBufferSize(size_t n) { return n; }
};
extern HardwareSerial Serial, Serial1;
struct EspClass
{
    uint32_t getFreeHeap() { return 0; }
};
extern EspClass ESP;
void setup();
void loop();
