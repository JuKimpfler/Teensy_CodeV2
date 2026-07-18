#include <Arduino.h>
#include <SPI.h>

class ADC108S102 {
    private:
        uint8_t csPin;
        uint16_t lastValue[8]; // Store the last read value for each channel
    public:
        void init();
        void read(uint16_t (&values)[8]);
        uint16_t read(uint8_t channel);
        uint16_t getLastValue(uint8_t channel) const { return lastValue[channel]; }
};