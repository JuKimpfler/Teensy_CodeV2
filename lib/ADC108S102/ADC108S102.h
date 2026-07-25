#include <Arduino.h>
#include <SPI.h>

class ADC108S102 {
    private:
        SPIClass *_spi;
        uint8_t csPin;
        uint16_t lastValue[8]; // Store the last read value for each channel
        int comandlist[8] = {2048,4096,6144,8192,10240,12288,14336,16384};
    public:
        ADC108S102(SPIClass *spi_port, uint8_t chipSelectPin) : _spi(spi_port), csPin(chipSelectPin) {}
        void init();
        void read(uint16_t (&values)[8]);
        uint16_t read(uint8_t channel,bool MOT_ADC = false,bool ADC_Mode = false); // Mot_ADC -> Enable if Motor ADC is used / ADC_Mode -> I = false - RPM = true
        uint16_t getLastValue(uint8_t channel) const { return lastValue[channel]; }
};