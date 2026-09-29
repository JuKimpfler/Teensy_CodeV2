#include <Arduino.h>
#include <SPI.h>

struct DecoderAddress {
    bool A2; // true = H, false = L
    bool A1;
    bool A0;
};

class ADC108S102 {
    private:
        SPIClass *_spi;
        uint8_t csPin;
        uint8_t csPort;
        uint8_t csPinBin1;
        uint8_t csPinBin2;
        uint8_t csPinBin3;
        bool useBinExp = false;
        int16_t lastValue[8]; // Store the last read value for each channel
        uint16_t comandlist[8] = {2048,4096,6144,8192,10240,12288,14336,16384};
        DecoderAddress getAddressForLowPin(uint8_t pin);
    public:
        ADC108S102(SPIClass *spi_port, uint8_t chipSelectPin, bool useBinExpander = false) : _spi(spi_port), csPin(chipSelectPin), useBinExp(useBinExpander) {}
        ADC108S102(SPIClass *spi_port, uint8_t chipSelectPinBin1, uint8_t chipSelectPinBin2, uint8_t chipSelectPinBin3,uint8_t chipSelectPort, bool useBinExpander = true) : _spi(spi_port), csPort(chipSelectPort), csPinBin1(chipSelectPinBin1), csPinBin2(chipSelectPinBin2), csPinBin3(chipSelectPinBin3), useBinExp(useBinExpander) {}
        void init();
        void read(uint16_t (&values)[8]);
        int16_t read(uint8_t channel,bool MOT_ADC = false,bool ADC_Mode = false); // Mot_ADC -> Enable if Motor ADC is used / ADC_Mode -> I = false - RPM = true
        int16_t getLastValue(uint8_t channel) const { return lastValue[channel]; }
};