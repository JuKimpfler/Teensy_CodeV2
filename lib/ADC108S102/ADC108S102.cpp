#include "ADC108S102.h"

// vielleicht muss man die Return werte & 0x0FFF machen, da die oberen 4 bit nicht relevant sind. (12 bit ADC)????

/**
 * @brief Initializes the ADC108S102 by setting the chip select pin and starting SPI communication.
 */
void ADC108S102::init()
{
    if(!useBinExp){
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);
    }
    else{
        pinMode(csPinBin1, OUTPUT);
        pinMode(csPinBin2, OUTPUT);
        pinMode(csPinBin3, OUTPUT);
        digitalWrite(csPinBin1, getAddressForLowPin(7).A0);
        digitalWrite(csPinBin2, getAddressForLowPin(7).A1);
        digitalWrite(csPinBin3, getAddressForLowPin(7).A2);
    }
}

DecoderAddress ADC108S102::getAddressForLowPin(uint8_t pin) {

    DecoderAddress addr;

    if (pin > 7) {
        return addr;
    }

    addr.A2 = (pin & 0b100) != 0; // Bit 2 -> A2 (MSB)
    addr.A1 = (pin & 0b010) != 0; // Bit 1 -> A1
    addr.A0 = (pin & 0b001) != 0; // Bit 0 -> A0 (LSB)

    return addr;
}

/**
 * @brief Reads the value from a specific channel of the ADC108S102.
 *
 * @param channel The channel number to read (0-7).
 */
int16_t ADC108S102::read(uint8_t channel,bool MOT_ADC,bool ADC_Mode)
{
    int result_raw=0;
    if (channel > 7)
        return 0; // Invalid channel

    _spi->beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE3));

    int16_t result = 0;

    if(!useBinExp){
        digitalWriteFast(csPin, LOW);
        result = _spi->transfer16(comandlist[channel]);
        digitalWriteFast(csPin, HIGH);
    }
    else{
        digitalWrite(csPinBin1, getAddressForLowPin(csPort).A0);
        digitalWrite(csPinBin2, getAddressForLowPin(csPort).A1);
        digitalWrite(csPinBin3, getAddressForLowPin(csPort).A2);
        result = _spi->transfer16(comandlist[channel]);
        digitalWrite(csPinBin1, getAddressForLowPin(7).A0);
        digitalWrite(csPinBin2, getAddressForLowPin(7).A1);
        digitalWrite(csPinBin3, getAddressForLowPin(7).A2);
    }

    _spi->endTransaction();

    result_raw = map(result,0,4060,0,3300);

    if(MOT_ADC && ADC_Mode){
        if(result_raw > 1700){
            result = (result_raw - 1650) * (6000 - 0) / (3300 - 1650) + 0;
        }
        else if(result_raw < 1600){
            result = ((result_raw - 1650) * (6000 - 0) / (0 - 1650) + 0)*-1;
        }
        else{
            result = 0;
        }
    }
    else if(MOT_ADC && !ADC_Mode){
        if(result_raw > 1650){
            result = map(result_raw,1650,3300,0,6000);
        }
        else if(result_raw <= 1650){
            result = map(result_raw,1650,0,0,6000);
        }
        else{
            result = 0;
        }
    }
    else{
        result = result_raw;
    }

    lastValue[channel] = result;
    return result;
}

/**
 * @brief Reads values from all channels of the ADC108S102 and stores them in the provided array.
 */
void ADC108S102::read(uint16_t (&values)[8])
{
    _spi->beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE3));
    if(!useBinExp){
        digitalWriteFast(csPin, LOW);
    }
    else{
        digitalWrite(csPinBin1, getAddressForLowPin(csPort).A0);
        digitalWrite(csPinBin2, getAddressForLowPin(csPort).A1);
        digitalWrite(csPinBin3, getAddressForLowPin(csPort).A2);
    }
    for (int i = 0; i < 8; i++)
    {
        values[i] = _spi->transfer16(comandlist[i]);
    }
    if(!useBinExp){
        digitalWriteFast(csPin, HIGH);
    }
    else{
        digitalWrite(csPinBin1, getAddressForLowPin(7).A0);
        digitalWrite(csPinBin2, getAddressForLowPin(7).A1);
        digitalWrite(csPinBin3, getAddressForLowPin(7).A2);
    }
    for (int i = 0; i < 8; i++)
        lastValue[i] = values[i];

    _spi->endTransaction();
}