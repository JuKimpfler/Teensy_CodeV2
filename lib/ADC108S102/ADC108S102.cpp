#include "ADC108S102.h"

// vielleicht muss man die Return werte & 0x0FFF machen, da die oberen 4 bit nicht relevant sind. (12 bit ADC)????

/**
 * @brief Initializes the ADC108S102 by setting the chip select pin and starting SPI communication.
 */
void ADC108S102::init()
{
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
}

/**
 * @brief Reads the value from a specific channel of the ADC108S102.
 *
 * @param channel The channel number to read (0-7).
 */
uint16_t ADC108S102::read(uint8_t channel)
{
    if (channel > 7)
        return 0; // Invalid channel

    _spi->beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE3));

    uint16_t command = (channel + 1) << 11; // Command format for ADC108S102
    digitalWrite(csPin, LOW);
    uint16_t result = _spi->transfer16(comandlist[channel]);
    digitalWrite(csPin, HIGH);

    _spi->endTransaction();
    lastValue[channel] = result;
    return result;
}

/**
 * @brief Reads values from all channels of the ADC108S102 and stores them in the provided array.
 */
void ADC108S102::read(uint16_t (&values)[8])
{
    _spi->beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE3));
    digitalWriteFast(csPin, LOW);
    for (int i = 0; i < 8; i++)
    {
        uint16_t command = (i + 1) << 11; // Command format for ADC108S102
        values[i] = _spi->transfer16(comandlist[i]);
    }
    digitalWriteFast(csPin, HIGH);
    for (int i = 0; i < 8; i++)
        lastValue[i] = values[i];

    _spi->endTransaction();
}