#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <ADC108S102.h>
#include <TCA9534DW.h>

#define MySPI SPI

constexpr uint8_t EN_VDRIB = 0;
constexpr uint8_t EN_VMOT = 1;
constexpr uint8_t EN_BARRIER = 2;
constexpr uint8_t EN_LINE = 3;
constexpr uint8_t EN_UART_DIO = 4;
constexpr uint8_t EN_INTERCON = 5;

/// Ruhezustand von IC8: nur Sensor- und LED-Versorgung an, Leistung aus.
/// Ohne LINE_EN sind die Linien-LEDs dunkel, ohne INTERCON fehlt den externen
/// Modulen am 3V3-Intercon die Versorgung — beides wuerde als Defekt aussehen.
constexpr uint8_t EN_SAFE = (1 << EN_BARRIER) | (1 << EN_VDRIB) | (1 << EN_VMOT) | (1 << EN_LINE); // ALles aktivieren

TCA9534DW expEN(&Wire1, 0x23); ///< IC8


ADC108S102 ADC_Line1(&MySPI, 31, 32, 33, 0);
ADC108S102 ADC_Line2(&MySPI, 31, 32, 33, 1);
ADC108S102 ADC_Line3(&MySPI, 31, 32, 33, 2);
ADC108S102 ADC_Line4(&MySPI, 31, 32, 33, 3);
ADC108S102 ADC_Line5(&MySPI, 31, 32, 33, 4);
ADC108S102 ADC_Line6(&MySPI, 31, 32, 33, 5);

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        delay(10);

    

    Wire1.begin();
    Wire1.setClock(100000);
    expEN.write(EN_SAFE);
    expEN.setup(false);
    

    MySPI.begin();
    // SPI.setClockDivider(SPI_CLOCK_DIV16); // 24 MHz / 16 = 1,5 MHz
    // SPI.setDataMode(SPI_MODE0);
    // SPI.setBitOrder(MSBFIRST);

    ADC_Line1.init();
    ADC_Line2.init();
    ADC_Line3.init();
    ADC_Line4.init();
    ADC_Line5.init();
    ADC_Line6.init();

    Serial.println("ADC108S102 Test");
    Serial.println("Kanal 0..7: Spannung in mV");
}

void loop()
{
    Serial.print("ADC1: ");
    for (int i = 0; i < 8; i++)
    {
        Serial.print(ADC_Line1.read(i));
        Serial.print(" ");
    }
    Serial.println("");
    Serial.print(" | ADC2: ");
    for (int i = 0; i < 8; i++)
    {
        Serial.print(ADC_Line2.read(i));
        Serial.print(" ");
    }
    Serial.println("");
    Serial.print(" | ADC3: ");
    for (int i = 0; i < 8; i++)
    {
        Serial.print(ADC_Line3.read(i));
        Serial.print(" ");
    }
    Serial.println("");
    Serial.print(" | ADC4: ");
    for (int i = 0; i < 8; i++)
    {
        Serial.print(ADC_Line4.read(i));
        Serial.print(" ");
    }
    Serial.println("");
    Serial.print(" | ADC5: ");
    for (int i = 0; i < 8; i++)
    {
        Serial.print(ADC_Line5.read(i));
        Serial.print(" ");
    }
    Serial.println("");
    Serial.print(" | ADC6: ");
    for (int i = 0; i < 8; i++)
    {
        Serial.print(ADC_Line6.read(i));
        Serial.print(" ");
    }
    Serial.println();

    delay(600);
}
