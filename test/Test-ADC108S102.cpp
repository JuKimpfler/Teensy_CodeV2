#include <Arduino.h>

#include <ADC108S102.h>

#define CS_PIN 1

ADC108S102 adc(CS_PIN);

void setup() {
    Serial.begin(115200);

    SPI.begin(); // optional, as ADC108S102::init() also calls SPI.begin()
    adc.init();
}

void loop() {
    uint16_t values[8];
    adc.read(values);

    Serial.print("ADC Values: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(values[i]);
        Serial.print(", ");
    }

    Serial.print(adc.getLastValue(6)); // Read the 6th channel from buffer
    Serial.print(", ");

    Serial.print(adc.read(7)); // Read the last channel directly
    Serial.println();

    delay(500); // Read every half second
}
