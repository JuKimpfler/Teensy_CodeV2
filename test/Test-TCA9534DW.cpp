#include <Arduino.h>

#include <TCA9534DW.h>

TCA9534DW portExpander(&Wire1, 0x20); // Create an instance of the TCA9534DW class with the I2C address 0x20

void setup()
{
    Serial.begin(115200);
    Wire1.begin();                                                          // Initialize I2C communication important for the port expander
    portExpander.begin(true, true, true, true, false, false, false, false); // Initialize the port expander with the first four pins as outputs and the last four pins as inputs
}

void loop()
{
    // Example usage of the port expander
    portExpander.writePin(0, true); // Set pin 0 high
    delay(1000);
    portExpander.writePin(0, false); // Set pin 0 low
    delay(1000);

    portExpander.write(0b00001111); // Set the first four pins high and the last four pins low
    delay(1000);

    uint8_t pinState = portExpander.readPin(4); // Read the state of pin 4
    Serial.print("Pin 4 state: ");
    Serial.println(pinState);
}
