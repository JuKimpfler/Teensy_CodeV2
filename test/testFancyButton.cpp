#include <Arduino.h>

#include <fancyBTN.h>
#include <TCA9534DW.h>

TCA9534DW expander(&Wire1, 0x20);

fancyBTN button(250, 3, 5, true); // Sensitivity: 250 (500ms), Long press time: 3 (1.5 seconds), Status code lifetime: 100ms, Debounce time: 5ms

void setup()
{
    Serial.begin(115200);

    Wire1.begin();
    Wire1.setClock(1000000); // Set I2C clock to 400kHz

    expander.begin(false); // Set all pins as input
}

void loop()
{
    // Read the state of pin 0 on the expander
    expander.read();
    bool buttonState = expander.givePinState(4); // SWI1

    // Update the button state
    button.update(buttonState);

    // Check the status code and print it
    uint8_t status = button.giveStatusCode();
    if (status == fancy_CLICKED)
    {
        Serial.println("Button clicked");
    }
    else if (status == fancy_DOUBLE_CLICK)
    {
        Serial.println("Button double clicked");
    }
    else if (status == fancy_LONG_PRESS)
    {
        Serial.println("Button long pressed");
    }

    // delay(5); // Small delay to avoid flooding the serial output
}
