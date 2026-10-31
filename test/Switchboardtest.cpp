#include <Arduino.h>
#include "TCA9534DW.h"

#define MyWire Wire1

TCA9534DW TCA_SWI(&MyWire, 0x26);
TCA9534DW TCA_BTN(&MyWire, 0x27);

void setup()
{
    Serial.begin(115200);

    MyWire.begin();

    TCA_SWI.setup(true);
    TCA_BTN.setup(true);
}

void loop()
{
    // Read the state of the buttons
    uint8_t buttonState = TCA_BTN.read();
    uint8_t switchState = TCA_SWI.read();

    Serial.print("Button State: ");
    for (uint8_t i = 0; i < 5; i++)
    {
        bool isPressed = (buttonState >> i) & 0x01;
        Serial.print(i);
        Serial.print(": ");
        Serial.print(isPressed ? "P |" : "R |");
    }
    Serial.println();

    Serial.print("Switch State: ");
    for (uint8_t i = 0; i < 5; i++)
    {
        bool isOn = (switchState >> i) & 0x01;
        Serial.print(i);
        Serial.print(": ");
        Serial.print(isOn ? "ON |" : "OFF |");
    }
    Serial.println();

    Serial.print("RCJ-Module: ");
    Serial.print((TCA_SWI.readPin(7)) ? "DA  |" : "WEG |");
    Serial.print((TCA_SWI.readPin(6)) ? "EN  |" : "DIS |");
    Serial.print((TCA_BTN.readPin(5)) ? "AN  |" : "AUS |");
    Serial.print((TCA_BTN.readPin(6)) ? "AN  |" : "AUS |");
    Serial.println();

    Serial.print("PX-Pin: ");
    Serial.print((TCA_BTN.readPin(7)) ? "AN" : "AUS");
    Serial.println();

    delay(1000);
}