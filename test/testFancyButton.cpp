#include <Arduino.h>

#include <fancyBTN.h>
#include <TCA9534DW.h>

TCA9534DW expander(&Wire1, 0x20);

fancyBTN button(250, 3, 0, 5, true); // Sensitivity: 250 (500ms), Long press time: 3 (1.5 seconds), Status code lifetime: 100ms, Debounce time: 5ms

void setup() {
  Serial.begin(115200);

  Wire1.begin();
  Wire1.setClock(1000000); // Set I2C clock to 400kHz

  expander.setup(false); // Set all pins as input
}

void loop() {
  // Read the state of pin 0 on the expander
  expander.read();
  bool buttonState = expander.getPinState(4);

  /*Serial.print("Buttons Pressed: ");
  for (uint8_t i = 0; i < 8; i++) {
    if (expander.getPinState(i)) {
      Serial.print(i);
      Serial.print(" ");
    }
  }
  Serial.println();*/

  // Update the button state
  button.update(buttonState);

  // Check the status code and print it
  uint8_t status = button.getStatusCode();
  if (status == 4) {
    Serial.println("Button clicked");
  } else if (status == 5) {
    Serial.println("Button double clicked");
  } else if (status == 6) {
    Serial.println("Button long pressed");
  }

  // delay(5); // Small delay to avoid flooding the serial output
}