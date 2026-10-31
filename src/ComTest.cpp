#include <Arduino.h>
#include "commlib.h"

void setup() {
    Serial.begin(115200);

    Serial6.begin(115200);
}

void loop() {
    // if(Serial6.available()) {
    //     char c = Serial6.read();
    //     Serial.print(c);
    // }
    // if(Serial.available()) {
    //     char c = Serial.read();
    //     Serial6.print(c);
    // }

    Serial6.print("t");
    Serial.print(Serial6.read());
    delay(100);

}