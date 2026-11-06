#include <Arduino.h>
#include "comProtokoll.h"
#include "systemlib.h"

systemlib sys;
elapsedMillis Debug_tim;

void setup() {
    Serial.begin(250000);
    sys.initialize();
}

void loop() {
    sys.readSensors();

    if(Debug_tim > 100){
        Serial.print("cycle: ");
        Serial.println(sys.giveCycleTime_us());
        Debug_tim = 0;
    }
}