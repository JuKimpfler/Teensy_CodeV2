#include <Arduino.h>
#include "comProtokoll.h"
#include "systemlib.h"

systemlib sys;
elapsedMillis Debug_tim;

void setup() {
    Serial.begin(115200);
    sys.initialize();
}

void loop() {
    sys.readSensors();

    if(Debug_tim > 100){
        Serial.print("Radar: ");
        Serial.println(sys.radar_test);
        Debug_tim = 0;
    }
}