#include <Arduino.h>
#include "comProtokoll.h"
#include "systemlib.h"

systemlib sys;

void setup() {
    Serial.begin(115200);
    sys.initialize();
}

void loop() {
    sys.readSensors();

    if(Timer.debug > 100){
        //Serial.print("Radar: ");
        //Serial.println(sys.radar_test);
        Timer.debug
    }
}