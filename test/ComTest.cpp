#include <Arduino.h>
#include "comProtokoll.h"
#include "systemlib.h"

systemlib sys;
elapsedMillis debug;

void setup() {
    Serial.begin(115200);
    sys.initialize();
}

void loop() {
    sys.readSensors();

    if(debug > 100){
        Serial.print("IR: ");
        Serial.println(sys.giveBallAngle());
        Serial.print("IR: ");
        Serial.println(sys.giveBallDistance());
        Serial.print("BNO: ");
        Serial.println(sys.giveOrbitDir());
        Serial.print("Start: ");
        Serial.println(sys.giveStart());
        Serial.print("Cam2: ");
        Serial.println(sys.giveCam2Angle());
        debug  = 0;
    }
}