#include <Arduino.h>
#include "comProtokoll.h"
#include "systemlib.h"

systemlib sys;
elapsedMillis debug;
elapsedMicros debug2;

void setup() {
    Serial.begin(250000);
    sys.initialize();
}

void loop() {
    
    sys.readSensors();
    sys.Position.Y = random(240);
    sys.Position.X = random(180);
    sys.Position.Certainty = 1;
    debug2 = 0;
    sys.moveToPointAvoid(random(180),random(180),SPattern_Cosine,100,0,0);

    if(debug > 100){
        Serial.print("Cycle: ");
        Serial.println(debug2);
        debug  = 0;
    }
}