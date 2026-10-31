#include <Arduino.h>
#include "comProtokoll.h"

DataSync datasync;
elapsedMillis toogle_timer;
elapsedMillis toogle_timer2;

void setup() {
    Serial.begin(115200);
    datasync.begin();

    pinMode(9,OUTPUT);
}

bool toogle;

void loop() {
    datasync.update();

    if( toogle_timer > 2000){
        if(toogle==true){
            toogle = false;
        }
        else{
            toogle = true;
        }
        toogle_timer = 0;
        Serial.println("toogle");
    }
    
    if( toogle_timer2 > 2000){
      Serial.print("bool:");
      Serial.println(datasync.rc_bool);
      Serial.print("Connected:");
      Serial.println(datasync.isConnected());
      toogle_timer2 = 0;
    }

    datasync.test_bool = toogle;

    if(datasync.isConnected()){
        if(datasync.rc_bool){
            digitalWrite(9,HIGH);
        }
        else{
            digitalWrite(9,LOW);
        }
    }
    else{
        digitalWrite(9,LOW);
    }
}