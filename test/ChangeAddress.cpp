#include <Wire.h>
#include "SRF08.h"

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(100000);  // Niedrige Taktrate für Adressänderung empfohlen

    SRF08Sensor sensor(SRF08_ADDR(0xE0));
    if (!sensor.begin(Wire)) {
        Serial.println("Sensor nicht gefunden!");
        return;
    }

    // Adresse auf 0xE2 ändern
    if (sensor.changeI2CAddress(0xE6)) {
        Serial.println("Erfolg! Sensor jetzt auf 0xE2.");
        Serial.println("Bitte '0xE2' auf den Sensor schreiben.");
    } else {
        Serial.println("Fehler bei der Adressänderung!");
    }
}

void loop() {}