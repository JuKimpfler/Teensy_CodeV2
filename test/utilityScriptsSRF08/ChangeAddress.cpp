#include <Wire.h>
#include "SRF08.h"

void setup() {
    delay(10000);
    Serial.begin(115200);
    Wire1.begin();
    Wire1.setClock(100000);  // Niedrige Taktrate für Adressänderung empfohlen

    SRF08Sensor sensor(SRF08_ADDR(0xE6));
    if (!sensor.begin(Wire1)) {
        Serial.println("Sensor nicht gefunden!");
        return;
    }

    // Adresse auf 0xE2 ändern
    if (sensor.changeI2CAddress(0xE0)) {
        Serial.println("Erfolg! Sensor jetzt auf 0xE0.");
        Serial.println("Bitte '0xE0' auf den Sensor schreiben.");
    } else {
        Serial.println("Fehler bei der Adressänderung!");
    }
}

void loop() {}