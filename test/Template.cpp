#include <Arduino.h>
#include <PDS.h>

#define BUILD_VERSION "Template - hier Testname/Testversion eintragen"

void setup() {
    Serial.begin(115200);
    PDS.init();
    String Version = "v" + String(BUILD_VERSION) + " (Build " + String(BUILD_DATE) + " " + String(BUILD_TIME) + ")";
    PDS.setFirmwareVersion(Version.c_str());  // Set the firmware version for the
}

void loop() {
    static float x = 0;
    static float y = 0;
    static float z = 0;

    x += 0.1f;
    y += 0.2f;
    z += 0.3f;

    PDS.plot("X", x, "m/s");
    PDS.plot("Y", y, "m/s");
    PDS.plot("Z", z, "m/s");

    PDS.update();
    delay(100);
}