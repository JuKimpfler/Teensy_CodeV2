/*
 * ============================================================
 *  Template fuer einen eigenstaendigen Test-Sketch mit PDS
 * ============================================================
 *  Kopiervorlage fuer kleine Versuche OHNE systemlib: nur Teensy und die
 *  PDS-Bibliothek. Wer systemlib benutzt, bekommt die Systemkanaele
 *  (Akku, Radar, Position, ...) automatisch -- siehe test/main.cpp.
 *
 *  Zum Flashen in src/ kopieren; test/ wird von "pio run" nicht gebaut.
 * ============================================================
 */

#include <Arduino.h>
#include <PDS.h>

// platformio.ini setzt BUILD_VERSION bereits ueber die build_flags. Fuer
// einen einzelnen Test ist ein sprechender Name nuetzlicher als "0.0.1" --
// ohne #undef gaebe es hier eine Redefinition-Warnung.
#undef BUILD_VERSION
#define BUILD_VERSION "Template - hier Testname/Testversion eintragen"

float x = 0;
float y = 0;
float z = 0;

elapsedMillis statusTimer;

void setup() {
    Serial.begin(115200);

    PDS.begin();   // startet UART_DBG und meldet die Kanalnamen an die GUI

    // Erscheint in der Fusszeile der GUI: welcher Stand laeuft hier?
    String Version = "v" + String(BUILD_VERSION) +
                     " (Build " + String(BUILD_DATE) + " " + String(BUILD_TIME) + ")";
    PDS.setFirmwareVersion(Version.c_str());

    // track() bindet die Variable einmalig; die Werte holt die Bibliothek
    // danach 100x/s selbst ab. Im loop() ist dafuer nichts mehr zu tun.
    PDS.track("X", &x, "m/s");
    PDS.track("Y", &y, "m/s");
    PDS.track("Z", &z, "m/s");

    PDS.log("Template gestartet");
}

void loop() {
    x += 0.1f;
    y += 0.2f;
    z += 0.3f;

    // Werte, die man nicht binden will, gehen direkt raus -- die Kanalnummer
    // vergibt PDS selbst:
    PDS.plot("Summe", x + y + z);

    // Eine Zeile Klartext-Diagnose ueber USB-Serial (Zykluszeit, Paketzahl,
    // Dauer von update(), Zustand der Verbindung).
    if (statusTimer >= 1000) {
        statusTimer = 0;
        PDS.printStatus();
    }

    PDS.update();   // genau einmal pro loop(); blockiert nie
    delay(10);
}
