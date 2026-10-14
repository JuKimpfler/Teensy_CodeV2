// XM125_Connectivity.cpp
// Grundlegender Verkabelungs-/Inbetriebnahmetest für den Acconeer XM125 Radarsensor.
// Prüft: I2C-Erreichbarkeit, Wake/Ready-Sequenz, Version- und Status-Register.
//
// Ausführen: Datei nach src/main.cpp des Hauptprojekts kopieren (in src/ darf
// immer nur eine Datei mit setup()/loop() liegen) und normal bauen/hochladen:
//   pio run -e teensy40 -t upload
// Siehe lib/XM125/examples/README.md fuer alle Beispiele im Ueberblick.
//
// Verkabelung (Beispiel):
//   XM125 SDA/SCL -> Teensy Wire (Pins 18/19)
//   XM125 WAKE_UP -> optional an einen GPIO (unten WAKE_PIN), sonst per Jumper auf 3.3V lassen
//   XM125 MCU_INT -> optional an einen GPIO (unten INT_PIN), sonst -1 lassen
//
// Hinweis: Ist WAKE_UP weder auf einen GPIO noch fest auf 3.3V verdrahtet
// (floatender Pin), kann die Software den Ready-Zustand nicht erzwingen.
// begin()/wakeUp() pollen in diesem Fall bis XM125_WAKEUP_TIMEOUT_MS lang
// per I2C, ob der Sensor zufaellig antwortet — das ersetzt aber keine
// solide Beschaltung (siehe Kommentar unten bei "Sensor nicht erreichbar").

#include <Arduino.h>
#include <Wire.h>
#include "XM125.h"

// Auf -1 lassen, wenn die Pins nicht mit dem MCU verbunden sind.
static constexpr int8_t WAKE_PIN = -1;
static constexpr int8_t INT_PIN = -1;

XM125Base radar(XM125_I2C_ADDR_DEFAULT, WAKE_PIN, INT_PIN);

static constexpr uint8_t CONNECT_ATTEMPTS = 5;
static constexpr uint32_t CONNECT_RETRY_MS = 500;

bool tryConnect()
{
    for (uint8_t attempt = 1; attempt <= CONNECT_ATTEMPTS; attempt++)
    {
        Serial.print("Verbindungsversuch ");
        Serial.print(attempt);
        Serial.print("/");
        Serial.print(CONNECT_ATTEMPTS);
        Serial.print("... ");
        if (radar.begin(Wire))
        {
            Serial.println("OK");
            return true;
        }
        Serial.println("fehlgeschlagen");
        delay(CONNECT_RETRY_MS);
    }
    return false;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000)
    {
    }

    Wire.begin();
    Wire.setClock(100000); // niedriger Takt, mehr Toleranz fuer Clock-Stretching des XM125

    Serial.println("XM125 Connectivity Test");
    Serial.println("========================");

    if (!tryConnect())
    {
        Serial.print("FEHLER: Sensor nach mehreren Versuchen nicht erreichbar (I2C-Adresse 0x");
        Serial.print(radar.getAddress(), HEX);
        Serial.println(").");
        Serial.println("Moegliche Ursachen:");
        Serial.println(" - SDA/SCL/GND/3V3 falsch verkabelt oder Pullups fehlen");
        Serial.println(" - Modul steckt in einem Sleep-Zustand (z.B. nach externem WAKE_UP=Low)");
        if (WAKE_PIN < 0)
        {
            Serial.println(" - WAKE_UP-Pin ist bewusst unverkabelt/floatend (Projektentscheidung, kein Pullup)");
            Serial.println("   -> Solange nie ein Sleep-Kommando gesendet wird, bleibt das Modul davon");
            Serial.println("      unabhaengig wach; ein floatender Pin ist daher nicht die erwartete Ursache,");
            Serial.println("      bleibt aber ein Restrisiko fuer Stoereinkopplungen (siehe XM125.h).");
        }
        return;
    }

    uint32_t version = radar.getVersionRaw();
    Serial.print("Version (roh, 32-bit): 0x");
    Serial.println(version, HEX);

    uint32_t protoStatus = radar.getProtocolStatus();
    Serial.print("Protocol Status: 0x");
    Serial.println(protoStatus, HEX);
    if (protoStatus != 0)
    {
        Serial.println("-> Ungleich 0: Es gab einen Protokollfehler (falsches Registerformat/Adresse?).");
    }

    uint32_t detStatus = radar.getDetectorStatus();
    Serial.print("Detector Status: 0x");
    Serial.println(detStatus, HEX);
    Serial.print("  BUSY:  ");
    Serial.println(XM125Base::statusBusy(detStatus) ? "ja" : "nein");
    Serial.print("  ERROR: ");
    Serial.println(XM125Base::statusError(detStatus) ? "ja" : "nein");

    Serial.print("Measure Counter: ");
    Serial.println(radar.getMeasureCounter());
}

void loop()
{
    // Periodisch neu pruefen, damit man am Monitor sieht, ob die Verbindung
    // (z.B. wegen floatendem WAKE_UP) zwischenzeitlich abreisst/wiederkommt.
    delay(2000);
    bool ok = radar.isConnected();
    Serial.print("Verbindung: ");
    Serial.println(ok ? "ok" : "verloren -> versuche erneut zu verbinden");
    if (!ok)
        tryConnect();
}
