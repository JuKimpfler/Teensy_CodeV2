#include <Arduino.h>
#include <Wire.h>
#include "XM125Distance.h"

#define PDS_ENABLED 1 // only if PDS is used, otherwise comment out
#ifdef PDS_ENABLED
#include "PDS.h"
PowerDebugger PDS;
#endif

#define RADARWIRE Wire
#define RADAR_NUM_SENSORS 4

XM125Distance radarVo = XM125Distance(0x51, -1, -1);
XM125Distance radarRe = XM125Distance(0x52, -1, -1);
XM125Distance radarHi = XM125Distance(0x53, -1, -1);
XM125Distance radarLi = XM125Distance(0x54, -1, -1);

XM125Distance *radars[RADAR_NUM_SENSORS] = {&radarVo, &radarRe, &radarHi, &radarLi};

bool calibrateSensor(XM125Distance &radar)
{
    radar.setMeasureTimeout(3000); // Profile5/CFAR braucht laenger als der Default

    // Gleiche Konfiguration wie im Einzelsensor-Test (XM125_Distance.cpp)
    radar.config.startMm = 200;
    radar.config.endMm = 2500;
    radar.config.maxProfile = XM125D_PROFILE5;
    radar.config.thresholdMethod = XM125D_THRESHOLD_CFAR;
    radar.config.peakSorting = XM125D_SORT_CLOSEST;
    radar.config.reflectorShape = XM125D_REFLECTOR_PLANAR;

    if (!radar.applyConfigAndCalibrate())
    {
        Serial.print("Sensor 0x");
        Serial.print(radar.getAddress(), HEX);
        Serial.println(": FEHLER bei Konfiguration/Kalibrierung!");
        uint16_t failedReg = radar.getLastFailedConfigRegister();
        if (failedReg != 0xFFFF)
        {
            Serial.print("  -> I2C-Schreibfehler bei Register 0x");
            Serial.println(failedReg, HEX);
        }

        return false;
    }
    else
    {
        Serial.print("Sensor 0x");
        Serial.print(radar.getAddress(), HEX);
        Serial.println(": bereit.");
        return true;
    }
}

void setup()
{
    Serial.begin(115200);

    RADARWIRE.begin();
    RADARWIRE.setClock(100000); // niedriger Takt, mehr Toleranz fuer Clock-Stretching des XM125

    for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
    {
        if (!radars[i]->begin(RADARWIRE))
        {
            Serial.print("Radar sensor ");
            Serial.print(i);
            Serial.print(" (I2C addr 0x");
            Serial.print(radars[i]->getAddress(), HEX);
            Serial.print(") connected.");
        }
        else
        {
            Serial.print("Radar sensor ");
            Serial.print(i);
            Serial.print(" (I2C addr 0x");
            Serial.print(radars[i]->getAddress(), HEX);
            Serial.print(") connected.");
        }

        calibrateSensor(*radars[i]);
    }

#ifdef PDS_ENABLED
    PDS.init();

#endif
}

void loop()
{
    for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
    {
        if (!radars[i]->isConnected() || radars[i]->getMeasureError() || radars[i]->getCalibrationNeeded())
        {
            Serial.print("Sensor 0x");
            Serial.print(radars[i]->getAddress(), HEX);
            Serial.println(" nicht erreichbar oder fehlerhaft (Messung/Kalibrierung):");
            if (radars[i]->getCalibrationNeeded())
            {
                Serial.println("  -> Kalibrierung erforderlich, kalibriere neu.");
                calibrateSensor(*radars[i]);
            }
            else if (radars[i]->getMeasureError())
            {
                Serial.println("  -> Messfehler, starte neu.");
                radars[i]->startMeasurement();
            }
            else
            {
                Serial.println("  -> Sensor nicht erreichbar, pruefe Verkabelung.");
            }
            continue;
        }

        if (radars[i]->isMeasuring())
        {
            // Messung laeuft noch, Ergebnis noch nicht fertig
            continue;
        }
        else
        {
            // Messung fertig, Ergebnis abrufen
            const XM125DistanceResult &res = radars[i]->getLastResult();
            Serial.print("[0x");
            Serial.print(radars[i]->getAddress(), HEX);
            Serial.print("] Number of Peaks: ");
            Serial.println(res.numDistances);
            for (uint8_t p = 0; p < res.numDistances; p++)
            {
                Serial.print(", Peak ");
                Serial.print(p);
                Serial.print(": DistanceRaw=");
                Serial.print(res.peakDistanceRaw[p]);
                Serial.print(", Strength=");
                Serial.print(res.peakStrength[p]);
            }

#ifdef PDS_ENABLED
            PDS.Channel(150 + i, res.peakDistanceRaw[0]);
#endif
        }

        radars[i]->startMeasurement();
    }

#ifdef PDS_ENABLED
    PDS.update();
#endif
}
