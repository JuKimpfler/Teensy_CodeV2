#pragma once

/**
 * params_sources.h  --  Wo laufen BNO055, Radar und IR-Ring: am Teensy oder am ESP32-S3?
 *
 * DEBUG-FLAG zum Umstellen. Der Wert muss in BEIDEN Firmwarestaenden gleich sein (platformio.ini des Teensy und
 * esp/platformio.ini, je als build_flag); die Pruefsumme im HELLO meldet einen Unterschied.
 *
 *   -DSENSOR_SOURCE=0   alles am Teensy (Uebergang, ESP-Firmware bleibt am Bus passiv)
 *   -DSENSOR_SOURCE=1   alles am ESP (Ziel: der Teensy wartet nie auf I2C-Sensoren)
 *
 * Einzeln umstellbar (ueberschreibt SENSOR_SOURCE fuer dieses Geraet):
 *   -DHEADING_SOURCE=0|1   BNO055
 *   -DRADAR_SOURCE=0|1     vier XM125-Radare
 *   -DIR_SOURCE=0|1        IR-Ring
 *
 * Verkabelung: BNO055, Radar und IR-Ring haengen am ESP-Bus (Stecker J14); mit geschlossenen Jumpern SDA_Jumper/
 * SCL_Jumper ist der Teensy (Wire, Pins 18/19) am selben Bus. Am Bus darf nur EIN Master senden:
 *   - Quelle Teensy: ESP-Firmware flashen ist harmlos (sie fasst den Bus dann nicht an), Jumper bleiben zu.
 *   - Quelle ESP: Jumper oeffnen, damit der Teensy den Bus nicht mehr treibt.
 * Steckt ein Geraet am Teensy-Stecker J13 (Wire1, 100 kHz noetig, I2C_WIRE1_CLOCK_HZ), gehoert es zur Quelle Teensy.
 */

#define SENSOR_SOURCE_TEENSY 0
#define SENSOR_SOURCE_ESP 1

#ifndef SENSOR_SOURCE
#define SENSOR_SOURCE SENSOR_SOURCE_TEENSY
#endif

#define HEADING_SOURCE_DIRECT SENSOR_SOURCE_TEENSY // BNO055 am Teensy
#define HEADING_SOURCE_LINK SENSOR_SOURCE_ESP      // BNO055 am ESP, Winkel ueber den Link
#define RADAR_SOURCE_TEENSY SENSOR_SOURCE_TEENSY
#define RADAR_SOURCE_ESP SENSOR_SOURCE_ESP
#define IR_SOURCE_TEENSY SENSOR_SOURCE_TEENSY
#define IR_SOURCE_ESP SENSOR_SOURCE_ESP

#ifndef HEADING_SOURCE
#define HEADING_SOURCE SENSOR_SOURCE
#endif
#ifndef RADAR_SOURCE
#define RADAR_SOURCE SENSOR_SOURCE
#endif
#ifndef IR_SOURCE
#define IR_SOURCE SENSOR_SOURCE
#endif

/// true, wenn mindestens ein Geraet am ESP haengt (dann braucht der Teensy den Link und wartet beim Start auf den ESP)
#define SENSORS_ANY_ON_ESP ((HEADING_SOURCE == SENSOR_SOURCE_ESP) || (RADAR_SOURCE == SENSOR_SOURCE_ESP) || (IR_SOURCE == SENSOR_SOURCE_ESP))
