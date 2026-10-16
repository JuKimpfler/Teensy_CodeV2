#pragma once
#include <Arduino.h>
#include "params_radar.h"
#include "params_sources.h" // SENSOR_SOURCE, HEADING_/RADAR_/IR_SOURCE

/**
 * params_link.h  --  Verbindung Teensy <-> ESP32-S3 (lib/Link, lib/SensorLink)
 *
 * Gilt fuer beide Seiten; die ESP-Firmware wird aus denselben Headern gebaut. Die Pruefsumme
 * LINK_PARAMS_HASH deckt alles ab, was beide Seiten gleich haben muessen (HELLO-Nachricht).
 */

static constexpr uint8_t LINK_PROTOCOL_VERSION = 1;

/* -------------- Uebertragung -------------- */
static constexpr uint32_t LINK_BAUD = 2000000;         ///< 8N1, keine Flusssteuerung; Fallback 921600
static constexpr uint16_t LINK_RX_BUFFER_BYTES = 1024; ///< Teensy-RX-Puffer (addMemoryForRead), keine Flusssteuerung!

/* -------------- Takte und Zeitlimits -------------- */
static constexpr uint16_t LINK_IMU_PERIOD_MS = 10;      ///< BNO055 liefert 100 Hz
static constexpr uint16_t LINK_HEALTH_PERIOD_MS = 200;  ///< ESP-Status
static constexpr uint16_t LINK_IR_PERIOD_MS = 10;       ///< IR-Ring: Winkel und Distanz
static constexpr uint16_t LINK_INPUT_REFRESH_MS = 1000; ///< Tasten: bei Aenderung sofort, sonst mindestens so oft
static constexpr uint16_t LINK_ALIVE_MS = 50;           ///< laenger kein gueltiger Rahmen: Link gilt als ausgefallen
static constexpr uint32_t LINK_BOOT_TIMEOUT_MS = 3000;  ///< so lange wartet der Teensy beim Start auf den ESP
static constexpr uint16_t LINK_HELLO_INTERVAL_MS = 500; ///< HELLO wiederholen, bis die Antwort da ist
static constexpr uint16_t LINK_PING_FAST_MS = 100;      ///< Zeitabgleich zu Beginn schnell ...
static constexpr uint8_t LINK_PING_FAST_COUNT = 10;     ///< ... fuer diese Anzahl Pings
static constexpr uint16_t LINK_PING_INTERVAL_MS = 1000; ///< danach 1 Hz
static constexpr uint8_t LINK_CLOCK_WINDOW = 8;         ///< Zeitabgleich nimmt das Ergebnis mit der kuerzesten Laufzeit der letzten Pings

/* -------------- Winkel -------------- */
static constexpr uint32_t LINK_EXTRAP_MAX_US = 25000;  ///< so weit wird der Winkel hoechstens mit der Drehrate weitergerechnet
static constexpr float LINK_GYRO_SIGN_DEFAULT = -1.0f; ///< Drehrate Z (BNO055, gegen den Uhrzeigersinn positiv) zu Winkel (im Uhrzeigersinn positiv); wird im Betrieb nachgelernt

/* -------------- Pruefsumme der gemeinsamen Parameter -------------- */
constexpr uint32_t linkHashStep(uint32_t h, uint32_t value)
{
    for (int i = 0; i < 4; i++)
    {
        h ^= (value >> (8 * i)) & 0xFFu;
        h *= 16777619u; // FNV-1a
    }
    return h;
}

/// Gemeinsame Parameter beider Firmwarestaende: stimmt die Summe nicht, meldet der Teensy einen Fehler.
constexpr uint32_t linkParamsHash()
{
    uint32_t h = 2166136261u;
    h = linkHashStep(h, LINK_PROTOCOL_VERSION);
    h = linkHashStep(h, LINK_BAUD);
    h = linkHashStep(h, LINK_IMU_PERIOD_MS);
    h = linkHashStep(h, HEADING_SOURCE);
    h = linkHashStep(h, RADAR_SOURCE);
    h = linkHashStep(h, IR_SOURCE);
    h = linkHashStep(h, RADAR_START_MM);
    h = linkHashStep(h, RADAR_END_MM);
    h = linkHashStep(h, RADAR_SIGNAL_QUALITY);
    h = linkHashStep(h, RADAR_THRESHOLD_SENSITIVITY);
    h = linkHashStep(h, RADAR_PEAKS_TO_FETCH);
    h = linkHashStep(h, RADAR_MEASURE_TIMEOUT_MS);
    for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
        h = linkHashStep(h, RADAR_SENSOR_ADDRS[i]);
    return h;
}
static constexpr uint32_t LINK_PARAMS_HASH = linkParamsHash();
