#pragma once
#include <Arduino.h>

/**
 * enum_esp.h  --  Pins des ESP32-S3 (Seeed XIAO) auf der Platine 2vs2-Lightweight-Main V2.0
 *
 * Quelle: Schaltplan Blatt 9. Ausschliesslich XIAO-Namen D0 ... D10 (stehen im Arduino-Core des Boards),
 * nie rohe GPIO-Nummern: das Symbol im Plan traegt noch die GPIO-Namen des C3.
 * Teensy-Pins: enum.h (nicht auf dem ESP einbinden, dort gibt es Serial6/Serial7 nicht).
 */

static constexpr uint8_t ESP_PIN_SERVO = D0;   // SERVO_PWM
static constexpr uint8_t ESP_PIN_PIEZO = D1;   // PIEZO_PWM
static constexpr uint8_t ESP_PIN_SWITCH2 = D2; // SWITCH2
static constexpr uint8_t ESP_PIN_SPI_CS = D3;  // SPI_CS (Maus-Stecker, nicht benutzt)
static constexpr uint8_t ESP_PIN_SDA = D4;     // SDA_ESP_3V3: BNO055, Intercon
static constexpr uint8_t ESP_PIN_SCL = D5;     // SCL_ESP_3V3
static constexpr uint8_t ESP_PIN_LINK_TX = D6; // zum Teensy-RX (Pin 25, SUBCPU_RX)
static constexpr uint8_t ESP_PIN_LINK_RX = D7; // vom Teensy-TX (Pin 24, SUBCPU_TX)
// D8 (SCK), D9 (MISO), D10 (MOSI) gehen an den Maus-Stecker; ein Maus-Sensor wird nicht benutzt.
static constexpr uint8_t ESP_PIN_SPI_SCK = D8;
static constexpr uint8_t ESP_PIN_SPI_MISO = D9;
static constexpr uint8_t ESP_PIN_SPI_MOSI = D10;
