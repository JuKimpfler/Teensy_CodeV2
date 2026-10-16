#pragma once
#include <Arduino.h>

/**
 * enum.h  --  Pins, Schnittstellen und Hardware-IDs der Platine 2vs2-Lightweight-Main V2.0
 *
 * Quelle: Schaltplan Blatt 1 (Teensy), 4 (Port-Expander), 5 (Motor-ADC), 9 (ESP32-S3).
 * Alle Parameter (Takte, Schwellen, Shunts): lib/include/params/.
 */

/* -------------- Serielle Schnittstellen -------------- */
#define USBC_UART Serial
// PDS-Port, Pins 0/1 (DBG_RX, DBG_TX). PDS.cpp sieht enum.h nicht: der Wert kommt zusaetzlich als
// build_flag (-DUART_DBG=Serial1) aus platformio.ini, sonst nimmt params_PDS.h Serial3.
#ifndef UART_DBG
#define UART_DBG Serial1
#endif
#define UART_CAM1 Serial2 // Kamera 1, Pins 7/8 (UART1_RX, UART1_TX)
#define UART_DIO Serial3  // Schuss, Pins 14/15 (UART_DIO_TX, UART_DIO_RX)
#define UART_ESP Serial6  // Link zum ESP32-S3, Pins 24/25 (SUBCPU_TX, SUBCPU_RX)
#define UART_CAM2 Serial7 // Kamera 2, Pins 28/29 (UART2_RX, UART2_TX)

#define LED_LENGTH 19

/* -------------- Motor-IDs -------------- */
#define MotorVR_ID 0
#define MotorVL_ID 1
#define MotorHL_ID 2
#define MotorHR_ID 3
#define MotorDR_ID 4

/* -------------- Teensy-Pins -------------- */
enum PinOut
{
    // PWM-Sollwert an die ESCON-Module (RC-gefiltert auf AI1+).
    // Motor 1 = VL, Motor 2 = HL, Motor 3 = VR, Motor 4 = HR
    MotorDR_PWM = 2, // BLDC_PWM (Dribbler)
    MotorVL_PWM = 3, // MO1_SPEED
    MotorHL_PWM = 4, // MO2_SPEED
    MotorVR_PWM = 5, // MO3_SPEED
    MotorHR_PWM = 6, // MO4_SPEED

    KICKER_PORT = 9,

    // SPI (SPI_TNS_*), Bus `SPI`
    SPI_CS_INTERCON = 10,
    SPI_MOSI = 11,
    SPI_MISO = 12,
    SPI_SCK = 13,

    // I2C: Teensy-Bus = Wire1; ESP-Bus = Wire (nur ueber SDA_Jumper/SCL_Jumper, Standard getrennt)
    I2C_SCL_3V3 = 16,
    I2C_SDA_3V3 = 17,
    I2C_SDA_ESP = 18,
    I2C_SCL_ESP = 19,

    BLDC_DO1 = 20, // Dribbler-Richtung (ESCON DI2), Ausgang; Enable und Bremse: Expander 0x22 P6/P7
    ROBO_ID = 21,  // Roboter-ID (Jumper JP4, 10k Pull-Up)
    RGB_DATA_3V3 = 22,
    LIGHT_BARRIER = 23, // LDR_IN, analog

    BLDC_AO2 = 26, // Dribbler-ESCON, analoge Rueckmeldung (A12)
    BLDC_AO1 = 27, // Dribbler-ESCON, analoge Rueckmeldung (A13)

    // Chip-Selects am Bus `SPI`
    ADC108S102_Motor_CS = 30,
    LINE_DEC_A = 31, // SPI2_DEC_A, niedrigstwertiges Adressbit der Linien-Platine
    LINE_DEC_B = 32,
    LINE_DEC_C = 33,

    // ADC108S102 Motor-ADC (Blatt 5): IN0 = MO1_AO1 ... IN7 = MO4_AO2
    // AP1 = Drehzahl, AP2 = Strom (Annahme wie bisher, haengt von der ESCON-Konfiguration ab)
    MotorVL_AP1 = 0,
    MotorVL_AP2 = 1,
    MotorHL_AP1 = 2,
    MotorHL_AP2 = 3,
    MotorVR_AP1 = 4,
    MotorVR_AP2 = 5,
    MotorHR_AP1 = 6,
    MotorHR_AP2 = 7
};

/* -------------- I2C-Adressen -------------- */
enum I2cAddress : uint8_t
{
    EXP_ADDR_MOTOR_B = 0x20, // IC7: Motor 3 + 4, SWITCH1, BUTTON1
    EXP_ADDR_MOTOR_A = 0x22, // IC9: Motor 1 + 2, Dribbler
    EXP_ADDR_SYSTEM = 0x23,  // IC8: Leistungsschalter und Freigaben
    INA_ADDR_MAIN = 0x40,    // Akku, 5 mOhm
    INA_ADDR_MOTOR = 0x41,   // Motorversorgung, 5 mOhm
    INA_ADDR_DRIB = 0x45     // Dribbler, 10 mOhm
};

/* -------------- Port-Expander: Bit n = Pin Pn --------------
 * Je Motor haengen drei Expander-Pins am ESCON (Angabe des Teams):
 *   DO1  -> ESCON DI2  = Richtung
 *   DIO3 -> ESCON DIO3 = Enable
 *   DIO4 -> ESCON DIO4 = Bremse (Break)
 * Pegel (Annahme, am Roboter pruefen): Richtungsbit 0 bei Dir = true, Enable hoch = an, Bremse hoch = Bremse an. */

// Expander 0x22 (IC9): Motor 1 (VL), Motor 2 (HL), Dribbler (Enable, Bremse)
enum ExpMotorA : uint8_t
{
    MotorVL_DIR_BIT = 0, // MO1_DO1
    MotorVL_EN_BIT = 1,  // MO1_DIO3
    MotorVL_BRK_BIT = 2, // MO1_DIO4
    MotorHL_DIR_BIT = 3, // MO2_DO1
    MotorHL_EN_BIT = 4,  // MO2_DIO3
    MotorHL_BRK_BIT = 5, // MO2_DIO4
    MotorDR_EN_BIT = 6,  // BLDC_DIO1
    MotorDR_BRK_BIT = 7  // BLDC_DIO2
};

// Expander 0x20 (IC7): Motor 3 (VR), Motor 4 (HR), Schalter
enum ExpMotorB : uint8_t
{
    MotorVR_DIR_BIT = 0, // MO3_DO1
    MotorVR_EN_BIT = 1,  // MO3_DIO3
    MotorVR_BRK_BIT = 2, // MO3_DIO4
    MotorHR_DIR_BIT = 3, // MO4_DO1
    MotorHR_EN_BIT = 4,  // MO4_DIO3
    MotorHR_BRK_BIT = 5, // MO4_DIO4
    SWITCH1_BIT = 6,     // Schalter, hoch = EIN
    BUTTON1_BIT = 7      // Taster, niedrig = gedrueckt
};

// Expander 0x23 (IC8): Leistungsschalter und Freigaben
enum ExpSystem : uint8_t
{
    VDRIB_EN_BIT = 0,           // hoch = Dribbler-Versorgung an
    VMOT_EN_BIT = 1,            // hoch = Motorversorgung an (zusaetzlich Schalter SW3)
    LIGHT_BARRIER_EN_BIT = 2,   // niedrig = Lichtschranken-LED an
    LINE_EN_T_BIT = 3,          // Freigabe der Linien-Platine, Pegel siehe LINE_EN_LEVEL
    UART_DIO1_BIT = 4,          // DIO der Schuss-Schnittstelle
    I2C_INTERCON_3V3_EN_BIT = 5 // niedrig = geschaltete 3V3 am ESP-Bus-Intercon an
};
