#pragma once
#include <Arduino.h>

/* ============================================================
 *  Pinbelegung — Stand der Platinen in pcbs/
 *  (Main V2.0 vom 18.09.26, Bottom, DasSwitchBoard V2.0)
 *  Referenz ist die Hardware-Karte in src/CheckSystems.cpp.
 * ============================================================ */

#define USBC_UART Serial
// DBG-INTERCON (Teensy 0/1) -> Debugger-Board/RPi. PDS.cpp sieht diese Datei
// nicht, darum steht derselbe Wert auch in den build_flags (platformio.ini).
#ifndef UART_DBG
#define UART_DBG Serial1
#endif
#define UART_CAM1   Serial2 // CAM1, Teensy 7/8
#define UART_CAM2   Serial7 // CAM2, Teensy 28/29
#define UART_DIO    Serial3 // UART/DIO-Stecker J6, Teensy 14/15
#define UART_SUBCPU Serial6 // ESP32-C3 SubCPU, Teensy 24/25

#define LED_LENGTH 19

#define MotorVR_ID 0
#define MotorVL_ID 1
#define MotorHL_ID 2
#define MotorHR_ID 3
#define MotorDR_ID 4

enum PinOut
{
    /*
    Rad -> ESCON-Modul (MO1..MO4, Main Blatt 6/7). Steht in keinem Plan,
    sondern haengt an der Verkabelung der Motoren (bestaetigt 30.10.26):
        MO1 = VR    MO2 = HR    MO3 = HL    MO4 = VL
    Stimmt das nicht, alle Zeilen eines Rads zusammen tauschen -- PWM,
    Expanderbits und ADC-Kanaele gehoeren immer zu einem MOx.
    */

    /* ---- Sollwert (ESCON AI1+) ---- */
    MotorVR_PWM = 3, // MO1_SPEED
    MotorHR_PWM = 4, // MO2_SPEED
    MotorHL_PWM = 5, // MO3_SPEED
    MotorVL_PWM = 6, // MO4_SPEED
    MotorDR_PWM = 2, // BLDC_PWM, Dribbler-ESCON (Main Blatt 13)

    /*
    ESCON-Steuerleitungen auf den Portexpandern, als Bitnummer 0..15:
         0..7   IC9 (0x22) P0..P7   MO1, MO2, Dribbler
         8..15  IC7 (0x20) P0..P7   MO3, MO4, SWITCH1/BUTTON1
    Je Modul drei Leitungen am Stueck:
        +0  MOx_DO1   -> ESCON DI2   Freigabe (HIGH = frei)
        +1  MOx_DIO3  -> ESCON DI3   Richtung
        +2  MOx_DIO4  -> ESCON DI4   Stopp    (HIGH = Stopp, angenommen)
    */
    MotorVR_EN = 0,
    MotorVR_DIR = 1,
    MotorVR_STOP = 2,
    MotorHR_EN = 3,
    MotorHR_DIR = 4,
    MotorHR_STOP = 5,
    MotorDR_EN = 6,  // BLDC_DIO1, Dribbler hat keine Stopp-Leitung
    MotorDR_DIR = 7, // BLDC_DIO2
    MotorHL_EN = 8,
    MotorHL_DIR = 9,
    MotorHL_STOP = 10,
    MotorVL_EN = 11,
    MotorVL_DIR = 12,
    MotorVL_STOP = 13,
    MAIN_SWITCH1 = 14, // Eingang
    MAIN_BUTTON1 = 15, // Eingang

    /*
    Motor-ADC IC10 (ADC108S102, Main Blatt 5): IN0..IN7 = MO1_AO1, MO1_AO2,
    MO2_AO1, ... MO4_AO2. Was AO1/AO2 ausgeben, legt die ESCON-Konfiguration
    fest -- angenommen wie auf der alten Platine AO2 = Drehzahl, AO1 = Strom.
    Nachpruefen mit CheckSystems, Stufe 12 ('m').
    */
    MotorVR_RPM = 1,
    MotorVR_CUR = 0,
    MotorHR_RPM = 3,
    MotorHR_CUR = 2,
    MotorHL_RPM = 5,
    MotorHL_CUR = 4,
    MotorVL_RPM = 7,
    MotorVL_CUR = 6,

    /* ---- Dribbler-ESCON direkt am Teensy ---- */
    MotorDR_AO1 = 27, // BLDC_AO1, analog
    MotorDR_AO2 = 26, // BLDC_AO2, analog
    MotorDR_DO1 = 20, // BLDC_DO1

    /* ---- IC8 (0x23): Freigaben, Bitnummern ---- */
    VDRIB_EN = 0,            // Dribblerspannung, zusaetzlich Schalter SW19
    VMOT_EN = 1,             // Motorspannung, zusaetzlich Schalter SW3
    LIGHT_BARRIER_EN = 2,    // LED der Lichtschranke
    LINE_EN = 3,             // Linien-LEDs (LINE_EN_T)
    UART_DIO1 = 4,           // DIO zum UART/DIO-Stecker
    I2C_INTERCON_3V3_EN = 5, // 3V3 am I2C-INTERCON, LOW = ein

    KICKER_PORT = 9,

    ROBO_ID = 21,       // Roboter 1 / 2
    RGB_DATA_3V3 = 22,  // WS2812B
    LIGHT_BARRIER = 23, // LDR_IN, analog

    /* ---- SPI: Motor-ADC und SPI-INTERCON zur Bottom-Platine ---- */
    SPI_TNS_CS = 10,
    SPI_MOSI = 11,
    SPI_MISO = 12,
    SPI_SCK = 13,
    ADC108S102_Motor_CS = 30,
    SPI2_DEC_A = 31, // 74HC138 (Bottom): Adresse 0..5 = Linien-ADC 1..6, 6 = Maus, 7 = keiner
    SPI2_DEC_B = 32,
    SPI2_DEC_C = 33,

    /* ---- I2C ---- */
    I2C_TNS_SCL = 16, // Wire1
    I2C_TNS_SDA = 17,
    I2C_ESP_SDA = 18, // Wire, ueber Jumper JP1/JP2
    I2C_ESP_SCL = 19,

    /* ---- UART ---- */
    DBG_RX = 0,
    DBG_TX = 1,
    UART1_TNS_RX = 7, // CAM1
    UART1_TNS_TX = 8,
    UART_DIO_TX = 14,
    UART_DIO_RX = 15,
    SUBCPU_TX = 24,
    SUBCPU_RX = 25,
    UART2_TNS_RX = 28, // CAM2
    UART2_TNS_TX = 29
};
