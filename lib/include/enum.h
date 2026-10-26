#pragma once
#include <Arduino.h>

#define USBC_UART Serial
#define UART_DBG Serial3

#define LED_LENGTH 19

#define MotorVR_ID 0 
#define MotorVL_ID 1
#define MotorHL_ID 2
#define MotorHR_ID 3
#define MotorDR_ID 4

enum PinOut
{
    /*
    4 -> 9 / 4
    1 -> 4 / 9
    2 -> 5 / 6
    3 -> 6 / 5*/

    MotorVR_PWM = 4,
    MotorVL_PWM = 9,
    MotorHL_PWM = 6,
    MotorHR_PWM = 5,
    MotorDR_PWM = 33,

    MotorVR_P1 = 6,
    MotorVR_P2 = 7,
    MotorVL_P1 = 0,
    MotorVL_P2 = 1,
    MotorHL_P1 = 2,
    MotorHL_P2 = 3,
    MotorHR_P1 = 4,
    MotorHR_P2 = 5,

    MotorVR_AP1 = 1,
    MotorVR_AP2 = 0,
    MotorVL_AP1 = 7,
    MotorVL_AP2 = 6,
    MotorHL_AP1 = 5,
    MotorHL_AP2 = 4,
    MotorHR_AP1 = 3,
    MotorHR_AP2 = 2,

    /*
    1 RPM VR
    0 Strom VR
    7 RPM VL
    6 Strom VL
    5 RPM HL
    4 Strom HL
    3 RPM HR
    2 Strom HR
    */

    KICKER_PORT = 18,

    START_SWITCH = 10,

    RGB_DATA_3V3 = 20,

    SPI_MOSI = 11,
    SPI_MISO = 12,
    SPI_SCK = 13,
    SPI_DIO0 = 19,
    SPI_DIO1 = 31,
    SPI_DIO2 = 30,
    SPI_DIO3 = 21,
    SPI_DIO4 = 22,

    I2C_SCL_3V3 = 16,
    I2C_SDA_3V3 = 17,

    TNS_TX_PIXY_RX = 14,
    TNS_RX_PIXY_TX = 15,

    UART1_TNS_TX = 24,
    UART1_TNS_RX = 25,
    UART2_TNS_TX = 28,
    UART2_TNS_RX = 29, 

    LIGHT_BARRIER = 23,

    EMPTY_27 = 27,

    ADC108S102_Motor_CS = 32
};