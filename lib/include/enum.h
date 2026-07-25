#pragma once
#include <Arduino.h>

#define USBC_UART Serial
#define UART_DBG Serial3

#define LED_LENGTH 19

#define MotorVL_ID 0
#define MotorVR_ID 1
#define MotorHL_ID 2
#define MotorHR_ID 3
#define MotorDR_ID 4

enum PinOut
{
    MOT1_IN1 = 0,
    MOT1_IN2 = 1,
    MOT2_IN1 = 2,
    MOT2_IN2 = 3,
    MOT1_PWM = 4,
    MOT2_PWM = 5,
    MOT3_PWM = 6,
    MOT3_IN1 = 7,
    MOT3_IN2 = 8,
    MOT4_IN1 = 26,
    MOT4_IN2 = 32,
    MOT4_PWM = 9,

    BLDC_DRIBBLER_PWM = 33,

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

    ADC108S102_CS1 = 32
};