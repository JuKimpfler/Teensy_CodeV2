// Pinbelegung_Check -- Prueft zur Compile-Zeit, dass enum.h zum Schaltplan 2vs2-Lightweight-Main V2.0 passt.
//
// Es gibt nichts auszufuehren: baut die Datei, stimmt die Belegung. Eine Abweichung ist ein Compilerfehler mit
// dem Namen des Pins. Die Sollwerte stammen aus Blatt 1 (Teensy), 4 (Expander), 5 (Motor-ADC) und 13 (INA219).
// Motor 1 = VL, Motor 2 = HL, Motor 3 = VR, Motor 4 = HR (Angabe des Teams).
#include <Arduino.h>
#include <enum.h>

#define SOLL(name, wert) static_assert((name) == (wert), #name " stimmt nicht mit dem Schaltplan ueberein")

/* -------------- Teensy-Pins (Blatt 1) -------------- */
SOLL(MotorDR_PWM, 2); // BLDC_PWM
SOLL(MotorVL_PWM, 3); // MO1_SPEED
SOLL(MotorHL_PWM, 4); // MO2_SPEED
SOLL(MotorVR_PWM, 5); // MO3_SPEED
SOLL(MotorHR_PWM, 6); // MO4_SPEED
SOLL(KICKER_PORT, 9);
SOLL(SPI_CS_INTERCON, 10);
SOLL(SPI_MOSI, 11);
SOLL(SPI_MISO, 12);
SOLL(SPI_SCK, 13);
SOLL(I2C_SCL_3V3, 16);
SOLL(I2C_SDA_3V3, 17);
SOLL(I2C_SDA_ESP, 18);
SOLL(I2C_SCL_ESP, 19);
SOLL(BLDC_DO1, 20);
SOLL(ROBO_ID, 21);
SOLL(RGB_DATA_3V3, 22);
SOLL(LIGHT_BARRIER, 23);
SOLL(BLDC_AO2, 26);
SOLL(BLDC_AO1, 27);
SOLL(ADC108S102_Motor_CS, 30);
SOLL(LINE_DEC_A, 31);
SOLL(LINE_DEC_B, 32);
SOLL(LINE_DEC_C, 33);

// Kein Pin doppelt belegt (Pins 0/1, 7/8, 14/15, 24/25, 28/29 sind die UARTs, siehe enum.h)
constexpr uint8_t PINS[] = {0, 1, MotorDR_PWM, MotorVL_PWM, MotorHL_PWM, MotorVR_PWM, MotorHR_PWM, 7, 8, KICKER_PORT,
                            SPI_CS_INTERCON, SPI_MOSI, SPI_MISO, SPI_SCK, 14, 15, I2C_SCL_3V3, I2C_SDA_3V3,
                            I2C_SDA_ESP, I2C_SCL_ESP, BLDC_DO1, ROBO_ID, RGB_DATA_3V3, LIGHT_BARRIER, 24, 25,
                            BLDC_AO2, BLDC_AO1, 28, 29, ADC108S102_Motor_CS, LINE_DEC_A, LINE_DEC_B, LINE_DEC_C};
constexpr bool pinsUnique()
{
    for (unsigned i = 0; i < sizeof(PINS); i++)
        for (unsigned j = i + 1; j < sizeof(PINS); j++)
            if (PINS[i] == PINS[j])
                return false;
    return true;
}
static_assert(sizeof(PINS) == 34, "alle 34 Teensy-Pins sind belegt");
static_assert(pinsUnique(), "ein Teensy-Pin ist doppelt belegt");

/* -------------- Motor-ADC (Blatt 5): IN0 = MO1_AO1 ... IN7 = MO4_AO2 -------------- */
SOLL(MotorVL_AP1, 0); // MO1_AO1
SOLL(MotorVL_AP2, 1); // MO1_AO2
SOLL(MotorHL_AP1, 2); // MO2_AO1
SOLL(MotorHL_AP2, 3); // MO2_AO2
SOLL(MotorVR_AP1, 4); // MO3_AO1
SOLL(MotorVR_AP2, 5); // MO3_AO2
SOLL(MotorHR_AP1, 6); // MO4_AO1
SOLL(MotorHR_AP2, 7); // MO4_AO2

/* -------------- I2C-Adressen (Blatt 2, 4, 13) -------------- */
SOLL(EXP_ADDR_MOTOR_B, 0x20);
SOLL(EXP_ADDR_MOTOR_A, 0x22);
SOLL(EXP_ADDR_SYSTEM, 0x23);
SOLL(INA_ADDR_MAIN, 0x40);
SOLL(INA_ADDR_MOTOR, 0x41);
SOLL(INA_ADDR_DRIB, 0x45);

/* -------------- Port-Expander (Blatt 4) -------------- */
// IC9, 0x22: P0 MO1_DO1, P1 MO1_DIO3, P2 MO1_DIO4, P3 MO2_DO1, P4 MO2_DIO3, P5 MO2_DIO4, P6 BLDC_DIO1, P7 BLDC_DIO2
SOLL(MotorVL_EN_BIT, 0);
SOLL(MotorVL_DIR_BIT, 1);
SOLL(MotorVL_DIO4_BIT, 2);
SOLL(MotorHL_EN_BIT, 3);
SOLL(MotorHL_DIR_BIT, 4);
SOLL(MotorHL_DIO4_BIT, 5);
SOLL(MotorDR_DIR_BIT, 6);
SOLL(MotorDR_DIO2_BIT, 7);
// IC7, 0x20: P0 MO3_DO1, P1 MO3_DIO3, P2 MO3_DIO4, P3 MO4_DO1, P4 MO4_DIO3, P5 MO4_DIO4, P6 SWITCH1, P7 BUTTON1
SOLL(MotorVR_EN_BIT, 0);
SOLL(MotorVR_DIR_BIT, 1);
SOLL(MotorVR_DIO4_BIT, 2);
SOLL(MotorHR_EN_BIT, 3);
SOLL(MotorHR_DIR_BIT, 4);
SOLL(MotorHR_DIO4_BIT, 5);
SOLL(SWITCH1_BIT, 6);
SOLL(BUTTON1_BIT, 7);
// IC8, 0x23: P0 VDRIB_EN, P1 VMOT_EN, P2 LIGHT_BARRIER_EN, P3 LINE_EN_T, P4 UART_DIO1, P5 I2C_INTERCON_3V3_EN
SOLL(VDRIB_EN_BIT, 0);
SOLL(VMOT_EN_BIT, 1);
SOLL(LIGHT_BARRIER_EN_BIT, 2);
SOLL(LINE_EN_T_BIT, 3);
SOLL(UART_DIO1_BIT, 4);
SOLL(I2C_INTERCON_3V3_EN_BIT, 5);

void setup() {}
void loop() {}
