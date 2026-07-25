#include "Motor.h"

void Motor::setMotor(u_int8_t MotorNum, int MotorSpeed)
{

    bool in1, in2, dir;

    if (MotorSpeed == 0)
        in1 = false, in2 = false;
    else
        dir = abs(MotorSpeed) == MotorSpeed;

    /*HIGH, LOW wenn positiv; LOW, HIGH wenn negativ*/
    in1 = dir, in2 = !dir;

    switch (MotorNum)
    {
    case 1:
        digitalWrite(MOT1_IN1, in1);
        digitalWrite(MOT1_IN2, in2);
        analogWrite(MOT1_PWM, abs(MotorSpeed));
        break;

    case 2:
        digitalWrite(MOT2_IN1, in1);
        digitalWrite(MOT2_IN2, in2);
        analogWrite(MOT2_PWM, abs(MotorSpeed));
        break;

    case 3:
        digitalWrite(MOT3_IN1, in1);
        digitalWrite(MOT3_IN2, in2);
        analogWrite(MOT3_PWM, abs(MotorSpeed));
        break;

    case 4:
        digitalWrite(MOT4_IN1, in1);
        digitalWrite(MOT4_IN2, in2);
        analogWrite(MOT4_PWM, abs(MotorSpeed));
        break;

    default:
        break;
    }
}
