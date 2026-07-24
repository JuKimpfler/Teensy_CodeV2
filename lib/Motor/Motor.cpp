#include "Motor.h"
#include "MotorEnum.h"

void Motor::setMotor(u_int8_t MotorNum, int MotorSpeed){

    bool in1;
    bool in2;
    if(MotorSpeed == 0){
        in1, in2 = false, false;
    }

    bool sign = abs(MotorSpeed) == MotorSpeed;

    /*HIGH, LOW wenn positiv; LOW, HIGH wenn negativ*/
    in1, in2 = sign, !sign;

	switch (MotorNum){
        case 1:
            digitalWrite(MotorEnum::MOT1_IN1, in1);
            digitalWrite(MotorEnum::MOT1_IN2, in2);
            analogWrite(MotorEnum::MOT1_PWM, abs(MotorSpeed));
            break;

        case 2:
            digitalWrite(MotorEnum::MOT2_IN1, in1);
            digitalWrite(MotorEnum::MOT2_IN2, in2);
            analogWrite(MotorEnum::MOT2_PWM, abs(MotorSpeed));
            break;
        
        case 3:
            digitalWrite(MotorEnum::MOT3_IN1, in1);
            digitalWrite(MotorEnum::MOT3_IN2, in2);
            analogWrite(MotorEnum::MOT3_PWM, abs(MotorSpeed));
            break;

        case 4:
            digitalWrite(MotorEnum::MOT4_IN1, in1);
            digitalWrite(MotorEnum::MOT4_IN2, in2);
            analogWrite(MotorEnum::MOT4_PWM, abs(MotorSpeed));
            break;

        default:
            break;
    }
}
