#include <Arduino.h>
#include "systemlib.h"

INA219 INA219_Main(0x40);

TCA9534DW EXP_MOT(&Wire1, 0x20);

BNO055 BNO(0x28, &Wire);

ADC108S102 ADC_MOT(&SPI1,32);
ADC108S102 ADC_IR1(&SPI1,30);
ADC108S102 ADC_IR2(&SPI1,31);

void setup()
{
    Serial.begin(115200);
    Wire1.begin();
    Wire.begin();
    SPI1.begin();
    SPI.begin();
    INA219_Main.begin();

    BNO.init();

    ADC_MOT.init();
    ADC_IR1.init();
    ADC_IR2.init();

    EXP_MOT.setup(false);
}

void loop()
{
  analogWrite(6,map(Util.AMap(BNO.eulHeading()),-180,180,0,255));
  EXP_MOT.writePin(3,digitalRead(10));
  EXP_MOT.writePin(4,digitalRead(10));
  Serial.print(map(Util.AMap(BNO.eulHeading()),-180,180,0,255));
  Serial.print(" , ist 0:");
  Serial.print(ADC_MOT.read(0));
  Serial.print(" , ist 1:");
  Serial.print(ADC_MOT.read(1));
  Serial.print(" , ist 2:");
  Serial.print(ADC_MOT.read(2));
  Serial.print(" , ist 3:");
  Serial.print(ADC_MOT.read(3));
  Serial.print(" , ist 1:");
  Serial.print(ADC_MOT.read(4));
  Serial.print(" , ist 2:");
  Serial.print(ADC_MOT.read(5));
  Serial.print(" , ist 3:");
  Serial.print(ADC_MOT.read(6));
  Serial.print(" , ist 3:");
  Serial.println(ADC_MOT.read(7));
}