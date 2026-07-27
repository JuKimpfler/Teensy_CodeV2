#include <Arduino.h>
#include "systemlib.h"

INA219 INA219_Main(0x40);

TCA9534DW EXP_MOT(&Wire1, 0x20);
TCA9534DW EXP_SW1(&Wire, 0x22);

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
    EXP_SW1.setup(true);
}

void loop()
{
  if(EXP_SW1.readPin(1)){analogWrite(6,map(Util.AMap(BNO.eulHeading()),-180,180,0,255));}else{analogWrite(6,0);}
  EXP_MOT.writePin(4,EXP_SW1.readPin(0));// Enable
  EXP_MOT.writePin(5,!digitalRead(10));// richtung
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
  Serial.print(ADC_MOT.read(4,true,true));
  Serial.print(" , ist 2:");
  Serial.print(ADC_MOT.read(5,true,false));
  Serial.print(" , ist 3:");
  Serial.print(ADC_MOT.read(6));
  Serial.print(" , ist 3:");
  Serial.println(ADC_MOT.read(7));
}