#include <Arduino.h>
#include <PDS.h>
#include <systemlib.h>
#include "Servo.h"

systemlib sys;
Servo RadarServo;
// PowerDebugger PDS;

float param1 = 0;
float param2 = 0;
float timer = 0;
float Akku_V = 0;
float Akku_I = 0;
float Dist = 0;
float angle ;
float speed ;
float x ;
float y ;

void setup()
{
  RadarServo.attach(7);
  RadarServo.write(0);
  delay(1000);
  RadarServo.write(90);
  Serial.begin(115200);
  UART_DBG.begin(UART_DBG_BAUD);

  Serial.println("------------------------------- Hallo -------------------------------");
  sys.initialize();

  PDS.init();

  PDS.bind("Timer", &timer, 0, "s");
  PDS.bind("Akku_V", &Akku_V, 1, "V");
  PDS.bind("Akku_I", &Akku_I, 2, "A");
  PDS.bind("angle", &angle, 4);
  PDS.bind("speed", &speed, 5);
  PDS.bind("bno", &param2, 6);

  PDS.bind("RadarVo", &sys.RadarVo.Dist, 7, "mm");
  PDS.bind("RadarLi", &sys.RadarLi.Dist, 8, "mm");
  PDS.bind("RadarHi", &sys.RadarHi.Dist, 9, "mm");
  PDS.bind("RadarRe", &sys.RadarRe.Dist, 10, "mm");
  PDS.bind("Feld_X_Pos", &sys.feld_x_pos, 103, "*2 cm");
  PDS.bind("Feld_Y_Pos", &sys.feld_y_pos, 104, "*2 cm");

  sys.calibBNO();
}

void loop()
{
  if (PDS.getFastParam(3)>20)
  {
    sys.calibBNO();
    delay(1000);
  }

  

  sys.readSensors();

  param2 = sys.giveOrbitDir();

  x = PDS.getFastParam(1);
  y = PDS.getFastParam(0);
  angle = atan2(y, x) * RAD_TO_DEG;
  speed = min(100.0f, hypot(x, y))/2;

  if (PDS.getFastParam(2)>20)
  {
    sys.moveToPoint(50,50,50,0);
  }
  else{
    sys.moveDir(angle,speed,0);
  }

  //sys.moveDir(angle,speed,0);
  
  //sys.setOmniWheelVek(x,y,0);
  //sys.setHeading();
  //param1 = sys.giveOrbitDir()*-0.9;

  /*sys.setMotorData(sys.MoDataVL.ID,param1,true,false);
  sys.setMotorData(sys.MoDataVR.ID,param1,true,false);
  sys.setMotorData(sys.MoDataHL.ID,param1,true,false);
  sys.setMotorData(sys.MoDataHR.ID,param1,true,false);*/

  //sys.MoDataVR.Speed= 90;
  //sys.MoDataVR.Dir= true;
  //sys.MoDataVR.Enable = true;

  /*sys.MoDataVL.Speed = 3;
  sys.MoDataVL.Dir = true;
  sys.MoDataVL.Enable = true;

  sys.MoDataVR.Speed = 33;
  sys.MoDataVR.Dir = true;
  sys.MoDataVR.Enable = false;

  sys.MoDataHL.Speed = 90;
  sys.MoDataHL.Dir = false;
  sys.MoDataHL.Enable = false;

  sys.MoDataHR.Speed = 100;
  sys.MoDataHR.Dir = false;
  sys.MoDataHR.Enable = false;*/

  Akku_I = sys.giveMainCurrent();
  Akku_V = sys.giveMainVoltage();

  timer = millis() / 1000;
  PDS.update();
}