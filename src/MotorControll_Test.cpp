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
  PDS.bind("angle", &angle, 4, "A");
  PDS.bind("speed", &speed, 5, "A");
  PDS.bind("bno", &param2, 6, "A");

  PDS.bind("RadarVo", &sys.Radar[RadarVo].Dist, 7, "mm");
  PDS.bind("RadarLi", &sys.Radar[RadarLi].Dist, 8, "mm");
  PDS.bind("RadarHi", &sys.Radar[RadarHi].Dist, 9, "mm");
  PDS.bind("RadarRe", &sys.Radar[RadarRe].Dist, 10, "mm");
  PDS.bind("Position", &sys.Position.Certainty, 11, "%");
  PDS.bind("Feld_Y_Pos", &sys.Position.Y, 103, " cm");
  PDS.bind("Feld_X_Pos", &sys.Position.X, 104, " cm");

  PDS.Channel(99,LineSensorSchwelle[0]);

  sys.calibBNO();
}

void loop()
{
  if (PDS.getFastParam(3)>20)
  {
    sys.calibBNO();
    delay(1000);
  }

  PDS.Channel(12,sys.giveLineAngle(),"LineAngle");
  PDS.Channel(13,sys.giveLineDepth(),"LineDepth");
  PDS.Channel(14,sys.giveLinePresent(),"LinePresent");
  for ( int i = 47 ; i>=0; i--){
    PDS.Channel((50+i),sys.line_array[i]);
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