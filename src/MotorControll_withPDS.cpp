#include <Arduino.h>
#include <PDS.h>
#include <systemlib.h>
#include "Servo.h"

systemlib sys;
Servo RadarServo ;
PowerDebugger PDS;

float param1 = 0;
float param2 = 0;
float timer = 0;
float Akku_V = 0;
float Akku_I = 0;
float Dist = 0;
float feld_x_Valid=0;
float feld_y_Valid=0;
float feld_x_pos=0;
float feld_y_pos=0;
float pos_x_1 = 0;
float pos_y_1 = 0;
float pos_x_2 = 0;
float pos_y_2 = 0;

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

  PDS.bind(0,&param1,"Return");
  PDS.bind(1,&param2,"Return_t2");
  PDS.bind(6,&sys.RadarVo.Dist,"RadarVo1");
  PDS.bind(7,&sys.RadarLi.Dist,"RadarVo2");
  PDS.bind(8,&sys.RadarHi.Dist,"RadarVo3");
  PDS.bind(9,&sys.RadarRe.Dist,"RadarVo4");
  PDS.bind(10,&feld_x_Valid,"RadarVo3");
  PDS.bind(11,&feld_y_Valid,"RadarVo4");
  PDS.bind(103,&feld_x_pos,"RadarVo3");
  PDS.bind(104,&feld_y_pos,"RadarVo4");
  PDS.bind(2,&timer);

  PDS.bind(180,&Akku_V);
  PDS.bind(181,&Akku_I);

  sys.calibRadar();
}

void loop()
{
  if(PDS.getParamBool(3)){
    sys.calibRadar();
    delay(1000);
  }

  sys.cfgP.Kp = PDS.getParam(0);
  sys.cfgP.Kd = PDS.getParam(1);
  sys.cfgP.Ki = PDS.getParam(2);
  sys.setHeadingControllerConfig();

  feld_x_Valid = abs(sys.RadarLi.Dist+sys.RadarRe.Dist-(1830-60));
  feld_y_Valid = abs(sys.RadarHi.Dist+sys.RadarVo.Dist-(2430-60));

  pos_x_1 = sys.RadarLi.Dist/10;
  pos_y_1 = sys.RadarHi.Dist/10;

  pos_x_2 = (1830-sys.RadarRe.Dist)/10;
  pos_y_2 = (2430-sys.RadarVo.Dist)/10;

  feld_x_pos = ((pos_x_1+pos_x_2)/2)-(183/2);
  feld_y_pos = ((pos_y_1+pos_y_2)/2)-(243/2);

  sys.readSensors();

  if(abs(sys.giveOrbitDir())<90){
    RadarServo.write(sys.giveOrbitDir()+90);
  }
  else{
    RadarServo.write(90);
  }

  param1 = 30;//sys.giveHeadingController();

  sys.MoDataVL.Speed = abs(param1);
  if(param1<0){sys.MoDataVL.Dir = true;}
  else{sys.MoDataVL.Dir = false;}
  sys.MoDataVL.Enable = sys.giveExpander(0,3);

  sys.MoDataVR.Speed = abs(param1);
  if(param1<0){sys.MoDataVR.Dir = true;}
  else{sys.MoDataVR.Dir = false;}
  sys.MoDataVR.Enable = sys.giveExpander(0,3);

  sys.MoDataHL.Speed = abs(param1);
  if(param1<0){sys.MoDataHL.Dir = true;}
  else{sys.MoDataHL.Dir = false;}
  sys.MoDataHL.Enable = sys.giveExpander(0,3);

  sys.MoDataHR.Speed = abs(param1);
  if(param1<0){sys.MoDataHR.Dir = true;}
  else{sys.MoDataHR.Dir = false;}
  sys.MoDataHR.Enable = sys.giveExpander(0,3);

  Akku_I = sys.giveMainCurrent();
  Akku_V = sys.giveMainVoltage();

  timer = millis()/1000;
  PDS.update();
  //param1 = PDS.getFastParam(0);
}