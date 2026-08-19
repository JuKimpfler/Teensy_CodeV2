#include <Arduino.h>
#include <PDS.h>
#include <systemlib.h>

systemlib sys;

PowerDebugger PDS;

float param1 = 0;
float param2 = 0;
float timer = 0;
float Akku_V = 0;
float Akku_I = 0;
float Dist = 0;

void setup()
{
  Serial.begin(115200);
  UART_DBG.begin(UART_DBG_BAUD);

  Serial.println("------------------------------- Hallo -------------------------------");
  sys.initialize();

  PDS.init();

  PDS.bind(0,&param1,"Return");
  PDS.bind(1,&param2,"Return_t2");
  PDS.bind(5,&sys.RadarVo.Dist,"RadarVo9");
  PDS.bind(6,&sys.RadarVo.isCon,"RadarVo1");
  PDS.bind(7,&sys.RadarLi.isCon,"RadarVo2");
  PDS.bind(8,&sys.RadarHi.isCon,"RadarVo3");
  PDS.bind(9,&sys.RadarRe.isCon,"RadarVo4");
  PDS.bind(10,&sys.RadarVo.Peaks,"RadarVo5");
  PDS.bind(2,&timer);

  PDS.bind(180,&Akku_V);
  PDS.bind(181,&Akku_I);
}

void loop()
{
  if(PDS.getParamBool(0)){
    sys.calibRadar();
    delay(1000);
  }

  sys.readSensors();

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
  param1 = PDS.getFastParam(0);
}