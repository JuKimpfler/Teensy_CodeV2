#include <Arduino.h>
#include <PDS.h>
#include <systemlib.h>

systemlib sys;

PowerDebugger PDS1;

float param1 = 0;
float param2 = 0;
float timer = 0;

void setup()
{
  Serial.begin(115200);
  UART_DBG.begin(UART_DBG_BAUD);

  Serial.println("------------------------------- Hallo -------------------------------");
  sys.initialize();
  PDS1.init();
  //PDS1.bind(0,&param1,"Return");
  //PDS1.bind(1,&param2,"Return_t2");
  //PDS1.bind(2,&timer,"Timer");
}

void loop()
{
  sys.readSensors();

  sys.MoDataVL.Speed = abs(param1);
  sys.MoDataVL.Dir = false;
  sys.MoDataVL.Enable = sys.giveExpander(0,3);

  sys.MoDataVR.Speed = abs(param1);
  sys.MoDataVR.Dir = false;
  sys.MoDataVR.Enable = sys.giveExpander(0,3);

  sys.MoDataHL.Speed = abs(param1);
  if(param1<0){sys.MoDataHL.Dir = true;}
  else{sys.MoDataHL.Dir = false;}
  sys.MoDataHL.Enable = sys.giveExpander(0,3);

  sys.MoDataHR.Speed = abs(param1);
  sys.MoDataHR.Dir = false;
  sys.MoDataHR.Enable = sys.giveExpander(0,3);


  timer = millis()/1000;
  PDS1.update();
  param1 = PDS1.getFastParam(0);
  if(PDS1.fastParamsAreFresh()){
    Serial.println(param1);
  }
  else{
    Serial.println("no data");
    //param1 = 0;
  }
}