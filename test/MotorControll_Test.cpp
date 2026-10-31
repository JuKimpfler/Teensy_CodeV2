#include <Arduino.h>
#include <PDS.h>
#include <systemlib.h>
#include "Servo.h"

systemlib sys;
Servo RadarServo;
// PowerDebugger PDS;

float param1 = 0;
float param2 = 0;
float Dist = 0;
float angle;
float speed;
float x;
float y;

uint8_t buttons[5] = {0, 0, 0, 0, 0};

const ArrayPoint eck_route0[] = {
    // X    Y   Winkel  Pattern             Max Min  Wartezeit
    {  60,  60,   0,   SPattern_Trapezoid,     100, 50,    0 },
    {  60, 180,   0,   SPattern_Trapezoid,  100, 50,    0 },
    {  120, 180,   0,   SPattern_Trapezoid,  100, 50,   0 },
    {  120, 60,   0,   SPattern_Trapezoid,  100, 50,    0 },
};

const ArrayPoint eck_route45[] = {
    // X    Y   Winkel  Pattern             Max Min  Wartezeit
    {  60,  60,   45,   SPattern_Trapezoid,     100, 50,    0 },
    {  60, 180,   45,   SPattern_Trapezoid,  100, 50,    0 },
    {  120, 180,   45,  SPattern_Trapezoid,  100, 50,   00 },
    {  120, 60,   45,   SPattern_Trapezoid,  100, 50,    00 },
};

void setup()
{
  RadarServo.attach(7);
  RadarServo.write(0);
  delay(1000);
  RadarServo.write(90);
  Serial.begin(115200);

  Serial.println("------------------------------- Hallo -------------------------------");

  // sys.initialize() startet auch PDS (PDS.init()) und bindet die Systemwerte
  // auf die festen Kanaele 0..25 -- Akku, Radar, Feldposition, Motoren und
  // Zykluszeit sind damit ohne weiteres Zutun in der GUI. Siehe
  // lib/systemlib/systemlib.cpp (setupPDS()) und docs/PDS_Belegung.md.
  sys.initialize();

  PDS.log("Setup abgeschlossen");

  sys.calibBNO();
}

void loop()
{
  // Taster aus dem Parameter-Tab der GUI (channel_config.h -> PARAM_SLOW_BOOLS).
  // Frueher lagen die beiden auf Fast-Param 2/3; die 5 Fast-Kanaele sind aber
  // fuer den 100-Hz-Joystick da, und die GUI kann einen Taster ohnehin nur auf
  // einen Slow-Bool legen (siehe qml/ParamsView.qml).
  if (PDS.paramBool(1)) // "BNO_kalibrieren"
  {
    PDS.event("BNO kalibriert");
    sys.calibBNO();
    delay(1000);
  }

  for (int i = 47; i >= 0; i--)
  {
    PDS.Channel((50 + i), sys.line_array[i]);
  }

  sys.readSensors();

  param2 = sys.giveOrbitDir();

  x = PDS.fastParam(1); // "Fahrt_X"  (Pad-Achse, 100 Hz)
  y = PDS.fastParam(0); // "Fahrt_Y"
  angle = atan2(y, x) * RAD_TO_DEG;
  speed = min(100.0f, hypot(x, y)) /2;

  if (PDS.getFastParam(3)>50) // "Zielpunkt_anfahren"
  {
    if (PDS.getFastParam(2)>50){
      sys.setPointArray(eck_route45,4,true);
      sys.startArray();
    }
    else{
      
      sys.setPointArray(eck_route0,4,true);
      sys.startArray();
    }
  }
  else
  {
    sys.resetArray();
    if (PDS.getFastParam(2)>50){
      sys.moveDir(angle, speed, 45);
    }
    else{
      sys.moveDir(angle, speed, 0);
    }
  }

  sys.giveButtonStates(buttons);

  if (buttons[0] == fancy_CLICKED)
  {
    PDS.event("Kompass Calib");
    sys.calibBNO();
  }

  if (sys.giveExpStartSwitch()){
    sys.MoDataHL.Speed=100;
  }

  if (buttons[1] == fancy_LONG_PRESS)
  {
    digitalWrite(9, HIGH);
    delay(1000);
    digitalWrite(9, LOW);
  }

  Serial.print("B1 :");
  Serial.print(buttons[0]);
  Serial.print(" B2 :");
  Serial.println(buttons[1]);


  /*Serial.print("X: ");
  Serial.print(sys.Position.X);
  Serial.print(" Y: ");
  Serial.print(sys.Position.Y);
  Serial.print(" Cert: ");
  Serial.print(sys.Position.Certainty);
  Serial.print (" V: ");
  Serial.print(sys.Radar[RadarVo].Dist);
  Serial.print (" L: ");
  Serial.print(sys.Radar[RadarLi].Dist);
  Serial.print (" R: ");
  Serial.print(sys.Radar[RadarRe].Dist);
  Serial.print (" H: ");
  Serial.print(sys.Radar[RadarHi].Dist);
  Serial.print (" VA: ");
  Serial.print(sys.Radar.giveAge(RadarVo));
  Serial.print (" LA: ");
  Serial.print(sys.Radar.giveAge(RadarLi));
  Serial.print (" RA: ");
  Serial.print(sys.Radar.giveAge(RadarRe));
  Serial.print (" HA: ");
  Serial.print(sys.Radar.giveAge(RadarHi));
  Serial.println();*/



  // sys.moveDir(angle,speed,0);

  // sys.setOmniWheelVek(x,y,0);
  // sys.setHeading();
  // param1 = sys.giveOrbitDir()*-0.9;

  /*sys.setMotorData(sys.MoDataVL.ID,param1,true,false);
  sys.setMotorData(sys.MoDataVR.ID,param1,true,false);
  sys.setMotorData(sys.MoDataHL.ID,param1,true,false);
  sys.setMotorData(sys.MoDataHR.ID,param1,true,false);*/

  // sys.MoDataVR.Speed= 90;
  // sys.MoDataVR.Dir= true;
  // sys.MoDataVR.Enable = true;

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

  // PDS.update() steckt in sys.readSensors() und laeuft damit genau einmal
  // pro Durchlauf -- hier ist nichts mehr zu tun. Akku, Timer und Position
  // holt sich die Bibliothek selbst von den gebundenen Zeigern.
}