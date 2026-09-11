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

  // Nur noch die Werte, die es allein in diesem Sketch gibt. track() vergibt
  // die Kanalnummer selbst -- ab 32, siehe PDS_AUTO_CHANNEL_BASE in
  // platformio.ini -- und der Name geht automatisch an die GUI.
  PDS.track("Fahrt_Winkel", &angle, "°");
  PDS.track("Fahrt_Speed", &speed, "%");
  PDS.track("Orbit", &param2, "°");

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

  PDS.Channel(12, sys.giveLineAngle(), "LineAngle");
  PDS.Channel(13, sys.giveLineDepth(), "LineDepth");
  PDS.Channel(14, sys.giveLinePresent(), "LinePresent");
  for (int i = 47; i >= 0; i--)
  {
    PDS.Channel((50 + i), sys.line_array[i]);
  }

  sys.readSensors();

  param2 = sys.giveOrbitDir();

  x = PDS.fastParam(1); // "Fahrt_X"  (Pad-Achse, 100 Hz)
  y = PDS.fastParam(0); // "Fahrt_Y"
  angle = atan2(y, x) * RAD_TO_DEG;
  speed = min(100.0f, hypot(x, y)) / 2;

  if (PDS.paramBool(0)) // "Zielpunkt_anfahren"
  {
    sys.moveToPoint(50, 50, 50, 0);
  }
  else
  {
    sys.moveDir(angle, speed, 0);
  }

  sys.giveButtonStates(buttons);

  if (buttons[0] == fancy_CLICKED)
  {
    PDS.event("Kompass Calib");
    sys.calibBNO();
  }

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