#include <Arduino.h>
#include "pmw3389.h"
#include "Cam.h"
#include "SRF08.h"
#include <BNO055.h>
#include <Utils.h>

BNO055 bno = BNO055(0x28, &Wire1);

PMW3389 pmw = PMW3389(&SPI, 27); // SPI CS vom Moussensor kommt auf LDR Port

SRF08Sensor SRF08_vo(SRF08_ADDR(0xE2), SRF08_RANGE_2M, SRF08_GAIN_MID); // US nach vorne
SRF08Sensor SRF08_re(SRF08_ADDR(0xE4), SRF08_RANGE_2M, SRF08_GAIN_MID); // US nach rechts
SRF08Sensor SRF08_hi(SRF08_ADDR(0xE6), SRF08_RANGE_2M, SRF08_GAIN_MID); // US nach hinten
SRF08Sensor SRF08_li(SRF08_ADDR(0xE8), SRF08_RANGE_2M, SRF08_GAIN_MID); // US nach links

CamC cam1; // Camera nach vorne
CamC cam2; // Camera nach hinten

PMW3389_DATA mouseData;

float direction = 0.0f; // Richtung des Roboters in Grad, 0 = nach vorne, +90 = nach rechts, -90 = nach links, 180/-180 = nach hinten

void setup()
{
  Wire1.begin();
  Serial.begin(115200);
  SPI.begin();
  pmw.begin();
  bno.init();

  cam1.init(Serial6, 115200);
  cam2.init(Serial7, 115200);

  // -- US -----
  SRF08_vo.setEMAAlpha(0.25f);
  SRF08_hi.setEMAAlpha(0.25f);
  SRF08_re.setEMAAlpha(0.25f);
  SRF08_li.setEMAAlpha(0.25f);

  SRF08_vo.setJumpThreshold(50);
  SRF08_hi.setJumpThreshold(50);
  SRF08_re.setJumpThreshold(50);
  SRF08_li.setJumpThreshold(50);

  SRF08_vo.begin(Wire1);
  SRF08_hi.begin(Wire1);
  SRF08_re.begin(Wire1);
  SRF08_li.begin(Wire1);

  SRF08_vo.startRanging();
  SRF08_hi.startRanging();
  SRF08_re.startRanging();
  SRF08_li.startRanging();
}

void loop()
{
  cam1.Update(); // vordere Cam um bekannten Blopp1 auf feld zu erkennen.
  cam2.Update(); // hintere Cam um bekannten Blopp2 auf feld zu erkennen.

  mouseData = pmw.readBurst();

  if (SRF08_vo.update())
    SRF08_vo.getDistance(), SRF08_vo.startRanging();

  if (SRF08_hi.update())
    SRF08_hi.getDistance(), SRF08_hi.startRanging();

  if (SRF08_re.update())
    SRF08_re.getDistance(), SRF08_re.startRanging();

  if (SRF08_li.update())
    SRF08_li.getDistance(), SRF08_li.startRanging();

  direction = Util.AMap(bno.eulHeading());


  // read US via SRF08.getDistance()
  // read Cam via cam1.give_Angle() and cam2.give_Angle()
  // read Mouse via mouseData.dx and mouseData.dy
  // read BNO via direction
  
}