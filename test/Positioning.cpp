#include <Arduino.h>

#include "pmw3389.h"

#include "Cam.h"

#include "SRF08.h"

PMW3389 pmw = PMW3389(&SPI, 27); // SPI CS vom Moussensor kommt auf LDR Port

SRF08Sensor SRF08_vo(SRF08_ADDR(0xE2), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor SRF08_re(SRF08_ADDR(0xE4), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor SRF08_hi(SRF08_ADDR(0xE6), SRF08_RANGE_2M, SRF08_GAIN_MID);
SRF08Sensor SRF08_li(SRF08_ADDR(0xE8), SRF08_RANGE_2M, SRF08_GAIN_MID);

CamC cam1;
CamC cam2;

void setup(){
  Wire1.begin();
  Serial.begin(115200);
  SPI.begin();
  pmw.begin();

  cam1.init(Serial6,115200);
  cam2.init(Serial7,115200);

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

  PMW3389_DATA mouseData = pmw.readBurst(); // Daten des Odometry sensors.

  SRF08_vo.update();
  SRF08_vo.getDistance();

  SRF08_hi.update();
  SRF08_hi.getDistance();

  SRF08_re.update();
  SRF08_re.getDistance();

  SRF08_li.update();
  SRF08_li.getDistance();

}