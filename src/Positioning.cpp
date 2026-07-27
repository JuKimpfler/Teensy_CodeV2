#include <Arduino.h>
#include <Servo.h>
#include "pmw3389.h"
#include "Cam.h"
#include "SRF08.h"
#include <BNO055.h>
#include <Utils.h>
#include "PositionEstimator.h"
#include "enum.h"
#include <PDS.h>

BNO055 bno = BNO055(0x28, &Wire1);

PowerDebugger PDS;

// PDS-Kanäle für die Positionsschätzung
static constexpr uint8_t DBG_CH_POS_GRID_X   = 102;
static constexpr uint8_t DBG_CH_POS_GRID_Y   = 103;
static constexpr uint8_t DBG_CH_POS_CONF     = 104;
static constexpr uint8_t DBG_CH_POS_US_VALID = 105;
static constexpr uint8_t DBG_CH_POS_CAM_VALID = 106;
static constexpr uint8_t DBG_CH_POS_X_CM      = 107;
static constexpr uint8_t DBG_CH_POS_Y_CM      = 108;

PMW3389 pmw = PMW3389(&SPI, 27); // SPI CS vom Moussensor kommt auf LDR Port

SRF08Sensor SRF08_vo(SRF08_ADDR(0xE2), SRF08_RANGE_2M, SRF08_GAIN_MID); // US nach vorne
SRF08Sensor SRF08_re(SRF08_ADDR(0xE4), SRF08_RANGE_2M, SRF08_GAIN_MID); // US nach rechts
SRF08Sensor SRF08_hi(SRF08_ADDR(0xE6), SRF08_RANGE_2M, SRF08_GAIN_MID); // US nach hinten
SRF08Sensor SRF08_li(SRF08_ADDR(0xE8), SRF08_RANGE_2M, SRF08_GAIN_MID); // US nach links

CamC cam1; // Camera nach vorne
CamC cam2; // Camera nach hinten

PMW3389_DATA mouseData;

// Servo, der den US-Sensor-Ring gegen die Roboterdrehung kontert, damit die
// Sensoren trotz Roboterdrehung feldparallel (senkrecht zur Wand) bleiben.
Servo usRingServo;
static constexpr uint8_t US_SERVO_PIN = EMPTY_27;

float direction = 0.0f; // Richtung des Roboters in Grad, 0 = nach vorne, +90 = nach rechts, -90 = nach links, 180/-180 = nach hinten

uint32_t lastUpdateUs = 0;

Positioning::MountConfig posCfg; // Werte bei Bedarf an eure Mechanik anpassen (Ringradius, Toleranzen, Feldmasse, ...)

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

  // -- US-Ring-Servo -----
  usRingServo.attach(US_SERVO_PIN);

  // -- Positionsschaetzung -----
  Positioning::begin(posCfg);

  // -- PDS -----
  PDS.init();

  lastUpdateUs = micros();
}

void loop()
{
  direction = Util.AMap(bno.eulHeading());

  cam1.Update(direction); // vordere Cam um bekannten Blopp1 auf feld zu erkennen.
  cam2.Update(direction); // hintere Cam um bekannten Blopp2 auf feld zu erkennen.

  mouseData = pmw.readBurst();

  uint16_t d_vo = 0, d_re = 0, d_hi = 0, d_li = 0;
  if (SRF08_vo.update()) d_vo = SRF08_vo.getDistance(), SRF08_vo.startRanging();
  if (SRF08_hi.update()) d_hi = SRF08_hi.getDistance(), SRF08_hi.startRanging();
  if (SRF08_re.update()) d_re = SRF08_re.getDistance(), SRF08_re.startRanging();
  if (SRF08_li.update()) d_li = SRF08_li.getDistance(), SRF08_li.startRanging();

  // US-Ring-Servo entsprechend dem BNO-Winkel nachfuehren, damit die
  // Sensoren senkrecht zur naechsten Wand ausgerichtet bleiben.
  float servoAngle = Positioning::computeServoAngle(direction, posCfg);
  usRingServo.write((int)lroundf(servoAngle));

  Positioning::USReading us[Positioning::US_COUNT] = {
      {(float)d_vo, SRF08_vo.hasValidData()}, // US_FRONT
      {(float)d_re, SRF08_re.hasValidData()}, // US_RIGHT
      {(float)d_hi, SRF08_hi.hasValidData()}, // US_BACK
      {(float)d_li, SRF08_li.hasValidData()}, // US_LEFT
  };

  Positioning::CamReading camFront{cam1.isValid(), cam1.give_Angle(), cam1.give_BlobH()};
  Positioning::CamReading camBack{cam2.isValid(), cam2.give_Angle(), cam2.give_BlobH()};

  Positioning::MouseReading mouse{mouseData.dx, mouseData.dy, mouseData.isOnSurface, 800};

  uint32_t nowUs = micros();
  float dt_s = (nowUs - lastUpdateUs) * 1e-6f;
  lastUpdateUs = nowUs;

  Positioning::Pose pose = Positioning::update(dt_s, direction, camFront, camBack, us, mouse);

  // pose.x_cm / pose.y_cm  : Position im Feld-KS (cm, unrund)
  // pose.gridX / pose.gridY: Position auf 2cm-Raster (fuer Taktik/Odometrie)
  // pose.confidence        : grobe Guete der Schaetzung (0..1)
  PDS.Channel(DBG_CH_POS_GRID_X, pose.gridX);
  PDS.Channel(DBG_CH_POS_GRID_Y, pose.gridY);
  PDS.Channel(DBG_CH_POS_CONF, pose.confidence);
  PDS.Channel(DBG_CH_POS_US_VALID, pose.usValidCount);
  PDS.Channel(DBG_CH_POS_CAM_VALID, pose.camValid ? 1.0f : 0.0f);
  PDS.Channel(DBG_CH_POS_X_CM, pose.x_cm);
  PDS.Channel(DBG_CH_POS_Y_CM, pose.y_cm);

  PDS.update();
}
