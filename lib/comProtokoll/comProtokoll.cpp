// Teensy = MASTER
#include "comProtokoll.h"

#define LINK_SERIAL Serial6

static uint8_t rxBuf[512];

static SerialSync serialLink(LINK_SERIAL, /*isMaster=*/true);

void DataSync::begin() {
  LINK_SERIAL.addMemoryForRead(rxBuf, sizeof(rxBuf));   // für hohe baudraten
  LINK_SERIAL.begin(4000000);

  serialLink.addFloat(1, PositionX,   SYNC_RECEIVE);
  serialLink.addFloat(2, PositionY,   SYNC_RECEIVE);
  serialLink.addFloat(3, Position_Cert,   SYNC_RECEIVE);
  serialLink.addBool (4, Position_XVal,   SYNC_RECEIVE);
  serialLink.addBool (5, Position_YVal,   SYNC_RECEIVE);
  serialLink.addBool (6, Position_AtGoal,  SYNC_RECEIVE);
  serialLink.addInt  (7, BNO_Heading,   SYNC_RECEIVE);
  serialLink.addBool (8, BNO_Calib,  SYNC_SEND);
  serialLink.addFloat (9, Motor_SpeedVL,  SYNC_SEND);
  serialLink.addFloat (10, Motor_SpeedVR,  SYNC_SEND);
  serialLink.addFloat (11, Motor_SpeedHL,  SYNC_SEND);
  serialLink.addFloat (12, Motor_SpeedHR,  SYNC_SEND);
  serialLink.begin();
}

bool DataSync::isConnected() { return serialLink.connected(); }

void DataSync::update() {
  serialLink.update();
  //Serial.println(serialLink.connected());
}