// Teensy = MASTER
#include "comProtokoll.h"

#define LINK_SERIAL Serial6

static SerialSync serialLink(LINK_SERIAL, /*isMaster=*/true);

void DataSync::begin() {
  LINK_SERIAL.begin(921600);

  serialLink.addBool (1, test_bool,   SYNC_SEND);
  serialLink.addFloat(2, test_float, SYNC_SEND, 0.01f);
  serialLink.addInt  (3, test_int,  SYNC_SEND);
  serialLink.addBool (4, rc_bool,   SYNC_RECEIVE);
  serialLink.addFloat(5, rc_float,  SYNC_RECEIVE);
  serialLink.addInt (6, rc_int,   SYNC_RECEIVE);
  serialLink.begin();
}

bool DataSync::isConnected() { return serialLink.connected(); }

void DataSync::update() {
  serialLink.update();
  //Serial.println(serialLink.connected());
}