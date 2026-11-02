// Teensy = MASTER
#include "comProtokoll.h"

#define LINK_SERIAL Serial6

// 1 = jede Sekunde den Link-Status ueber USB-Serial ausgeben
#define LINK_DEBUG 0

static uint8_t rxBuf[512];

static SerialSync serialLink(LINK_SERIAL, /*isMaster=*/true);

// Serial6 = LPUART1 (Teensy 4.x). Der Teensy-Core loescht die Fehlerflags nie.
// Ist OR (Overrun) einmal gesetzt, nimmt der Empfaenger laut Reference Manual keine
// Daten mehr an, bis das Flag geloescht wird (-> bleibt bis zum naechsten begin() tot).
#define LINK_LPUART IMXRT_LPUART1
static const uint32_t UART_ERR_FLAGS = LPUART_STAT_OR | LPUART_STAT_NF | LPUART_STAT_FE | LPUART_STAT_PF;
static uint32_t uartErrCount = 0, uartOrCount = 0;

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
  serialLink.addFloat (9, Motor_SpeedVL,  SYNC_SEND, 0.5f);   // Schwelle in rpm: weniger Traffic
  serialLink.addFloat (10, Motor_SpeedVR,  SYNC_SEND, 0.5f);
  serialLink.addFloat (11, Motor_SpeedHL,  SYNC_SEND, 0.5f);
  serialLink.addFloat (12, Motor_SpeedHR,  SYNC_SEND, 0.5f);
  serialLink.begin();
}

bool DataSync::isConnected() { return serialLink.connected(); }
uint32_t DataSync::rxBytes()      { return serialLink.rxBytes(); }
uint32_t DataSync::rxOk()         { return serialLink.rxOk(); }
uint32_t DataSync::rxBad()        { return serialLink.rxBad(); }
uint32_t DataSync::uartErrors()   { return uartErrCount; }
uint32_t DataSync::uartOverruns() { return uartOrCount; }

void DataSync::update() {
  // Fehlerflags loeschen, sonst kann ein einziger Overrun den Empfang dauerhaft sperren
  const uint32_t st = LINK_LPUART.STAT;
  if (st & UART_ERR_FLAGS) {
    uartErrCount++;
    if (st & LPUART_STAT_OR) uartOrCount++;
    LINK_LPUART.STAT |= UART_ERR_FLAGS;   // write-1-to-clear (gleiche Schreibweise wie der Core bei IDLE)
  }
  serialLink.update();

  #if LINK_DEBUG
  static uint32_t dbgT = 0;
  if (millis() - dbgT >= 1000) {
    dbgT = millis();
    Serial.printf("[Teensy] conn=%d bytes=%lu ok=%lu bad=%lu uartErr=%lu OR=%lu\n",
                  serialLink.connected(),
                  (unsigned long)serialLink.rxBytes(), (unsigned long)serialLink.rxOk(), (unsigned long)serialLink.rxBad(),
                  (unsigned long)uartErrCount, (unsigned long)uartOrCount);
  }
  #endif
}