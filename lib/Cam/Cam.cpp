#include "Cam.h"
#include "BNO055.h"
#include <math.h>

static float wrap180f(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

void CamC::init(HardwareSerial& ser, uint32_t baud) {
  port = &ser;
  port->begin(baud);
  lineLen = 0;
  rawValid = false;
  lastCamUpdateMs = 0;
}

bool CamC::isValid(uint32_t maxAgeMs) const {
  if (!rawValid) return false;
  return (millis() - lastCamUpdateMs) <= maxAgeMs;
}

void CamC::readOpenMV() {
  if (!port) return;

  while (port->available()) {
    char c = (char)port->read();

    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      if (lineLen > 0 && lineBuf[lineLen - 1] == '\r') lineBuf[lineLen - 1] = '\0';
      handleLine(lineBuf);
      lineLen = 0;
      continue;
    }

    if (lineLen < (LINE_BUF_SIZE - 1)) {
      lineBuf[lineLen++] = c;
    } else {
      lineLen = 0; // overflow -> drop
    }
  }
}

void CamC::handleLine(const char* line) {
  if (!line || !line[0]) return;

  if (line[0] == 'N') {
    rawValid = false;
    return;
  }

  // Erwartet: A,<goal>,<angle_cam_deg>,<blob_h>
  // Beispiel: A,B,-12.34,37
  if (line[0] == 'A' && line[1] == ',') {
    // einfache CSV parse ohne strtok allocation
    // Wir kopieren trotzdem in tmp für strtok_r (robust und schnell genug bei 20-60Hz)
    char tmp[LINE_BUF_SIZE];
    strncpy(tmp, line, LINE_BUF_SIZE - 1);
    tmp[LINE_BUF_SIZE - 1] = '\0';

    char* save = nullptr;
    char* tok = strtok_r(tmp, ",", &save);

    int idx = 0;
    char g = 'N';
    float ang = 0.0f;
    int h = 0;

    while (tok) {
      if (idx == 1) g = tok[0];
      else if (idx == 2) ang = strtof(tok, nullptr);
      else if (idx == 3) h = atoi(tok);
      tok = strtok_r(nullptr, ",", &save);
      idx++;
    }

    if (idx >= 4) {
      goal = g;
      angle_cam_deg = ang;
      blob_h_px = h;
      rawValid = true;
      lastCamUpdateMs = millis();
    }
  }
}

void CamC::Update(float headingDeg) {
  // UART pollen
  readOpenMV();

  if (!isValid(250)) return;

  // Kombination:
  // Standard: angle_field = imu + angle_cam
  // Falls bei dir das Vorzeichen vertauscht ist -> setSign(false)
  if (usePlusSign) angle_field_deg = wrap180f(headingDeg + angle_cam_deg);
  else             angle_field_deg = wrap180f(headingDeg - angle_cam_deg);

  // Ziel: Wenn Roboter auf der Stelle dreht, bleibt angle_field_deg möglichst konstant.
  // Das stimmt dann, wenn imu_deg und angle_cam_deg konsistente Vorzeichen haben.
}