#pragma once
#include <Arduino.h>

class CamC {
public:
  void init(HardwareSerial& ser = Serial2, uint32_t baud = 115200); // Serial2 = CAM1 (Teensy 7/8)
  // headingDeg: aktuelles BNO-Heading (Feld-KS), noetig um die kamerarelative
  // Peilung in eine absolute Feld-Peilung umzurechnen.
  void Update(float headingDeg);

  // Ergebnis: Tor-Bearing im Feld-KS (Grad, -180..180)
  float give_Angle() const { return angle_field_deg; }

  // Debug/Zusatz
  bool isValid(uint32_t maxAgeMs = 200) const;
  float give_Angle_Cam() const { return angle_cam_deg; }  // relativ zur Kamera
  int   give_BlobH() const { return blob_h_px; }

  // falls Vorzeichen nicht passt, hier umstellen:
  void setSign(bool usePlus) { usePlusSign = usePlus; } // true: field = cam + imu, false: field = imu - cam

private:
  void readOpenMV();
  
  HardwareSerial* port = nullptr;

  static constexpr uint16_t LINE_BUF_SIZE = 96;
  char lineBuf[LINE_BUF_SIZE];
  uint16_t lineLen = 0;

  bool rawValid = false;
  uint32_t lastCamUpdateMs = 0;

  char goal = 'N';
  float angle_cam_deg = 0.0f;
  int blob_h_px = 0;

  float angle_field_deg = 0.0f;

  bool usePlusSign = true; // Standard: field = cam + imu

  void handleLine(const char* line);
  static float wrap180(float a);
};