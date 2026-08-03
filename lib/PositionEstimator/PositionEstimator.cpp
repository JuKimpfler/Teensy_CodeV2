#include "PositionEstimator.h"
#include <math.h>

namespace Positioning {

namespace {

MountConfig s_cfg;
Pose        s_pose;
bool        s_initialized = false;

// Montagewinkel der 4 US-Sensoren auf dem Ring, relativ zur Roboter-Front
constexpr float kMountAngleDeg[US_COUNT] = {0.0f, 90.0f, 180.0f, 270.0f};
// (front,back) und (right,left) liegen sich je 180° gegenüber
constexpr uint8_t kOppositeOf[US_COUNT] = {US_BACK, US_LEFT, US_FRONT, US_RIGHT};

inline float lerp(float a, float b, float t) { return a + (b - a) * t; }

inline void dirFromCompass(float deg, float& outX, float& outY) {
  float rad = deg * DEG_TO_RAD;
  outX = sinf(rad);
  outY = cosf(rad);
}

// Rest der Roboterdrehung, den der Servo ausgleichen muss (Ring ist
// 90°-symmetrisch -> nur der Anteil modulo 90° zählt, dadurch minimale
// Servobewegung und kein "Aufwickeln" von Kabeln bei mehreren Umdrehungen).
float ringCorrectionDeg(float headingDeg) {
  if (!s_cfg.hasUSServo) return 0.0f;
  float m = fmodf(headingDeg, 90.0f);
  if (m < 0.0f) m += 90.0f;
  if (m > 45.0f) m -= 90.0f; // -> Bereich (-45, 45]
  if (s_cfg.servoInverted) m = -m;
  return m;
}

// Entfernung von Punkt (px,py) zur nächsten Feld-Wand entlang der
// Kompass-Richtung dirDeg (analytische Ray-AABB-Schnittberechnung).
float rayExitDistance(float px, float py, float dirDeg) {
  float halfW = s_cfg.fieldWidth_cm * 0.5f;
  float halfL = s_cfg.fieldLength_cm * 0.5f;

  float dx, dy;
  dirFromCompass(dirDeg, dx, dy);

  float tx = INFINITY;
  if (dx > 1e-5f) tx = (halfW - px) / dx;
  else if (dx < -1e-5f) tx = (-halfW - px) / dx;

  float ty = INFINITY;
  if (dy > 1e-5f) ty = (halfL - py) / dy;
  else if (dy < -1e-5f) ty = (-halfL - py) / dy;

  float t = min(tx, ty);
  return (t > 0.0f) ? t : 0.0f;
}

// Schnittpunkt zweier Tor-Peilungen (Kamera vorne/hinten) -> grobe Position.
// Rückgabe false, wenn eine Kamera ungültig ist oder die Peilungen zu
// parallel sind, um einen stabilen Schnittpunkt zu bilden.
bool triangulate(const CamReading& front, const CamReading& back, float& outX, float& outY, float& outWeight) {
  if (!front.valid || !back.valid) return false;
  if (front.blobHeight_px < s_cfg.camMinBlobHeight_px || back.blobHeight_px < s_cfg.camMinBlobHeight_px) return false;

  float halfL = s_cfg.fieldLength_cm * 0.5f;
  const float P1x = 0.0f, P1y = halfL;  // Tor, das die vordere Kamera sieht
  const float P2x = 0.0f, P2y = -halfL; // Tor, das die hintere Kamera sieht

  float d1x, d1y, d2x, d2y;
  dirFromCompass(front.angle_field_deg, d1x, d1y);
  dirFromCompass(back.angle_field_deg, d2x, d2y);

  float denom = d1x * d2y - d1y * d2x;
  if (fabsf(denom) < 0.05f) return false; // Peilungen (fast) parallel -> Schnitt instabil

  float t = ((P2x - P1x) * d2y - (P2y - P1y) * d2x) / denom;
  outX = P1x + t * d1x;
  outY = P1y + t * d1y;
  outWeight = constrain(fabsf(denom), 0.0f, 1.0f);
  return true;
}

int16_t snapToGrid(float v) {
  return (int16_t)lroundf(roundf(v / POS_GRID_CM) * POS_GRID_CM);
}

} // namespace

void begin(const MountConfig& cfg) {
  s_cfg = cfg;
  s_pose = Pose();
  s_initialized = false;
}

void reset(float x_cm, float y_cm) {
  s_pose = Pose();
  s_pose.x_cm = x_cm;
  s_pose.y_cm = y_cm;
  s_pose.gridX = snapToGrid(x_cm);
  s_pose.gridY = snapToGrid(y_cm);
  s_initialized = true;
}

float computeServoAngle(float headingDeg, const MountConfig& cfg) {
  if (!cfg.hasUSServo) return cfg.servoNeutralDeg;
  float m = fmodf(headingDeg, 90.0f);
  if (m < 0.0f) m += 90.0f;
  if (m > 45.0f) m -= 90.0f;
  if (cfg.servoInverted) m = -m;
  float out = cfg.servoNeutralDeg - m;
  return constrain(out, cfg.servoNeutralDeg - cfg.servoRangeDeg, cfg.servoNeutralDeg + cfg.servoRangeDeg);
}

Pose update(float dt_s,
            float headingDeg,
            const CamReading& camFront,
            const CamReading& camBack,
            const USReading us[US_COUNT],
            const MouseReading& mouse) {
  const float maxStep_cm = MAX_SPEED_CMS * max(dt_s, 0.0f) * 1.5f; // 1.5x Marge ggü. Topspeed

  // ── 1. Kamera-Triangulation (grobe Position) ──────────────────────────
  float camX = 0.0f, camY = 0.0f, camWeight = 0.0f;
  bool camValid = triangulate(camFront, camBack, camX, camY, camWeight);

  // ── 2. Bootstrap, falls noch keine Position bekannt ist ───────────────
  if (!s_initialized) {
    if (camValid) {
      s_pose.x_cm = camX;
      s_pose.y_cm = camY;
      s_initialized = true;
    } else {
      // Keine Kamera-Fix vorhanden: bei (0,0) starten (Feldmitte) bis zur
      // ersten gültigen Triangulation. Für einen bekannten Kickoff-Punkt
      // stattdessen reset(x,y) in setup() aufrufen.
      Pose p;
      p.confidence = 0.0f;
      s_pose = p;
      return s_pose;
    }
  }

  // ── 3. Maus-Odometrie: Delta ins Feld-KS drehen ───────────────────────
  float predX = s_pose.x_cm;
  float predY = s_pose.y_cm;

  if (mouse.onSurface && mouse.cpi > 0) {
    float dx_cm = (float)mouse.dx / (float)mouse.cpi * 2.54f;
    float dy_cm = (float)mouse.dy / (float)mouse.cpi * 2.54f;

    float mountDeg = headingDeg + s_cfg.mouseMountAngleDeg;
    float fwdX, fwdY, rightX, rightY;
    dirFromCompass(mountDeg, fwdX, fwdY);
    dirFromCompass(mountDeg + 90.0f, rightX, rightY);

    float deltaX = dx_cm * rightX + dy_cm * fwdX;
    float deltaY = dx_cm * rightY + dy_cm * fwdY;

    float stepLen = sqrtf(deltaX * deltaX + deltaY * deltaY);
    if (stepLen > maxStep_cm && stepLen > 1e-4f) {
      // Unplausibler Sprung (z.B. Sensor kurz abgehoben) -> auf Topspeed begrenzen
      float scale = maxStep_cm / stepLen;
      deltaX *= scale;
      deltaY *= scale;
    }

    predX += deltaX;
    predY += deltaY;
  }

  // ── 4. Referenzposition: Odometrie-Vorhersage mit Kamera geblendet ────
  float refX = predX;
  float refY = predY;
  if (camValid) {
    float blend = constrain(s_cfg.camBlend * (0.5f + 0.5f * camWeight), 0.0f, 1.0f);
    refX = lerp(predX, camX, blend);
    refY = lerp(predY, camY, blend);
  }

  // ── 5. US-Sensoren: Erwartungswert je Sensor & Validierung ────────────
  float ringCorr = ringCorrectionDeg(headingDeg);

  float facingDeg[US_COUNT];
  float expected[US_COUNT];
  bool  campassed[US_COUNT];
  bool  present[US_COUNT];

  for (uint8_t i = 0; i < US_COUNT; i++) {
    facingDeg[i] = headingDeg + kMountAngleDeg[i] - ringCorr;

    float dx, dy;
    dirFromCompass(facingDeg[i], dx, dy);
    float sensorX = refX + s_cfg.usRingRadius_cm * dx;
    float sensorY = refY + s_cfg.usRingRadius_cm * dy;
    expected[i] = rayExitDistance(sensorX, sensorY, facingDeg[i]);

    present[i] = us[i].valid && us[i].distance_cm > 0.0f;
    campassed[i] = !camValid || !present[i] ||
                   fabsf(us[i].distance_cm - expected[i]) <= s_cfg.usCamAgreement_cm;
  }

  // ── 6. Konsistenz-Check gegenüberliegender Sensorpaare ────────────────
  // Erkennt verdeckte/fehlerhafte Sensoren auch ohne Kamera-Fix: die Summe
  // der beiden gegenüberliegenden Distanzen (+ 2x Ringradius) muss der
  // Sehnenlänge des Feldes entlang dieser Linie entsprechen.
  bool pairpassed[US_COUNT] = {true, true, true, true};
  for (uint8_t i = 0; i < US_COUNT; i++) {
    uint8_t j = kOppositeOf[i];
    if (j <= i) continue; // jedes Paar nur einmal prüfen
    if (!present[i] || !present[j]) continue;

    float chord = rayExitDistance(refX, refY, facingDeg[i]) +
                  rayExitDistance(refX, refY, facingDeg[i] + 180.0f);
    float measuredSum = us[i].distance_cm + us[j].distance_cm + 2.0f * s_cfg.usRingRadius_cm;

    bool ok = fabsf(measuredSum - chord) <= s_cfg.usPairSum_cm;
    pairpassed[i] = ok;
    pairpassed[j] = ok;
  }

  // ── 7. US-Korrektur akkumulieren ───────────────────────────────────────
  float corrX = 0.0f, corrY = 0.0f;
  uint8_t usValidCount = 0;
  bool sensorValid[US_COUNT];

  for (uint8_t i = 0; i < US_COUNT; i++) {
    sensorValid[i] = present[i] && campassed[i] && pairpassed[i];
    if (!sensorValid[i]) continue;

    float dx, dy;
    dirFromCompass(facingDeg[i], dx, dy);
    float residual = us[i].distance_cm - expected[i];
    corrX += dx * residual;
    corrY += dy * residual;
    usValidCount++;
  }

  if (usValidCount > 0) {
    corrX = corrX / usValidCount * s_cfg.usCorrectionGain;
    corrY = corrY / usValidCount * s_cfg.usCorrectionGain;
  }

  // Wenn beide Sensoren einer Achse (z.B. vorne+hinten) ausfallen, bleibt
  // corrX/corrY für diese Richtung 0 -> die Referenzposition (Maus-Odometrie
  // + Kamera) bestimmt die Feinposition für diese Achse alleine. Das ist
  // genau der geforderte Fallback "Maus-Sensor wenn zwei US-Sensoren Quatsch
  // liefern".

  float finalX = refX + corrX;
  float finalY = refY + corrY;

  // ── 8. Plausibilitätsgrenze: max. 5 m/s seit letzter Schätzung ────────
  float movedX = finalX - s_pose.x_cm;
  float movedY = finalY - s_pose.y_cm;
  float moved = sqrtf(movedX * movedX + movedY * movedY);
  if (moved > maxStep_cm && moved > 1e-4f) {
    float scale = maxStep_cm / moved;
    finalX = s_pose.x_cm + movedX * scale;
    finalY = s_pose.y_cm + movedY * scale;
  }

  // ── 9. Feldgrenzen (+ kleine Toleranz für Robotergröße) ───────────────
  float halfW = s_cfg.fieldWidth_cm * 0.5f + 15.0f;
  float halfL = s_cfg.fieldLength_cm * 0.5f + 15.0f;
  finalX = constrain(finalX, -halfW, halfW);
  finalY = constrain(finalY, -halfL, halfL);

  // ── 10. Ergebnis zusammenstellen ──────────────────────────────────────
  Pose out;
  out.x_cm = finalX;
  out.y_cm = finalY;
  out.gridX = snapToGrid(finalX);
  out.gridY = snapToGrid(finalY);
  out.usValidCount = usValidCount;
  out.camValid = camValid;
  out.confidence = constrain((camValid ? 0.5f : 0.0f) + (usValidCount / (float)US_COUNT) * 0.5f, 0.0f, 1.0f);

  s_pose = out;
  return s_pose;
}

Pose getPose() { return s_pose; }

} // namespace Positioning
