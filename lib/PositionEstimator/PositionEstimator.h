#pragma once
#include <Arduino.h>

// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  PositionEstimator  —  Sensorfusion für die Roboter-Position (x,y)         ║
// ║  RoboCup Junior Soccer 2v2 (Infrared League)                              ║
// ╠══════════════════════════════════════════════════════════════════════════╣
// ║  Kombiniert:                                                              ║
// ║    - Kamera(s)   : grobe Position via Triangulation zweier Tor-Peilungen  ║
// ║    - US-Sensoren : Feinkorrektur via Abstand zu den Feldwänden            ║
// ║                    (validiert gegen die Kamera-Position, siehe unten)     ║
// ║    - Maus-Sensor : Odometrie/Feinbestimmung, wenn die US-Werte durch      ║
// ║                    verdeckte Sensoren (anderer Roboter im Weg) unbrauchbar║
// ║                    sind                                                  ║
// ║    - BNO055      : Ausrichtung (Heading) für alle obigen Berechnungen und ║
// ║                    zur Nachführung des Servos, der den US-Sensor-Ring     ║
// ║                    unabhängig von der Roboterdrehung feldparallel hält    ║
// ╠══════════════════════════════════════════════════════════════════════════╣
// ║  Koordinatensystem (Feld-KS):                                            ║
// ║    x-Achse : quer zum Feld (Seitenlinie <-> Seitenlinie), 0 = Mitte       ║
// ║    y-Achse : längs zum Feld (Tor <-> Tor),               0 = Mitte       ║
// ║    heading : Kompass-Winkel in Grad, 0° = Roboter-Front zeigt Richtung   ║
// ║              +y, im Uhrzeigersinn steigend (90° = Richtung +x), wie      ║
// ║              von BNO055::eulHeading() / Util.AMap() geliefert.           ║
// ║    Ausgabe : x/y werden zusätzlich auf ein 2cm-Raster gerundet           ║
// ║              (POS_GRID_CM) für die Odometrie-/Taktik-Weiterverarbeitung. ║
// ╚══════════════════════════════════════════════════════════════════════════╝
//
// Erwartete Sensor-Montage (Standardfall, siehe MountConfig zum Anpassen):
//   US-Ring: 4 Sensoren im 90°-Abstand auf einem gemeinsamen, drehbaren
//            Ring montiert (vo=0°, re=90°, hi=180°, li=270° relativ zur
//            Roboter-Front). Ein Servo kann diesen Ring gegen die
//            Roboterdrehung kontern, siehe computeServoAngle().
//   Kamera : cam vorne "sieht" das Tor bei y=+FIELD_LENGTH/2,
//            cam hinten "sieht" das Tor bei y=-FIELD_LENGTH/2
//            (Farbfilter/Auswahl passiert in der OpenMV-Kamera selbst).
//   Maus   : Achsen liegen wie Roboter-Vorwärts/Rechts (dy=vorwärts,
//            dx=rechts), ggf. über MountConfig.mouseMountAngleDeg drehen.

namespace Positioning {

// ── Feldgeometrie [cm] ────────────────────────────────────────────────────
// Default: RoboCup Junior Soccer "Open" Feld. Bei "Lightweight" anpassen
// (siehe begin()/MountConfig zum Überschreiben zur Laufzeit).
static constexpr float FIELD_LENGTH_CM = 243.0f; // y: Tor zu Tor
static constexpr float FIELD_WIDTH_CM  = 182.0f; // x: Seitenlinie zu Seitenlinie

static constexpr float POS_GRID_CM = 2.0f; // Ausgabe-Raster

static constexpr float MAX_SPEED_CMS = 500.0f; // 5 m/s Top-Speed -> Plausibilitätsgrenze

// ── Ergebnis ──────────────────────────────────────────────────────────────
struct Pose {
  float x_cm = 0.0f; // ungerundete, interne Position (Feld-KS)
  float y_cm = 0.0f;
  int16_t gridX = 0; // x_cm gerundet auf POS_GRID_CM Raster
  int16_t gridY = 0; // y_cm gerundet auf POS_GRID_CM Raster
  float confidence = 0.0f; // 0..1, grobe Güte der Schätzung
  uint8_t usValidCount = 0; // Anzahl aktuell als plausibel bewerteter US-Sensoren
  bool camValid = false;    // Kamera-Triangulation in diesem Zyklus verwendbar
};

// ── Rohwert-Container (vom Aufrufer pro Zyklus befüllt) ──────────────────
struct USReading {
  float distance_cm = -1.0f; // < 0 => keine Messung vorhanden
  bool valid = false;        // Sensor meldet eine plausible Rohmessung (z.B. hasValidData())
};

struct CamReading {
  bool valid = false;
  float angle_field_deg = 0.0f; // absolute Peilung Roboter->Tor im Feld-KS (siehe Cam::give_Angle())
  int blobHeight_px = 0;         // Blob-Höhe, je größer desto näher/verlässlicher
};

struct MouseReading {
  int16_t dx = 0; // Counts seit letztem Zyklus, +x = nach rechts (Robotersicht)
  int16_t dy = 0; // Counts seit letztem Zyklus, +y = vorwärts (Robotersicht)
  bool onSurface = true;
  uint16_t cpi = 800; // aktuell eingestellte Counts-per-Inch des Sensors
};

// Reihenfolge entspricht der Ringmontage (90°-Schritte)
enum USIndex : uint8_t { US_FRONT = 0, US_RIGHT = 1, US_BACK = 2, US_LEFT = 3, US_COUNT = 4 };

// ── Montage-/Toleranz-Konfiguration ───────────────────────────────────────
struct MountConfig {
  float usRingRadius_cm = 9.0f; // Abstand Mittelpunkt US-Ring <-> Roboterzentrum

  bool hasUSServo = true; // true: US-Ring wird per Servo gegen Roboterdrehung gekontert
  float servoNeutralDeg = 90.0f;
  float servoRangeDeg   = 45.0f; // mechanischer Ausschlag +- um neutral (Ring ist 90°-symmetrisch)
  bool  servoInverted   = false; // falls Drehsinn des Servos der Rechnung entgegengesetzt ist

  float mouseMountAngleDeg = 0.0f; // Offset Maussensor-Achsen ggü. Roboter-Front

  // Validierungstoleranzen
  float usCamAgreement_cm  = 15.0f; // max. Abweichung US<->Kamera-Erwartung, sonst als verdeckt verworfen
  float usPairSum_cm       = 12.0f; // max. Abweichung der Summe gegenüberliegender US-Sensoren
  float usCorrectionGain   = 0.5f;  // Anteil der US-Korrektur, der pro Zyklus übernommen wird (0..1)
  float camBlend           = 0.5f;  // Anteil der Kamera-Position, der in die Referenzposition einfließt (0..1)
  int   camMinBlobHeight_px = 4;    // Blobs kleiner als das gelten als zu unsicher/fern

  float fieldLength_cm = FIELD_LENGTH_CM;
  float fieldWidth_cm  = FIELD_WIDTH_CM;
};

// Muss einmal in setup() aufgerufen werden.
void begin(const MountConfig& cfg = MountConfig());

// Setzt die interne Position hart (z.B. bekannte Kickoff-Position).
void reset(float x_cm, float y_cm);

/**
 * Haupt-Update, einmal pro loop() mit allen Rohwerten aufrufen.
 * @param dt_s          Zykluszeit in Sekunden seit letztem Aufruf
 * @param headingDeg    BNO055-Heading (Feld-KS, siehe Kommentar oben), z.B. Util.AMap(bno.eulHeading())
 * @param camFront/Back Kamera-Peilungen zu den beiden bekannten Toren
 * @param us[FRONT..LEFT] US-Rohmessungen, Reihenfolge wie an Ring montiert (vor Servokorrektur!)
 * @param mouse         Maussensor-Delta seit letztem Aufruf
 */
Pose update(float dt_s,
            float headingDeg,
            const CamReading& camFront,
            const CamReading& camBack,
            const USReading us[US_COUNT],
            const MouseReading& mouse);

Pose getPose();

/**
 * Zielwinkel für den US-Ring-Servo, damit der Ring unabhängig von der
 * Roboterdrehung feldparallel bleibt (nur der Rest modulo 90° muss
 * ausgeglichen werden, da der Ring selbst 90°-symmetrisch ist -> minimale
 * Servobewegung).
 */
float computeServoAngle(float headingDeg, const MountConfig& cfg);

} // namespace Positioning
