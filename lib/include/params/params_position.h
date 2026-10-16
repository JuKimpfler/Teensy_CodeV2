#pragma once
#include <Arduino.h>

/**
 * params_position.h  --  Positionsbestimmung (Radar-Validierung, Odometrie, Fusion)
 *
 * Verwendet in systemlib::calculatePosition(). Feldmasse: params_field.h.
 *
 *   ODO_*  Raeder -> Geschwindigkeit
 *   POS_*  Radarwerte pruefen und bewerten (Certainty)
 *   FUS_*  Kalman-Filter je Achse
 */

/* -------------- Odometrie -------------- */
static constexpr float ODO_WHEEL_RADIUS_CM = 2.9f; ///< Radradius
static constexpr float ODO_GEAR_RATIO = 16.0f;     ///< Getriebeuntersetzung Motor : Rad

/// Motordrehzahl [U/min] -> Radgeschwindigkeit [cm/s]: 2 * pi * Radradius / (Untersetzung * 60 s)
static constexpr float ODO_RPM_TO_CM_S = (float)(2.0 * 3.14159265358979323846 * ODO_WHEEL_RADIUS_CM / (ODO_GEAR_RATIO * 60.0));
static constexpr float ODO_DEG_TO_RAD = (float)(3.14159265358979323846 / 180.0);

// Kalibrierung im ROBOTER-Frame (vor der Drehung ins Feld): jede Achse wird
// einmal sauber skaliert, danach erst gedreht.
static constexpr float ODO_SCALE_X = 0.860f; ///< Roboter quer
static constexpr float ODO_SCALE_Y = 0.860f; ///< Roboter laengs (Mittel aus 0.83 / 0.857)

/* -------------- Radar -> Achsenmessung -------------- */
// Je Achse messen zwei gegenueberliegende Sensoren dieselbe Feldbreite aus:
// Weg zur einen Wand + Weg zur anderen Wand = Feldgroesse. Passt das Paar
// zusammen, ist die Position sicher; passt es nicht, hat mindestens einer der
// beiden etwas anderes als die Wand gesehen.
static constexpr float POS_RADAR_OFFSET_CM = 2.0f;   ///< Abstand Roboter-Mittelpunkt -> Sensorflaeche (Koordinaten zaehlen ab Mittelpunkt)
static constexpr float POS_PAIR_TOL_CM = 10.0f;      ///< so weit darf die Summe des Paares von der Feldgroesse abweichen
static constexpr float POS_BLIND_CM = 22.0f;         ///< misst ein Sensor mehr als Feldgroesse - POS_BLIND_CM, steht der gegenueberliegende im Blindbereich (RADAR_START_MM) und wird verworfen
static constexpr float POS_MAX_OVERSHOOT_CM = 10.0f; ///< so weit darf ein Messwert ueber die Feldbreite hinausgehen
static constexpr uint32_t POS_AGE_MAX_MS = 1000;     ///< ab diesem Alter zaehlt ein Messwert gar nicht mehr (Certainty 0)

// Plausibilitaet vor der Altersbewertung. Bei einem passenden Paar wird
// zwischen PAIR (Summe stimmt genau) und SINGLE (Abweichung = POS_PAIR_TOL_CM)
// linear interpoliert.
static constexpr float POS_PLAUS_PAIR = 1.00f;      ///< Paar passt zur Feldgroesse
static constexpr float POS_PLAUS_SINGLE = 0.60f;    ///< nur ein Sensor nutzbar -> keine Gegenprobe moeglich
static constexpr float POS_PLAUS_CONFLICT = 0.30f;  ///< Paar widerspricht sich -> naeherer Wert, aber unsicher
static constexpr float POS_PLAUS_GOAL_EDGE = 0.30f; ///< Certainty-Faktor der Y-Achse, solange die Torkante nicht eindeutig ist

// Signalstaerke [mdB]: Mindeststaerke faellt mit dem Abstand (weiter weg = schwaecher).
//   minStrength = max(POS_STRENGTH_MIN_NEAR_MDB - Abstand[m] * POS_STRENGTH_SLOPE_MDB_M, POS_STRENGTH_MIN_FAR_MDB)
static constexpr float POS_STRENGTH_MIN_NEAR_MDB = -11000.0f; ///< Mindeststaerke direkt am Sensor
static constexpr float POS_STRENGTH_SLOPE_MDB_M = 2500.0f;    ///< Abfall je Meter
static constexpr float POS_STRENGTH_MIN_FAR_MDB = -17000.0f;  ///< Untergrenze der Mindeststaerke
// Certainty-Faktor aus der Staerke: sf = 1 + (Staerke + OFFSET) / SPAN, begrenzt auf POS_STRENGTH_CERT_MIN ... 1.
// Bei Staerke = -OFFSET ist der Faktor 1, darunter faellt er linear ab.
static constexpr float POS_STRENGTH_CERT_OFFSET_MDB = 10000.0f;
static constexpr float POS_STRENGTH_CERT_SPAN_MDB = 15000.0f;
static constexpr float POS_STRENGTH_CERT_MIN = 0.7f;

/* -------------- Fusion (Kalman je Achse) -------------- */
static constexpr float FUS_SIGMA_RADAR_CM = 2.5f;   ///< Radar-Rauschen bei Certainty 1. GROESSER = mehr Vertrauen in Raeder
static constexpr float FUS_Q_BASE = 1.0f;           ///< cm^2/s   Grunddrift (auch im Stand, z.B. Anschieben)
static constexpr float FUS_Q_SPEED = 0.15f;         ///< relativer Odometriefehler (15 % der Geschwindigkeit)
static constexpr float FUS_Q_ROT = 1.0f;            ///< cm pro rad/s  Schlupf beim Drehen
static constexpr float FUS_GATE_SIGMA = 3.0f;       ///< Gate-Breite
static constexpr uint32_t FUS_REACQ_MS = 300;       ///< stabil abweichender Radarwert -> Position neu setzen
static constexpr uint32_t FUS_REACQ_OBST_MS = 1500; ///< dasselbe, wenn es wie ein Hindernis aussieht
static constexpr float FUS_REACQ_TOL_CM = 4.0f;     ///< "stabil" = Werte streuen weniger als das
static constexpr float FUS_MAX_TILT_DEG = 25.0f;    ///< Radar nur nutzen, wenn der Roboter so nah an 0 / 180 Grad steht
static constexpr float FUS_TILT_SCALE_DEG = 10.0f;  ///< Schraeglage: Sigma waechst mit 1 + (tilt / FUS_TILT_SCALE_DEG)^2
static constexpr float FUS_V_DEADBAND = 2.0f;       ///< cm/s  darunter = Stand (Offset der Analogwerte)
static constexpr float FUS_ASSUMED_SPEED = 60.0f;   ///< cm/s  fuer Q, wenn die Odometrie abgeschaltet ist
static constexpr float FUS_P_INIT = 900.0f;         ///< cm^2  Startunsicherheit (30 cm)
static constexpr float FUS_VALID_SIGMA = 5.0f;      ///< cm    darunter gilt die Achse als "Valid"
static constexpr float FUS_LOST_SIGMA = 40.0f;      ///< cm    darueber -> Position = -1 (verloren)
static constexpr float FUS_MIN_CERT = 0.05f;
static constexpr float FUS_CERT_SLOPE = 0.3f; ///< Position.Certainty = 1 / (1 + FUS_CERT_SLOPE * sigma)

// Verzoegerungen (gegen das "Hinterherhinken")
// Radar: ein Wert ist ~Messdauer/2 + Abholverzoegerung alt, wenn er in
// Radar[..].Time landet. Der Filter vergleicht ihn deshalb mit der Position
// von damals (Historie), nicht mit der von jetzt.
static constexpr uint32_t FUS_RADAR_LATENCY_MS = 30;
static constexpr uint32_t FUS_RADAR_LATENCY_LINK_MS = 0; ///< Quelle ESP: der Zeitstempel ist schon die Mitte der Messrunde, keine Schaetzung noetig
// Motoren: Analogausgang des Maxon-Treibers + ADC hinken der echten Drehzahl
// hinterher. Wird ueber die gemessene Beschleunigung ausgeglichen.
static constexpr float FUS_MOTOR_LAG_MS = 20.0f;
static constexpr float FUS_MAX_ACCEL = 600.0f; ///< cm/s^2  physikalische Grenze, begrenzt Rauschen
static constexpr float FUS_ACCEL_LPF = 0.35f;  ///< 0..1, kleiner = glatter, aber traeger
static constexpr uint8_t FUS_HIST_N = 128;     ///< Positionshistorie (Zyklen)
