#pragma once
#include <Arduino.h>

#define I2C_BNO Wire
#define I2C_IR Wire
#define I2C_SW Wire
#define I2C_US Wire
#define I2C_RADAR Wire

#define BNO_ADDRESS 0x28

const uint16_t LineSensorSchwelle[48] = 
{3000,3000,3000,3000,3000,3000,3000,3000 // pex1
,3000,3000,3000,3000,3000,3000,3000,3000 // pex2
,3000,3000,3000,3000,3000,3000,3000,3000 // pex3
,3000,3000,3000,3000,3000,3000,3000,3000 // pex4
,3000,3000,3000,3000,3000,3000,3000,3000 // pex5
,3000,3000,3000,3000,3000,3000,3000,3000}; // pex6

static constexpr uint8_t RADAR_NUM_SENSORS = 4;
static constexpr uint8_t RADAR_SENSOR_ADDRS[RADAR_NUM_SENSORS] = { 0x58,0x56,0x55,0x52 };
static constexpr int8_t  RADAR_WAKE_PIN = -1;
static constexpr int8_t  RADAR_INT_PIN  = -1;

// ── Radar-Abstimmung: gerade Holzwand bis 2,5 m ───────────────────────────────
// Zielbild: eine planare Holzflaeche in 0,2 m ... 2,5 m Abstand soll bei jeder
// Messung als Peak auftauchen. Holz reflektiert deutlich schwaecher als Metall,
// deshalb sind Signalqualitaet und CFAR-Empfindlichkeit gegenueber den
// Acconeer-Defaults angehoben. Ermittelt/nachpruefbar mit dem Sketch
// lib/XM125/examples/XM125_Calib_Sensitivity.cpp (Mode 's' = Parameter-Sweep). Sieht der
// Sensor die Wand ueberhaupt nicht, vorher lib/XM125/examples/XM125_Calib_Alignment.cpp
// (Ausrichtung) und lib/XM125/examples/XM125_Calib_Module.cpp (Kalibrierung laeuft durch?).
static constexpr uint32_t RADAR_START_MM = 150;   ///< naeher am Sensor liegt der Roboter selbst
static constexpr uint32_t RADAR_END_MM   = 2700;  ///< 200 mm Reserve, damit 3000 mm nicht auf der Bereichskante liegt
static constexpr uint32_t RADAR_SIGNAL_QUALITY        = 30000; ///< 25.0 statt Default 15.0 -> mehr SNR am Bereichsende
static constexpr uint32_t RADAR_THRESHOLD_SENSITIVITY = 0;   ///< 0.75 statt Default 0.50 -> CFAR laesst schwache Wand-Peaks durch
static constexpr uint32_t RADAR_MEASURE_TIMEOUT_MS    = 3000;  ///< PROFILE5 + hohe Signalqualitaet braucht laenger als der 1000ms-Default
//static constexpr uint32_t RADAR_THRESHOLD_STRENGHT = 23;
// Alle Sensoren messen grundsaetzlich GLEICHZEITIG: systemlib::readRadar()
// stoesst eine Runde bei allen Sensoren an und sieht in den Folgezyklen nur
// noch nach, ob die Ergebnisse da sind. Gewartet wird nirgends, die Messdauer
// faellt also einmal fuer alle Sensoren an statt einmal pro Sensor.
// Nachteil: alle Module senden im selben Frequenzband zur selben Zeit, also
// ist Uebersprechen moeglich. Nachmessen, ob das die Trefferquote kostet:
// lib/XM125/examples/XM125_Calib_Crosstalk.cpp (Mode 'g' vergleicht
// nacheinander gegen gleichzeitig, 'i'/'p' zeigen den einzelnen Stoerer).
// Nur wie viele PEAKS je Messung aus dem Sensor geholt werden, ist noch eine
// Stellschraube -- jeder Peak kostet zwei Registerlesezugriffe:
static constexpr uint8_t RADAR_PEAKS_TO_FETCH = 1; ///< ausgewertet wird nur Peak 0
// Ein nicht erreichbarer Sensor wird neu initialisiert -- das blockiert
// (Kalibrierung dauert bis ~2 s), darf also nicht in jedem Zyklus passieren:
static constexpr uint32_t RADAR_REINIT_INTERVAL_MS = 500;

// ── Positionsbestimmung aus den Radarwerten (systemlib::calculatePosition) ────
// Je Achse messen zwei gegenueberliegende Sensoren dieselbe Feldbreite aus:
// Weg zur einen Wand + Weg zur anderen Wand = Feldgroesse. Passt das Paar
// zusammen, ist die Position sicher; passt es nicht, hat mindestens einer der
// beiden etwas anderes als die Wand gesehen.
static constexpr float POS_RADAR_OFFSET_CM = 2.0f;  ///< Abstand Roboter-Mittelpunkt -> Sensorflaeche (Koordinaten zaehlen ab Mittelpunkt)
static constexpr float POS_PAIR_TOL_CM     = 10.0f; ///< so weit darf die Summe des Paares von der Feldgroesse abweichen
static constexpr float POS_BLIND_CM        = 22.0f; ///< misst ein Sensor mehr als Feldgroesse - POS_BLIND_CM, steht der gegenueberliegende im Blindbereich (RADAR_START_MM) und wird verworfen
static constexpr uint32_t POS_AGE_MAX_MS   = 1000;  ///< ab diesem Alter zaehlt ein Messwert gar nicht mehr (Certainty 0)

// Plausibilitaet vor der Altersbewertung. Bei einem passenden Paar wird
// zwischen PAIR (Summe stimmt genau) und SINGLE (Abweichung = POS_PAIR_TOL_CM)
// linear interpoliert.
static constexpr float POS_PLAUS_PAIR     = 1.00f; ///< Paar passt zur Feldgroesse
static constexpr float POS_PLAUS_SINGLE   = 0.60f; ///< nur ein Sensor nutzbar -> keine Gegenprobe moeglich
static constexpr float POS_PLAUS_CONFLICT = 0.30f; ///< Paar widerspricht sich -> naeherer Wert, aber unsicher

// ── Feldgeometrie [cm] ───────────────────────────────────────────────────────
// Masse nach RCJ_SF23, gemessen von Wandinnenseite zu Wandinnenseite -- also
// genau die Strecke, die die Radarsensoren sehen. Nullpunkt ist die linke
// untere Ecke am eigenen Tor (siehe Coding_Rules.md, dort 0-182 / 0-243).
static constexpr float POS_FIELD_X_CM = 182.0f; ///< Breite: linke Wand -> rechte Wand
static constexpr float POS_FIELD_Y_CM = 243.0f; ///< Laenge: Wand am eigenen Tor -> Wand am gegnerischen Tor

// ── Tore ─────────────────────────────────────────────────────────────────────
// An beiden Enden steht ein Tor mittig auf der X-Achse und ragt POS_GOAL_DEPTH_CM
// ins Feld hinein. Steht der Roboter vor der Toroeffnung, sieht der vordere bzw.
// hintere Sensor die Torrueckwand statt der Feldwand -- die Y-Achse ist dort um
// die Tortiefe kuerzer (an beiden Enden zusammen also um 2 x POS_GOAL_DEPTH_CM).
// Ist ein Tor stattdessen eine Nische IN der Wand (Feld dort laenger), hier
// einen negativen Wert eintragen; die Rechnung stimmt dann genauso.
static constexpr float POS_GOAL_WIDTH_CM = 60.0f; ///< Breite der Toroeffnung, mittig auf X
static constexpr float POS_GOAL_DEPTH_CM = 10.0f; ///< so weit steht das Tor im Feld
// Die Torkante ist fuer den Radar kein scharfer Sprung: die Keule ist breiter
// als ein Punkt und der Sensor sitzt nicht exakt auf der Roboterachse. In
// diesem Band um die Torkante steht nicht fest, ob der Sensor die Torrueckwand
// oder die Feldwand getroffen hat.
static constexpr float POS_GOAL_EDGE_CM    = 12.0f;
static constexpr float POS_PLAUS_GOAL_EDGE = 0.30f; ///< Certainty-Faktor der Y-Achse, solange die Torkante nicht eindeutig ist

static constexpr uint8_t POS_MAX_OVERSHOOT_CM = 10;
// Schwellenwerte für Positionsvalidierung und Vertrauen
static constexpr float POS_TRUST_THRESHOLD = 0.3f; // Schwelle für echtes Radar-Vertrauen
static constexpr float POS_MIN_THRESHOLD   = 0.1f; // Schwelle für Fallback/Odometrie-Überbrückung

// --- Systemparameter ---
static constexpr float WHEEL_RADIUS_CM = 5.0f;     // Radradius in cm
static constexpr float GEAR_RATIO       = 16.0f;    // Getriebeuntersetzung (z. B. 14:1)
static constexpr float SQRT2_OVER_4     = 0.35355339f; // sqrt(2) / 4


#define D_Mode_Test 0
#define D_Mode_Game 1

#define DPDS_Mode D_Mode_Test
