#pragma once
#include <Arduino.h>

// Wire  = "ESP"-Bus (Teensy 18/19): BNO055 und alles am I2C-INTERCON
// Wire1 = "TNS"-Bus (Teensy 16/17): Portexpander und INA219 der Main-Platine
#define I2C_BNO Wire
#define I2C_IR Wire
#define I2C_SW Wire
#define I2C_US Wire
#define I2C_RADAR Wire
#define I2C_MAIN Wire1

#define BNO_ADDRESS 0x28
#define IR_ADDRESS 0x0A

#define INA_MAIN_ADDRESS 0x40  // Akku VBAT-4S
#define INA_MOTOR_ADDRESS 0x41 // VMOT
#define INA_DRIB_ADDRESS 0x45  // VDrib (in systemlib noch nicht ausgelesen)

#define EXP_MO34_ADDRESS 0x20  // IC7: MO3, MO4, SWITCH1/BUTTON1
#define EXP_MO12_ADDRESS 0x22  // IC9: MO1, MO2, Dribbler
#define EXP_POWER_ADDRESS 0x23 // IC8: Freigaben
#define SW_SWITCH_ADDRESS 0x26 // SwitchBoard IC1: SW1-5, Start, RCJ (Jumper JP1-3)
#define SW_BUTTON_ADDRESS 0x27 // SwitchBoard IC2: BTN1-5, RCJ-Out (Jumper JP4-6)

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
static constexpr float WHEEL_RADIUS_CM = 2.9f;     // Radradius in cm
static constexpr float GEAR_RATIO       = 16.0f;    // Getriebeuntersetzung (z. B. 14:1)
static constexpr float SQRT2_OVER_4     = 0.35355339f; // sqrt(2) / 4

// ---- Tuning ---------------------------------------------------------
constexpr float    FUS_SIGMA_RADAR_CM = 2.5f;   // Radar-Rauschen bei Certainty 1. GROESSER = mehr Vertrauen in Raeder
constexpr float    FUS_Q_BASE         = 1.0f;   // cm²/s   Grunddrift (auch im Stand, z.B. Anschieben)
constexpr float    FUS_Q_SPEED        = 0.15f;  // relativer Odometriefehler (15 % der Geschwindigkeit)
constexpr float    FUS_Q_ROT          = 1.0f;   // cm pro rad/s  Schlupf beim Drehen
constexpr float    FUS_GATE_SIGMA     = 3.0f;   // Gate-Breite
constexpr uint32_t FUS_REACQ_MS       = 300;    // stabil abweichender Radarwert -> Position neu setzen
constexpr uint32_t FUS_REACQ_OBST_MS  = 1500;   // dasselbe, wenn es wie ein Hindernis aussieht
constexpr float    FUS_REACQ_TOL_CM   = 4.0f;   // "stabil" = Werte streuen weniger als das
constexpr float    FUS_MAX_TILT_DEG   = 25.0f;  // Radar nur nutzen, wenn Roboter so nah an 0° / 180° steht
constexpr float    FUS_V_DEADBAND     = 2.0f;   // cm/s  darunter = Stand (Offset der Analogwerte)
constexpr float    FUS_ASSUMED_SPEED  = 60.0f;  // cm/s  fuer Q, wenn Odometrie abgeschaltet ist
constexpr float    FUS_P_INIT         = 900.0f; // cm²  Startunsicherheit (30 cm)
constexpr float    FUS_VALID_SIGMA    = 5.0f;   // cm    darunter gilt die Achse als "Valid"
constexpr float    FUS_LOST_SIGMA     = 40.0f;  // cm    darueber -> Position = -1 (verloren)
constexpr float    FUS_MIN_CERT       = 0.05f;

/* ------------------------------------------------------------------------
 * Odometrie-Kalibrierung im ROBOTER-Frame (vor der Drehung ins Feld!)
 *
 * Deine alten Faktoren 0.83 / 0.857 haben beide vy_rob skaliert, aber
 * je nach Heading nur in einem Term -> bei diagonalen Headings inkonsistent.
 * Hier wird die Achse einmal sauber skaliert, danach erst gedreht.
 * Feintuning: siehe Kalibrierfahrt in der Antwort.
 * --------------------------------------------------------------------- */
constexpr float ODO_SCALE_X = 0.860f; // Roboter quer 
constexpr float ODO_SCALE_Y = 0.860f; // Roboter laengs (Mittel aus 0.83 / 0.857)

// ---- Verzoegerungen (gegen das "Hinterherhinken") ---------------------
// Radar: Ein Wert ist ~Messdauer/2 + Abholverzoegerung alt, wenn er in
// Radar[..].Time landet. Der Filter vergleicht ihn deshalb mit der
// Position von damals (Historie), nicht mit der von jetzt.
constexpr uint32_t FUS_RADAR_LATENCY_MS = 30;
// Motoren: Analogausgang des Maxon-Treibers + ADC hinken der echten
// Drehzahl hinterher. Wird ueber die gemessene Beschleunigung ausgeglichen.
constexpr float    FUS_MOTOR_LAG_MS     = 20.0f;
constexpr float    FUS_MAX_ACCEL        = 600.0f; // cm/s²  physikalische Grenze, begrenzt Rauschen
constexpr float    FUS_ACCEL_LPF        = 0.35f;  // 0..1, kleiner = glatter, aber traeger
constexpr uint8_t  FUS_HIST_N           = 128;    // Positionshistorie (Zyklen)


/* ========================================================================
 * Speed-Pattern fuer systemlib::moveToPoint()   -> in params.h einfuegen
 *
 * Ein Pattern beschreibt nur den VERLAUF der Geschwindigkeit ueber den
 * Fahrweg. Fortschritt p = 0 (Start) ... 1 (Ziel), f(p) = 0 ... 1.
 * Gefahren wird:   Speed = MinSpeed + (MaxSpeed - MinSpeed) * f(p)
 * ===================================================================== */
#define SPattern_Linear         0   // f = 1 - p             linear bremsen: Max am Start -> Min am Ziel
#define SPattern_LinearUp       1   // f = p                 linear beschleunigen: Min am Start -> Max am Ziel
#define SPattern_Constant       2   // f = 1                 konstant MaxSpeed (Halt erst im Zielkreis)
#define SPattern_Trapezoid      3   // 20 % hoch, Reise, 40 % runter
#define SPattern_Triangle       4   // hoch bis zur Mitte, dann runter
#define SPattern_QuadDecel      5   // f = (1-p)^2           frueh bremsen, sanfter Auslauf
#define SPattern_ConstDecel     6   // f = sqrt(1-p)         konstante Verzoegerung (spaet bremsen)
#define SPattern_SmoothStep     7   // weiche S-Kurve runter (ruckfrei an Start und Ziel)
#define SPattern_Cosine         8   // Cosinus-Rampe runter
#define SPattern_Sine           9   // sin(pi*p): weich hoch, Spitze in der Mitte, weich runter
#define SPattern_SCurve         10  // weich anfahren, Reise, weich bremsen
#define SPattern_LateBrake      11  // Vollgas bis 70 %, dann weich runter
#define SPattern_FastThenCrawl  12  // Vollgas bis 50 %, dann runter, letzte 30 % Schleichfahrt (Min)
#define SPattern_QuadUp         13  // f = p^2               Anlauf, am Ziel schnell (Rammen)
#define SPattern_CubicUp        14  // f = p^3               noch spaeter beschleunigen
#define SPattern_Expo           15  // exponentiell abklingend
#define SPattern_Stairs         16  // 4 Stufen: 100 / 67 / 33 / 0 %
#define SPattern_Wave           17  // Sinus-Welle mit abklingender Huellkurve
#define SPattern_Pulse          18  // abwechselnd Max / Min (5 Perioden)
#define SPattern_COUNT          19

/* Feineinstellung moveToPoint */
#define MOVE_ARRIVE_CM          4.0f    // Zielkreis: darin Speed = 0
#define MOVE_ARRIVE_HYST_CM     3.0f    // erst ausserhalb Zielkreis + Hysterese wird nachgefahren
#define MOVE_MAX_ACCEL_PCT_S    10000.0f  // max. Zunahme des Speed [%/s], 0 = aus (Bremsen ist nie begrenzt)
#define MOVE_RESTART_GAP_MS     200     // Pause zwischen zwei Aufrufen, ab der das Pattern neu startet
#define MOVE_TARGET_TOL_CM      3.0f    // Zielaenderung bis zu diesem Wert startet das Pattern NICHT neu


#define D_Mode_Test 0
#define D_Mode_Game 1

#define DPDS_Mode D_Mode_Test
