#pragma once
#include <Arduino.h>

// Wire  = "ESP"-Bus (Teensy 18/19): BNO055 und alles am I2C-INTERCON
// Wire1 = "TNS"-Bus (Teensy 16/17): Portexpander und INA219 der Main-Platine
#define I2C_IR Wire1
#define I2C_SW Wire1
#define I2C_MAIN Wire1

#define IR_ADDRESS 0x0A

#define INA_MAIN_ADDRESS 0x40  // Akku VBAT-4S
#define INA_MOTOR_ADDRESS 0x41 // VMOT
#define INA_DRIB_ADDRESS 0x45  // VDrib (in systemlib noch nicht ausgelesen)

#define EXP_MO34_ADDRESS 0x20  // IC7: MO3, MO4, SWITCH1/BUTTON1
#define EXP_MO12_ADDRESS 0x22  // IC9: MO1, MO2, Dribbler
#define EXP_POWER_ADDRESS 0x23 // IC8: Freigaben
#define SW_SWITCH_ADDRESS 0x26 // SwitchBoard IC1: SW1-5, Start, RCJ (Jumper JP1-3)
#define SW_BUTTON_ADDRESS 0x27 // SwitchBoard IC2: BTN1-5, RCJ-Out (Jumper JP4-6)

// ── Feldgeometrie [cm] ───────────────────────────────────────────────────────
// Masse nach RCJ_SF23, gemessen von Wandinnenseite zu Wandinnenseite -- also
// genau die Strecke, die die Radarsensoren sehen. Nullpunkt ist die linke
// untere Ecke am eigenen Tor (siehe Coding_Rules.md, dort 0-182 / 0-243).
static constexpr float POS_FIELD_X_CM = 182.0f; ///< Breite: linke Wand -> rechte Wand
static constexpr float POS_FIELD_Y_CM = 243.0f; ///< Laenge: Wand am eigenen Tor -> Wand am gegnerischen Tor

// ── Hardware ein/aus ─────────────────────────────────────────────────────────
// 1 = wird in systemlib::initialize() eingerichtet und in readSensors() gelesen,
// 0 = wird komplett uebersprungen (kein Buszugriff, keine Wartezeit). Die
// Messwerte bleiben dann auf 0 stehen. Gedacht zum Testen mit abgesteckten
// Platinen oder zum Eingrenzen, wer die Zykluszeit/den Start ausbremst.
// Ueberschreibbar per build_flags, z. B. -DSYS_EN_RADAR=0
#ifndef SYS_EN_LINE
#define SYS_EN_LINE 0        // 6x ADC108S102 der Linienplatinen
#endif
#ifndef SYS_EN_POWER
#define SYS_EN_POWER 1       // INA219 Akku/Motor (Spannung, Strom)
#endif
#ifndef SYS_EN_SWITCHBOARD
#define SYS_EN_SWITCHBOARD 1 // SwitchBoard: Schalter, Taster, Start, RCJ-Modul
#endif
#ifndef SYS_EN_MOTORS
#define SYS_EN_MOTORS 1      // Motor-Expander (ESCON-Freigaben) und Motor-ADC; 0 = PWM bleibt auf 0
#endif

const uint16_t LineSensorSchwelle[48] = 
{3000,3000,3000,3000,3000,3000,3000,3000 // pex1
,3000,3000,3000,3000,3000,3000,3000,3000 // pex2
,3000,3000,3000,3000,3000,3000,3000,3000 // pex3
,3000,3000,3000,3000,3000,3000,3000,3000 // pex4
,3000,3000,3000,3000,3000,3000,3000,3000 // pex5
,3000,3000,3000,3000,3000,3000,3000,3000}; // pex6

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
