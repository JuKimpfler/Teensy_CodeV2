#pragma once
#include <Arduino.h>

/**
 * params_motion.h  --  Fahrfunktionen: moveToPoint(), Punkt-Array, Wandabstand
 *
 * Ein Speed-Pattern beschreibt nur den VERLAUF der Geschwindigkeit ueber den
 * Fahrweg. Fortschritt p = 0 (Start) ... 1 (Ziel), f(p) = 0 ... 1.
 * Gefahren wird:   Speed = MinSpeed + (MaxSpeed - MinSpeed) * f(p)
 */

enum SPattern : uint8_t
{
    SPattern_Linear = 0,    ///< f = 1 - p             linear bremsen: Max am Start -> Min am Ziel
    SPattern_LinearUp,      ///< f = p                 linear beschleunigen: Min am Start -> Max am Ziel
    SPattern_Constant,      ///< f = 1                 konstant MaxSpeed (Halt erst im Zielkreis)
    SPattern_Trapezoid,     ///< 20 % hoch, Reise, 40 % runter
    SPattern_Triangle,      ///< hoch bis zur Mitte, dann runter
    SPattern_QuadDecel,     ///< f = (1-p)^2           frueh bremsen, sanfter Auslauf
    SPattern_ConstDecel,    ///< f = sqrt(1-p)         konstante Verzoegerung (spaet bremsen)
    SPattern_SmoothStep,    ///< weiche S-Kurve runter (ruckfrei an Start und Ziel)
    SPattern_Cosine,        ///< Cosinus-Rampe runter
    SPattern_Sine,          ///< sin(pi*p): weich hoch, Spitze in der Mitte, weich runter
    SPattern_SCurve,        ///< weich anfahren, Reise, weich bremsen
    SPattern_LateBrake,     ///< Vollgas bis 70 %, dann weich runter
    SPattern_FastThenCrawl, ///< Vollgas bis 50 %, dann runter, letzte 30 % Schleichfahrt (Min)
    SPattern_QuadUp,        ///< f = p^2               Anlauf, am Ziel schnell (Rammen)
    SPattern_CubicUp,       ///< f = p^3               noch spaeter beschleunigen
    SPattern_Expo,          ///< exponentiell abklingend
    SPattern_Stairs,        ///< 4 Stufen: 100 / 67 / 33 / 0 %
    SPattern_Wave,          ///< Sinus-Welle mit abklingender Huellkurve
    SPattern_Pulse,         ///< abwechselnd Max / Min (5 Perioden)
    SPattern_COUNT
};

/* -------------- moveToPoint -------------- */
static constexpr float MOVE_ARRIVE_CM = 4.0f;           ///< Zielkreis: darin Speed = 0
static constexpr float MOVE_ARRIVE_HYST_CM = 3.0f;      ///< erst ausserhalb Zielkreis + Hysterese wird nachgefahren
static constexpr float MOVE_MAX_ACCEL_PCT_S = 10000.0f; ///< max. Zunahme des Speed [%/s], 0 = aus (Bremsen ist nie begrenzt)
static constexpr uint32_t MOVE_RESTART_GAP_MS = 200;    ///< Pause zwischen zwei Aufrufen, ab der das Pattern neu startet
static constexpr float MOVE_TARGET_TOL_CM = 3.0f;       ///< Zielaenderung bis zu diesem Wert startet das Pattern NICHT neu

/* -------------- Wandabstand (moveDir) -------------- */
static constexpr float MOVE_WALL_MARGIN_CM = 20.0f; ///< naeher als das an der Wand -> Speed wird begrenzt
static constexpr uint8_t MOVE_WALL_SPEED_PCT = 5;   ///< Speed [%] im Wandbereich

/* -------------- Punkt-Array -------------- */
static constexpr uint8_t ARRAY_MAX_POINTS = 16;      ///< Platz fuer Punkte (auch fuer die Wegpunkte der Pfadplanung)
static constexpr float ARRAY_HEADING_TOL_DEG = 5.0f; ///< Winkeltoleranz beim Weiterschalten, 0 = Winkel wird nicht geprueft
