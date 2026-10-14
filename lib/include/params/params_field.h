#pragma once
#include <Arduino.h>

/**
 * params_field.h  --  Feldgeometrie [cm]
 *
 * Masse nach RCJ_SF23, gemessen von Wandinnenseite zu Wandinnenseite -- also
 * genau die Strecke, die die Radarsensoren sehen. Nullpunkt ist die linke
 * untere Ecke am eigenen Tor (siehe docs/Coding_Rules.md).
 */

static constexpr float FIELD_X_CM = 182.0f; ///< Breite: linke Wand -> rechte Wand
static constexpr float FIELD_Y_CM = 243.0f; ///< Laenge: Wand am eigenen Tor -> Wand am gegnerischen Tor

/* -------------- Tore -------------- */
// An beiden Enden steht ein Tor mittig auf der X-Achse und ragt
// FIELD_GOAL_DEPTH_CM ins Feld hinein. Steht der Roboter vor der Toroeffnung,
// sieht der vordere bzw. hintere Sensor die Torrueckwand statt der Feldwand --
// die Y-Achse ist dort um die Tortiefe kuerzer (an beiden Enden zusammen also
// um 2 x FIELD_GOAL_DEPTH_CM). Ist ein Tor stattdessen eine Nische IN der Wand
// (Feld dort laenger), hier einen negativen Wert eintragen.
static constexpr float FIELD_GOAL_WIDTH_CM = 60.0f; ///< Breite der Toroeffnung, mittig auf X
static constexpr float FIELD_GOAL_DEPTH_CM = 10.0f; ///< so weit steht das Tor im Feld

// Die Torkante ist fuer den Radar kein scharfer Sprung: die Keule ist breiter
// als ein Punkt und der Sensor sitzt nicht exakt auf der Roboterachse. In
// diesem Band um die Torkante steht nicht fest, ob der Sensor die Torrueckwand
// oder die Feldwand getroffen hat.
static constexpr float FIELD_GOAL_EDGE_CM = 12.0f;
