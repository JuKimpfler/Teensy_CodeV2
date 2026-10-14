#pragma once
#include <Arduino.h>

/**
 * params_path.h  --  Pfadplanung (A*) fuer systemlib::moveToPointAvoid()
 *
 * A* selbst, Raster und Gewichtskarten gehoeren zum Pathfinder (lib/Pathfinder),
 * zuerst stehen die Werte fuer moveToPointAvoid(), danach die des A*.
 */

static constexpr uint32_t PATH_REPLAN_MS = 300;       ///< Neuplanung alle ... ms, solange der Roboter unterwegs ist
static constexpr uint32_t PATH_REPLAN_FAIL_MS = 1000; ///< nach einer gescheiterten Planung seltener erneut versuchen
static constexpr float PATH_OBST_MOVE_CM = 8.0f;      ///< Hindernis hat sich so weit bewegt -> sofort neu planen
static constexpr uint8_t PATH_PASS_CM = 12;           ///< Zwischenpunkte gelten ab diesem Abstand als durchfahren
static constexpr uint16_t PATH_LOS_MAX_CELLS = 80;    ///< so weit (in Zellen) wird beim Vereinfachen vorausgeschaut
static constexpr bool PATH_FALLBACK_DIRECT = true;    ///< true = ohne Weg geradeaus fahren, false = stehen bleiben

/* -------------- A*-Pathfinder (lib/Pathfinder) -------------- */
// Das Raster deckt das ganze Feld ab (FIELD_X_CM x FIELD_Y_CM), eine Zelle ist
// also etwa 2 cm gross. Die Gewichtskarten (params_pathmap.h) muessen genau
// zu diesen Massen passen.
static constexpr uint8_t PATH_GRID_X_CELLS = 90;     ///< Zellen entlang der kurzen Feldachse (X)
static constexpr uint8_t PATH_GRID_Y_CELLS = 120;    ///< Zellen entlang der langen Feldachse (Y)
static constexpr uint16_t PATH_MAX_CELLS = 600;      ///< laengster Zellpfad, den ein Suchergebnis aufnimmt
static constexpr uint8_t PATH_MASK_CELLS = 32;       ///< Kantenlaenge der Masken um Ball und Roboter (quadratisch)
static constexpr uint16_t PATH_HEAP_CAPACITY = 6000; ///< Platz der offenen Liste; ist sie voll, gilt die Suche als gescheitert
static constexpr uint8_t PATH_COST_STRAIGHT = 10;    ///< Schrittkosten geradeaus
static constexpr uint8_t PATH_COST_DIAGONAL = 14;    ///< Schrittkosten diagonal (~ 10 * sqrt 2)
static constexpr uint8_t PATH_BLOCKED_WEIGHT = 255;  ///< Gewicht einer nicht begehbaren Zelle
static constexpr uint8_t PATH_NO_OBSTACLE = 255;     ///< als Hindernis-Zelle: kein Hindernis, die Maske wird nicht gelegt
