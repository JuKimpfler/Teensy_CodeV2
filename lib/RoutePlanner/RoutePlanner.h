#pragma once
#include <Arduino.h>

#include "params.h"
#include <MotionProfile.h>
#include <Pathfinder.h>

/**
 * RoutePlanner.h  --  Pfadplanung auf cm-Ebene
 *
 * Die Schicht zwischen Pathfinder (rechnet in Zellen) und MotionProfile
 * (faehrt die Punkte ab):
 *
 *   Position, Ziel, Ball, Partner  ->  Pathfinder::findPath()  (Raster PATH_GRID_X/Y_CELLS)
 *     ->  Zellpfad  ->  Pathfinder::giveWaypoints()  ->  Wegpunkte in cm  ->  Route
 *
 * RoutePlanner liefert nur Geometrie: Wegpunkte und Teilstueck-Laengen. Welches
 * Tempo daraus wird, rechnet MotionProfile::setRoute() -- deshalb kennt weder
 * Pathfinder noch RoutePlanner die Speed-Pattern.
 *
 * Neu geplant wird bei: neuem Ziel, bewegtem Hindernis (> PATH_OBST_MOVE_CM)
 * und zyklisch alle PATH_REPLAN_MS, solange der Roboter unterwegs ist.
 *
 * Parameter: lib/include/params/params_path.h. Doku: RoutePlanner.md
 */

/// Ergebnis eines RoutePlanner::update()-Aufrufs.
enum RouteResult : uint8_t
{
    RouteUnchanged = 0, ///< keine Neuplanung faellig, die bisherige Route gilt weiter
    RouteNew,           ///< neu geplant, out enthaelt die Route
    RouteFailed         ///< neu geplant, aber kein Weg gefunden
};

/// Ein Planungsauftrag, einmal je Zyklus.
struct RouteRequest
{
    uint32_t NowMs;  ///< millis()
    float FromX;     ///< aktuelle Position [cm]
    float FromY;     ///< aktuelle Position [cm]
    uint16_t ToX;    ///< Ziel [cm]
    uint16_t ToY;    ///< Ziel [cm]
    bool Arrived;    ///< true = am Ziel; dann wird nicht mehr zyklisch neu geplant
};

class RoutePlanner
{
public:
    /// Den A*-Pathfinder anmelden. Einmal beim Start aufrufen.
    void begin(Pathfinder &finder);

    /// Hindernisse [cm, Feldkoordinaten], -1 = kein Hindernis. Zyklisch aufrufbar, vor update().
    void setObstacles(int16_t ballX, int16_t ballY, int16_t robotX, int16_t robotY);

    /// Plant, wenn es noetig ist. Siehe RouteResult; out wird nur bei RouteNew gefuellt.
    RouteResult update(const RouteRequest &req, Route &out);

    /// Planung verwerfen; der naechste update() plant wieder.
    void reset();

    bool giveFound() const;      ///< A* hat beim letzten Versuch einen Weg gefunden
    uint8_t giveWaypoints() const; ///< Anzahl Wegpunkte der letzten Route
    uint32_t giveTimeUs() const;   ///< Rechenzeit der letzten Planung [us]

private:
    Pathfinder *_finder = nullptr;

    bool _valid = false; ///< es gibt eine Planung fuer das aktuelle Ziel
    bool _found = false; ///< A* hat einen Weg gefunden
    uint16_t _tx = 0;
    uint16_t _ty = 0;
    int16_t _ballIn[2] = {-1, -1};   ///< aktuelle Eingabe [cm], -1 = kein Hindernis
    int16_t _robIn[2] = {-1, -1};
    int16_t _ballUsed[2] = {-1, -1}; ///< beim letzten Planen verwendet
    int16_t _robUsed[2] = {-1, -1};
    uint32_t _lastPlanMs = 0;
    uint32_t _planUs = 0;
    uint8_t _wpCount = 0;

    bool _buildRoute(float sx, float sy, uint16_t tx, uint16_t ty, Route &out);
};
