#include <RoutePlanner.h>

namespace
{
uint8_t cmToCell(float cm, float fieldCm, uint8_t cells)
{
    const int c = (int)(cm / fieldCm * cells);
    return (uint8_t)constrain(c, 0, (int)cells - 1);
}

float cellToCm(uint8_t cell, float fieldCm, uint8_t cells)
{
    return (cell + 0.5f) * fieldCm / cells;
}

/// true, wenn sich ein Hindernis gegenueber der letzten Planung bewegt hat (oder neu/weg ist)
bool obstacleMoved(const int16_t (&now)[2], const int16_t (&used)[2])
{
    const bool a = (now[0] >= 0 && now[1] >= 0);
    const bool b = (used[0] >= 0 && used[1] >= 0);
    if (a != b)
        return true;
    if (!a)
        return false;
    const float dx = now[0] - used[0];
    const float dy = now[1] - used[1];
    return sqrtf(dx * dx + dy * dy) > PATH_OBST_MOVE_CM;
}
} // namespace

void RoutePlanner::begin(Pathfinder &finder)
{
    _finder = &finder;
    reset();
}

void RoutePlanner::setObstacles(int16_t ballX, int16_t ballY, int16_t robotX, int16_t robotY)
{
    _ballIn[0] = ballX;
    _ballIn[1] = ballY;
    _robIn[0] = robotX;
    _robIn[1] = robotY;
}

void RoutePlanner::reset()
{
    _valid = false;
    _found = false;
    _wpCount = 0;
}

bool RoutePlanner::giveFound() const { return _found; }
uint8_t RoutePlanner::giveWaypoints() const { return _wpCount; }
uint32_t RoutePlanner::giveTimeUs() const { return _planUs; }

/**
 * A* + Vereinfachung -> Wegpunkte in cm.
 *
 * Versuche: 1) mit Ball und Partner, 2) ohne Ball (das Ziel liegt oft im
 * Ballbereich), 3) ohne Hindernisse (Startzelle im Sperrbereich). Jeder
 * Fehlversuch kostet Rechenzeit.
 */
bool RoutePlanner::_buildRoute(float sx, float sy, uint16_t tx, uint16_t ty, Route &out)
{
    const uint8_t W = PATH_GRID_X_CELLS;
    const uint8_t H = PATH_GRID_Y_CELLS;

    const uint8_t scx = cmToCell(sx, FIELD_X_CM, W), scy = cmToCell(sy, FIELD_Y_CM, H);
    const uint8_t tcx = cmToCell(tx, FIELD_X_CM, W), tcy = cmToCell(ty, FIELD_Y_CM, H);

    const bool hasBall = (_ballIn[0] >= 0 && _ballIn[1] >= 0);
    const bool hasRob = (_robIn[0] >= 0 && _robIn[1] >= 0);
    const uint8_t bx = hasBall ? cmToCell(_ballIn[0], FIELD_X_CM, W) : PATH_NO_OBSTACLE;
    const uint8_t by = hasBall ? cmToCell(_ballIn[1], FIELD_Y_CM, H) : PATH_NO_OBSTACLE;
    const uint8_t rx = hasRob ? cmToCell(_robIn[0], FIELD_X_CM, W) : PATH_NO_OBSTACLE;
    const uint8_t ry = hasRob ? cmToCell(_robIn[1], FIELD_Y_CM, H) : PATH_NO_OBSTACLE;

    bool ok = false;
    for (uint8_t attempt = 0; attempt < 3 && !ok; attempt++)
    {
        if (attempt == 1 && !hasBall)
            continue;
        if (attempt == 2 && !hasRob)
            continue;

        const bool useBall = (attempt == 0);
        const bool useRob = (attempt <= 1);
        ok = _finder->findPath(scx, scy, tcx, tcy,
                               useBall ? bx : PATH_NO_OBSTACLE, useBall ? by : PATH_NO_OBSTACLE,
                               useRob ? rx : PATH_NO_OBSTACLE, useRob ? ry : PATH_NO_OBSTACLE);
    }
    if (!ok)
    {
        return false;
    }

    // ---- Wegpunkte (cm) -------------------------------------------------
    uint8_t wp[ARRAY_MAX_POINTS][2];
    uint8_t n = (_finder->givePathLength() >= 2) ? _finder->giveWaypoints(wp, ARRAY_MAX_POINTS) : 0;

    float px[ARRAY_MAX_POINTS + 1];
    float py[ARRAY_MAX_POINTS + 1];
    px[0] = sx;
    py[0] = sy;
    for (uint8_t i = 0; i < n; i++)
    {
        px[i + 1] = cellToCm(wp[i][0], FIELD_X_CM, W);
        py[i + 1] = cellToCm(wp[i][1], FIELD_Y_CM, H);
    }
    if (n == 0)
        n = 1;  // Start und Ziel in derselben Zelle: nur das Ziel
    px[n] = tx; // letzter Punkt = exaktes Ziel, nicht Zellmitte
    py[n] = ty;

    // ---- Geometrie in die Route -----------------------------------------
    out.Count = n;
    out.TotalCm = 0.0f;
    for (uint8_t i = 0; i < n; i++)
    {
        const float dx = px[i + 1] - px[i];
        const float dy = py[i + 1] - py[i];

        out.Points[i].X = (uint16_t)constrain(px[i + 1] + 0.5f, 0.0f, FIELD_X_CM);
        out.Points[i].Y = (uint16_t)constrain(py[i + 1] + 0.5f, 0.0f, FIELD_Y_CM);
        out.Points[i].SegmentCm = sqrtf(dx * dx + dy * dy);
        out.TotalCm += out.Points[i].SegmentCm;
    }
    return true;
}

RouteResult RoutePlanner::update(const RouteRequest &req, Route &out)
{
    if (_finder == nullptr)
    {
        _found = false;
        return RouteFailed;
    }

    const bool targetChanged = !_valid || req.ToX != _tx || req.ToY != _ty;
    const bool moved = obstacleMoved(_ballIn, _ballUsed) || obstacleMoved(_robIn, _robUsed);
    const bool settled = _found && req.Arrived; // am Ziel: nicht dauernd neu planen
    const uint32_t interval = _found ? PATH_REPLAN_MS : PATH_REPLAN_FAIL_MS;
    const bool due = !settled && (uint32_t)(req.NowMs - _lastPlanMs) >= interval;

    if (!targetChanged && !moved && !due)
    {
        return RouteUnchanged;
    }

    _lastPlanMs = req.NowMs;
    _valid = true;
    _tx = req.ToX;
    _ty = req.ToY;
    for (uint8_t i = 0; i < 2; i++)
    {
        _ballUsed[i] = _ballIn[i];
        _robUsed[i] = _robIn[i];
    }

    const uint32_t t0 = micros();
    _found = _buildRoute(req.FromX, req.FromY, req.ToX, req.ToY, out);
    _planUs = micros() - t0;

    _wpCount = _found ? out.Count : 0;
    if (!_found)
    {
        out.Count = 0;
        out.TotalCm = 0.0f;
        return RouteFailed;
    }
    return RouteNew;
}
