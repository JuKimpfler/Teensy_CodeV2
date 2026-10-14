#include "Pathfinder.h"
#include "params/params_pathmap.h" // Gewichtskarten, nur hier (Flash)

namespace
{
constexpr int8_t DIR_DX[8] = {1, 1, 0, -1, -1, -1, 0, 1};
constexpr int8_t DIR_DY[8] = {0, 1, 1, 1, 0, -1, -1, -1};

static_assert(sizeof(PATH_WEIGHTS_FIELD) == (size_t)PATH_GRID_X_CELLS * PATH_GRID_Y_CELLS, "params_pathmap.h passt nicht zum Raster in params_path.h");
static_assert(sizeof(PATH_WEIGHTS_BALL) == (size_t)PATH_MASK_CELLS * PATH_MASK_CELLS, "Ballmaske hat die falsche Groesse");
static_assert(sizeof(PATH_WEIGHTS_ROBOT) == (size_t)PATH_MASK_CELLS * PATH_MASK_CELLS, "Robotermaske hat die falsche Groesse");
} // namespace

void Pathfinder::begin()
{
    memcpy(_field, PATH_WEIGHTS_FIELD, sizeof(_field));
    memset(_gStamp, 0, sizeof(_gStamp)); // das Objekt darf auch ohne Nullinitialisierung benutzt werden
    memset(_closedStamp, 0, sizeof(_closedStamp));
    _searchStamp = 1;
    _heapSize = 0;
    _heapPeak = 0;
    _heapOverflow = false;
    _pathLength = 0;
}

uint8_t Pathfinder::giveWeight(uint8_t x, uint8_t y) const
{
    if (x >= PATH_GRID_X_CELLS || y >= PATH_GRID_Y_CELLS)
    {
        return PATH_BLOCKED_WEIGHT;
    }
    return _field[x][y];
}

/* -------------- Suche -------------- */

bool Pathfinder::findPath(uint8_t startX, uint8_t startY, uint8_t targetX, uint8_t targetY,
                          uint8_t ballX, uint8_t ballY, uint8_t robotX, uint8_t robotY)
{
    _pathLength = 0;
    if (startX >= PATH_GRID_X_CELLS || startY >= PATH_GRID_Y_CELLS ||
        targetX >= PATH_GRID_X_CELLS || targetY >= PATH_GRID_Y_CELLS)
    {
        return false;
    }

    // Karte neu aufbauen: Feldgewichte, dann Ball, dann Roboter (ueberdeckt den Ball)
    memcpy(_field, PATH_WEIGHTS_FIELD, sizeof(_field));
    _placeMask(PATH_WEIGHTS_BALL, ballX, ballY);
    _placeMask(PATH_WEIGHTS_ROBOT, robotX, robotY);

    _targetX = targetX;
    _targetY = targetY;
    _resetSearch();

    // Das Ziel selbst ist immer billig, auch wenn es in einer Maske liegt
    const uint8_t targetWeight = _field[targetX][targetY];
    _field[targetX][targetY] = 0;

    Node current = {startX, startY, 0};
    _setKnownG(startX, startY, 0);

    bool found = true;
    while (!(current.X == targetX && current.Y == targetY))
    {
        _explore(current);
        if (_heapOverflow || !_popHeap(current))
        {
            found = false;
            break;
        }
        _setClosed(current.X, current.Y);
    }

    _field[targetX][targetY] = targetWeight; // Karte inkl. Masken bleibt fuer giveWeight() stehen

    if (!found)
    {
        return false;
    }
    return _buildPath(current, startX, startY);
}

uint16_t Pathfinder::_targetDistance(uint8_t x, uint8_t y) const
{
    const uint16_t dx = (x > _targetX) ? (x - _targetX) : (_targetX - x);
    const uint16_t dy = (y > _targetY) ? (y - _targetY) : (_targetY - y);
    const uint16_t diagonal = (dx < dy) ? dx : dy;
    const uint16_t straight = (dx > dy) ? (dx - dy) : (dy - dx);
    return diagonal * PATH_COST_DIAGONAL + straight * PATH_COST_STRAIGHT;
}

uint16_t Pathfinder::_knownG(uint8_t x, uint8_t y) const
{
    return (_gStamp[x][y] == _searchStamp) ? _gScore[x][y] : 0xFFFF;
}

void Pathfinder::_setKnownG(uint8_t x, uint8_t y, uint16_t g)
{
    _gStamp[x][y] = _searchStamp;
    _gScore[x][y] = g;
}

bool Pathfinder::_isClosed(uint8_t x, uint8_t y) const
{
    return _closedStamp[x][y] == _searchStamp;
}

void Pathfinder::_setClosed(uint8_t x, uint8_t y)
{
    _closedStamp[x][y] = _searchStamp;
}

void Pathfinder::_resetSearch()
{
    _searchStamp++;
    if (_searchStamp == 0) // Zaehler uebergelaufen: alte Stempel koennten wieder passen
    {
        _searchStamp = 1;
        memset(_gStamp, 0, sizeof(_gStamp));
        memset(_closedStamp, 0, sizeof(_closedStamp));
    }
    _heapSize = 0;
    _heapOverflow = false;
}

void Pathfinder::_placeMask(const uint8_t (&mask)[PATH_MASK_CELLS][PATH_MASK_CELLS], uint8_t centerX, uint8_t centerY)
{
    if (centerX == PATH_NO_OBSTACLE || centerY == PATH_NO_OBSTACLE)
    {
        return;
    }

    constexpr int half = PATH_MASK_CELLS / 2;
    for (int mx = 0; mx < PATH_MASK_CELLS; mx++)
    {
        for (int my = 0; my < PATH_MASK_CELLS; my++)
        {
            const int fx = (int)centerX + mx - half;
            const int fy = (int)centerY + my - half;
            if (fx < 0 || fx >= PATH_GRID_X_CELLS || fy < 0 || fy >= PATH_GRID_Y_CELLS)
            {
                continue;
            }
            _field[fx][fy] = mask[mx][my];
        }
    }
}

void Pathfinder::_explore(const Node &current)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        const int nx = (int)current.X + DIR_DX[i];
        const int ny = (int)current.Y + DIR_DY[i];
        if (nx < 0 || nx >= PATH_GRID_X_CELLS || ny < 0 || ny >= PATH_GRID_Y_CELLS)
        {
            continue;
        }

        const uint8_t ux = (uint8_t)nx;
        const uint8_t uy = (uint8_t)ny;
        if (_field[ux][uy] == PATH_BLOCKED_WEIGHT || _isClosed(ux, uy))
        {
            continue;
        }

        const uint16_t moveG = current.G + ((DIR_DX[i] == 0 || DIR_DY[i] == 0) ? PATH_COST_STRAIGHT : PATH_COST_DIAGONAL);
        if (moveG < _knownG(ux, uy))
        {
            _setKnownG(ux, uy, moveG);
            _origin[ux][uy] = i;
            _pushHeap(ux, uy, moveG, (uint32_t)_targetDistance(ux, uy) + moveG + _field[ux][uy]);
        }
    }
}

/* -------------- Offene Liste (Min-Heap nach F) -------------- */

void Pathfinder::_pushHeap(uint8_t x, uint8_t y, uint16_t g, uint32_t f)
{
    if (_heapSize >= PATH_HEAP_CAPACITY)
    {
        _heapOverflow = true; // Suche wird abgebrochen, statt hinter das Feld zu schreiben
        return;
    }

    int i = _heapSize++;
    if (_heapSize > _heapPeak)
    {
        _heapPeak = _heapSize;
    }
    _heap[i] = {f, g, x, y};

    while (i > 0)
    {
        const int parent = (i - 1) / 2;
        if (_heap[parent].F <= _heap[i].F)
        {
            break;
        }
        const HeapEntry tmp = _heap[parent];
        _heap[parent] = _heap[i];
        _heap[i] = tmp;
        i = parent;
    }
}

/// Naechsten noch offenen Knoten mit den geringsten Kosten holen. Veraltete Eintraege (Zelle schon
/// abgeschlossen oder inzwischen guenstiger erreicht) werden uebersprungen.
bool Pathfinder::_popHeap(Node &next)
{
    while (_heapSize > 0)
    {
        const HeapEntry top = _heap[0];

        _heapSize--;
        _heap[0] = _heap[_heapSize];

        int i = 0;
        while (true)
        {
            const int l = 2 * i + 1;
            const int r = l + 1;
            int smallest = i;
            if (l < _heapSize && _heap[l].F < _heap[smallest].F)
            {
                smallest = l;
            }
            if (r < _heapSize && _heap[r].F < _heap[smallest].F)
            {
                smallest = r;
            }
            if (smallest == i)
            {
                break;
            }
            const HeapEntry tmp = _heap[i];
            _heap[i] = _heap[smallest];
            _heap[smallest] = tmp;
            i = smallest;
        }

        if (_isClosed(top.X, top.Y) || _knownG(top.X, top.Y) != top.G)
        {
            continue;
        }

        next = {top.X, top.Y, top.G};
        return true;
    }
    return false;
}

/* -------------- Ergebnis -------------- */

bool Pathfinder::_buildPath(Node current, uint8_t startX, uint8_t startY)
{
    // Vom Ziel ueber die Herkunftsrichtungen zurueck zum Start, dann umdrehen
    uint16_t length = 1;
    _path[0][0] = current.X;
    _path[0][1] = current.Y;

    uint8_t x = current.X;
    uint8_t y = current.Y;
    while (!(x == startX && y == startY))
    {
        const uint8_t dir = _origin[x][y];
        x = x - DIR_DX[dir];
        y = y - DIR_DY[dir];

        if (length >= PATH_MAX_CELLS)
        {
            return false;
        }
        _path[length][0] = x;
        _path[length][1] = y;
        length++;
    }

    for (uint16_t i = 0; i < length / 2; i++)
    {
        const uint16_t j = length - 1 - i;
        const uint8_t tx = _path[i][0];
        const uint8_t ty = _path[i][1];
        _path[i][0] = _path[j][0];
        _path[i][1] = _path[j][1];
        _path[j][0] = tx;
        _path[j][1] = ty;
    }

    _pathLength = length;
    return true;
}

/* -------------- Linie / Vereinfachen -------------- */

bool Pathfinder::lineClear(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t costLimit) const
{
    int x = x0;
    int y = y0;
    const int dx = abs((int)x1 - (int)x0);
    const int dy = abs((int)y1 - (int)y0);
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true)
    {
        const uint8_t w = giveWeight((uint8_t)x, (uint8_t)y);
        if (w >= PATH_BLOCKED_WEIGHT || w > costLimit)
        {
            return false;
        }
        if (x == x1 && y == y1)
        {
            return true;
        }
        const int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }
}

uint8_t Pathfinder::giveWaypoints(uint8_t (*waypoints)[2], uint8_t maxPoints) const
{
    uint8_t runMax[PATH_LOS_MAX_CELLS + 1];
    uint8_t n = 0;
    uint16_t i = 0;

    while (i + 1 < _pathLength && n < maxPoints)
    {
        const uint16_t jMax = (i + PATH_LOS_MAX_CELLS < _pathLength - 1) ? (i + PATH_LOS_MAX_CELLS) : (_pathLength - 1);

        // Teuerste Zelle des A*-Wegs bis j: so teuer darf auch die Abkuerzung hoechstens sein
        uint8_t m = 0;
        for (uint16_t j = i + 1; j <= jMax; j++)
        {
            const uint8_t w = _field[_path[j][0]][_path[j][1]];
            if (w > m)
            {
                m = w;
            }
            runMax[j - i] = m;
        }

        uint16_t best = i + 1; // mindestens ein Schritt: der A*-Weg ist begehbar
        for (uint16_t j = jMax; j > i + 1; j--)
        {
            if (lineClear(_path[i][0], _path[i][1], _path[j][0], _path[j][1], runMax[j - i]))
            {
                best = j;
                break;
            }
        }

        waypoints[n][0] = _path[best][0];
        waypoints[n][1] = _path[best][1];
        n++;
        i = best;
    }
    return n;
}
