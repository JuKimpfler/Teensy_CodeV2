#pragma once
#include <Arduino.h>
#include "params.h"

/**
 * Pathfinder  --  A*-Wegsuche auf einem Gewichtsraster (Teensy 4.x)
 *
 * Das Feld ist ein Raster PATH_GRID_X_CELLS x PATH_GRID_Y_CELLS (params_path.h)
 * mit einem Gewicht je Zelle (params_pathmap.h). Vor jeder Suche werden Masken
 * um Ball und zweiten Roboter in die Karte gelegt; die Suche liefert einen
 * Zellpfad vom Start zum Ziel, der Zellen mit hohem Gewicht meidet.
 *
 * Die Klasse kennt weder Position noch Motoren: Koordinaten sind Rasterzellen,
 * die Umrechnung von/nach cm macht systemlib. Rechenzeit: wenige ms, blockiert.
 *
 * ACHTUNG: Das Objekt belegt rund 130 kB. Nur global/statisch anlegen,
 * nie auf dem Stack.
 */
class Pathfinder
{
public:
    /// Karte laden. Vor der ersten Suche aufrufen (damit giveWeight() schon gueltig ist).
    void begin();

    /**
     * Weg von der Start- zur Zielzelle suchen. Die Zellen liegen in 0 ... PATH_GRID_*_CELLS - 1.
     * Ball / Roboter: Zelle des Hindernisses, PATH_NO_OBSTACLE = keins.
     * Das Ergebnis (inkl. Start- und Zielzelle) steht danach in givePathX/Y().
     *
     * @return true bei gefundenem Weg. false, wenn kein Weg existiert, eine Koordinate ausserhalb
     *         des Rasters liegt, der Weg laenger als PATH_MAX_CELLS waere oder die offene Liste voll lief.
     */
    bool findPath(uint8_t startX, uint8_t startY, uint8_t targetX, uint8_t targetY,
                  uint8_t ballX, uint8_t ballY, uint8_t robotX, uint8_t robotY);

    uint16_t givePathLength() const { return _pathLength; }
    uint8_t givePathX(uint16_t index) const { return _path[index][0]; }
    uint8_t givePathY(uint16_t index) const { return _path[index][1]; }
    const uint8_t (&givePath() const)[PATH_MAX_CELLS][2] { return _path; } ///< fuer PathfinderTelemetry

    /// Gewicht der Zelle inkl. der Masken der letzten Suche. Ausserhalb des Rasters: PATH_BLOCKED_WEIGHT.
    uint8_t giveWeight(uint8_t x, uint8_t y) const;
    const uint8_t (&giveWeights() const)[PATH_GRID_X_CELLS][PATH_GRID_Y_CELLS] { return _field; } ///< fuer PathfinderTelemetry

    /// true, wenn auf der Linie (Bresenham) nichts blockiert ist und keine Zelle teurer als costLimit ist.
    bool lineClear(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t costLimit) const;

    /**
     * Zellpfad der letzten Suche auf wenige Wegpunkte reduzieren: von jedem Ankerpunkt so weit
     * voraus (PATH_LOS_MAX_CELLS), wie eine freie Gerade moeglich ist. Die Gerade darf nie durch
     * teurere Zellen laufen als der A*-Weg selbst, die Gewichte der Karte bleiben also wirksam.
     *
     * @param waypoints  Platz fuer maxPoints Zellen [x, y]
     * @return Anzahl der Wegpunkte (ohne Startzelle, der letzte ist die Zielzelle)
     */
    uint8_t giveWaypoints(uint8_t (*waypoints)[2], uint8_t maxPoints) const;

    /// Hoechster Fuellstand der offenen Liste seit begin() (zum Dimensionieren von PATH_HEAP_CAPACITY).
    uint16_t giveHeapPeak() const { return _heapPeak; }

private:
    struct HeapEntry
    {
        uint32_t F; ///< Gesamtkosten (Strecke + Restweg + Zellgewicht)
        uint16_t G; ///< Strecke bis hierher
        uint8_t X;
        uint8_t Y;
    };
    struct Node
    {
        uint8_t X;
        uint8_t Y;
        uint16_t G;
    };

    static constexpr uint16_t GRID_CELLS = (uint16_t)PATH_GRID_X_CELLS * PATH_GRID_Y_CELLS;

    uint8_t _field[PATH_GRID_X_CELLS][PATH_GRID_Y_CELLS];   ///< Arbeitskarte: Feldgewichte + Masken
    uint16_t _gScore[PATH_GRID_X_CELLS][PATH_GRID_Y_CELLS]; ///< bisher beste Strecke je Zelle
    uint16_t _gStamp[PATH_GRID_X_CELLS][PATH_GRID_Y_CELLS]; ///< _gScore gilt nur, wenn Stempel = _searchStamp (spart das Loeschen je Suche)
    uint16_t _closedStamp[PATH_GRID_X_CELLS][PATH_GRID_Y_CELLS];
    uint8_t _origin[PATH_GRID_X_CELLS][PATH_GRID_Y_CELLS]; ///< Richtung, aus der die Zelle erreicht wurde
    HeapEntry _heap[PATH_HEAP_CAPACITY];
    uint8_t _path[PATH_MAX_CELLS][2];

    uint16_t _heapSize = 0;
    uint16_t _heapPeak = 0;
    bool _heapOverflow = false;
    uint16_t _searchStamp = 1;
    uint16_t _pathLength = 0;
    uint8_t _targetX = 0;
    uint8_t _targetY = 0;

    uint16_t _targetDistance(uint8_t x, uint8_t y) const;
    uint16_t _knownG(uint8_t x, uint8_t y) const;
    void _setKnownG(uint8_t x, uint8_t y, uint16_t g);
    bool _isClosed(uint8_t x, uint8_t y) const;
    void _setClosed(uint8_t x, uint8_t y);
    void _resetSearch();
    void _placeMask(const uint8_t (&mask)[PATH_MASK_CELLS][PATH_MASK_CELLS], uint8_t centerX, uint8_t centerY);
    void _explore(const Node &current);
    void _pushHeap(uint8_t x, uint8_t y, uint16_t g, uint32_t f);
    bool _popHeap(Node &next);
    bool _buildPath(Node current, uint8_t startX, uint8_t startY);
};
