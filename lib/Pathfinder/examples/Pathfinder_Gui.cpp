// Pathfinder_Gui -- Wegsuchen mit zufaelligen Start-/Zielzellen laufend an die PC-Oberflaeche senden.
//
// Gegenstelle: tools/pathfinder_gui/pathfinder_viewer.py (zeigt Karte, Weg, Dauer).
// Als src/main.cpp in ein PlatformIO-Projekt mit lib/Pathfinder und lib/include kopieren.
#include <Arduino.h>
#include <Pathfinder.h>
#include <PathfinderTelemetry.h>

static constexpr uint16_t FIELD_FRAME_INTERVAL = 1; ///< jede n-te Suche auch die ganze Karte senden (90 x 120 = 10800 Byte)

static Pathfinder AStar; // ~130 kB: global anlegen, nie auf dem Stack
static uint32_t iteration = 0;

void setup()
{
    Serial.begin(115200);
    AStar.begin();
    randomSeed(analogRead(A0));
}

void loop()
{
    const uint8_t startX = random(PATH_GRID_X_CELLS), startY = random(PATH_GRID_Y_CELLS);
    const uint8_t targetX = random(PATH_GRID_X_CELLS), targetY = random(PATH_GRID_Y_CELLS);
    const uint8_t ballX = random(PATH_GRID_X_CELLS), ballY = random(PATH_GRID_Y_CELLS);

    const uint32_t t0 = millis();
    const bool found = AStar.findPath(startX, startY, targetX, targetY, ballX, ballY, PATH_NO_OBSTACLE, PATH_NO_OBSTACLE);
    const uint32_t duration = millis() - t0;

    PathfinderTelemetry::Meta meta;
    meta.iteration = iteration;
    meta.durationMs = duration;
    meta.pathLength = AStar.givePathLength();
    meta.startX = startX;
    meta.startY = startY;
    meta.targetX = targetX;
    meta.targetY = targetY;
    meta.pathFound = found ? 1 : 0;

    PathfinderTelemetry::sendMeta(Serial, meta);
    if (iteration % FIELD_FRAME_INTERVAL == 0)
    {
        PathfinderTelemetry::sendField(Serial, AStar.giveWeights());
    }
    PathfinderTelemetry::sendPath(Serial, AStar.givePath(), AStar.givePathLength());

    iteration++;
    delay(200);
}
