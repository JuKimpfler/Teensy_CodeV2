// Pathfinder_Demo -- eine Wegsuche ausfuehren und den Zellpfad seriell ausgeben.
//
// Als src/main.cpp in ein PlatformIO-Projekt mit lib/Pathfinder und lib/include kopieren.
#include <Arduino.h>
#include <Pathfinder.h>

static Pathfinder AStar; // ~130 kB: global anlegen, nie auf dem Stack

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000)
    {
    }
    AStar.begin();
    Serial.println("Pathfinder Demo");
}

void loop()
{
    // Zellen, nicht cm: Raster PATH_GRID_X_CELLS x PATH_GRID_Y_CELLS
    const uint8_t startX = 0, startY = 0;
    const uint8_t targetX = PATH_GRID_X_CELLS - 1, targetY = PATH_GRID_Y_CELLS - 1;
    const uint8_t ballX = 45, ballY = 60;
    const uint8_t robotX = 30, robotY = 80;

    const uint32_t t0 = micros();
    const bool found = AStar.findPath(startX, startY, targetX, targetY, ballX, ballY, robotX, robotY);
    const uint32_t us = micros() - t0;

    if (found)
    {
        Serial.print("Weg gefunden, ");
        Serial.print(AStar.givePathLength());
        Serial.print(" Zellen in ");
        Serial.print(us);
        Serial.println(" us");

        uint8_t waypoints[16][2];
        const uint8_t n = AStar.giveWaypoints(waypoints, 16);
        for (uint8_t i = 0; i < n; i++)
        {
            Serial.print("  Wegpunkt ");
            Serial.print(i);
            Serial.print(": (");
            Serial.print(waypoints[i][0]);
            Serial.print(", ");
            Serial.print(waypoints[i][1]);
            Serial.println(")");
        }
    }
    else
    {
        Serial.println("Kein Weg gefunden.");
    }

    delay(5000);
}
