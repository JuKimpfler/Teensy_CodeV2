#ifndef PATHFINDER_H
#define PATHFINDER_H

#include <Arduino.h>

namespace pathfinder
{
constexpr uint8_t kFieldWidth = 90;
constexpr uint8_t kFieldHeight = 120;
constexpr uint16_t kPathMaxLength = 600;

bool findPath(uint8_t startX, uint8_t startY,
			  uint8_t targetX, uint8_t targetY,
			  uint8_t ballX, uint8_t ballY,
			  uint8_t robotX, uint8_t robotY,
			  uint8_t (&path)[kPathMaxLength][2],
			  uint16_t &pathLength);
}

#endif

