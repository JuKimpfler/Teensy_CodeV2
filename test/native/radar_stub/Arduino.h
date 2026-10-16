#pragma once
// Zeit steuerbar (g_us), damit RadarArray auf dem Rechner mit simulierten Sensoren laufen kann.
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
extern uint32_t g_us;
inline uint32_t micros() { return g_us; }
inline uint32_t millis() { return g_us / 1000; }
inline void delay(uint32_t) {}
#define constrain(a, l, h) ((a) < (l) ? (l) : ((a) > (h) ? (h) : (a)))
#define DEG_TO_RAD 0.017453292519943295
using std::abs;
