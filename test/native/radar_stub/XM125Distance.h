#pragma once
// Simulierter Sensor: Messung dauert g_dur[id] Mikrosekunden; jede Statusabfrage (isMeasuring) zaehlt als Buszugriff.
#include <Arduino.h>
#include <Wire.h>
enum
{
    XM125D_PROFILE3,
    XM125D_THRESHOLD_CFAR,
    XM125D_SORT_FARTHEST,
    XM125D_REFLECTOR_PLANAR
};
struct XM125DistanceConfig
{
    uint32_t startMm, endMm, maxProfile, thresholdMethod, thresholdSensitivity, signalQuality, peakSorting, reflectorShape;
    bool closeRangeLeakageCancellation;
};
struct XM125DistanceResult
{
    uint8_t numDistances;
    uint16_t peakDistanceRaw[1];
    int32_t peakStrength[1];
};
extern long g_busReads;
extern uint32_t g_dur[4];
extern uint32_t g_finishUs[4];
extern int g_nextId;
class XM125Distance
{
public:
    XM125DistanceConfig config{};
    int id;
    uint32_t endUs = 0;
    bool measuring = false;
    XM125DistanceResult res{1, {1234}, {-9000}};
    XM125Distance(uint8_t = 0, int8_t = -1, int8_t = -1) { id = g_nextId++ % 4; }
    bool begin(TwoWire &) { return true; }
    void setMeasureTimeout(uint32_t) {}
    void setMaxPeaksToFetch(uint8_t) {}
    bool applyConfigAndCalibrate() { return true; }
    void configure(uint8_t, int8_t, int8_t) {}
    uint16_t getLastFailedConfigRegister() { return 0xFFFF; }
    bool startMeasurement()
    {
        measuring = true;
        endUs = g_us + g_dur[id];
        g_finishUs[id] = endUs;
        return true;
    }
    bool isMeasuring()
    {
        if (!measuring)
            return false;
        g_busReads++;
        if (g_us >= endUs)
            measuring = false;
        return measuring;
    }
    bool hasResult() { return true; }
    const XM125DistanceResult &getLastResult() const { return res; }
};
