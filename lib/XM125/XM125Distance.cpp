/**
 * XM125Distance.cpp  —  Implementierung des Distance-Detectors.
 * Siehe XM125Distance.h für Dokumentation.
 */

#include "XM125Distance.h"

bool XM125Distance::writeConfig() {
    _lastFailedConfigRegister = 0xFFFF;

    const struct { uint16_t reg; uint32_t value; } regs[] = {
        { XM125D_REG_START,                         config.startMm },
        { XM125D_REG_END,                           config.endMm },
        { XM125D_REG_MAX_STEP_LENGTH,                config.maxStepLength },
        { XM125D_REG_CLOSE_RANGE_LEAKAGE_CANCEL,     config.closeRangeLeakageCancellation ? 1u : 0u },
        { XM125D_REG_SIGNAL_QUALITY,                 config.signalQuality },
        { XM125D_REG_MAX_PROFILE,                    config.maxProfile },
        { XM125D_REG_THRESHOLD_METHOD,               config.thresholdMethod },
        { XM125D_REG_PEAK_SORTING,                   config.peakSorting },
        { XM125D_REG_NUM_FRAMES_RECORDED_THRESHOLD,  config.numFramesRecordedThreshold },
        { XM125D_REG_FIXED_AMPLITUDE_THRESHOLD,      config.fixedAmplitudeThreshold },
        { XM125D_REG_THRESHOLD_SENSITIVITY,          config.thresholdSensitivity },
        { XM125D_REG_REFLECTOR_SHAPE,                config.reflectorShape },
        { XM125D_REG_FIXED_STRENGTH_THRESHOLD,       config.fixedStrengthThreshold },
        { XM125D_REG_MEASURE_ON_WAKEUP,              config.measureOnWakeup ? 1u : 0u },
    };

    for (const auto& r : regs) {
        if (!_writeReg(r.reg, r.value)) {
            _lastFailedConfigRegister = r.reg;
            return false;
        }
        delay(1); // kurze Verarbeitungspause zwischen Register-Writes
    }
    return true;
}

bool XM125Distance::applyConfigAndCalibrate(uint32_t timeoutMs) {
    if (!writeConfig()) return false;
    return _executeCommand(XM125D_CMD_APPLY_CONFIG_AND_CALIBRATE, timeoutMs);
}

bool XM125Distance::applyConfiguration(uint32_t timeoutMs) {
    if (!writeConfig()) return false;
    return _executeCommand(XM125D_CMD_APPLY_CONFIGURATION, timeoutMs);
}

bool XM125Distance::calibrate(uint32_t timeoutMs) {
    return _executeCommand(XM125D_CMD_CALIBRATE, timeoutMs);
}

bool XM125Distance::recalibrate(uint32_t timeoutMs) {
    return _executeCommand(XM125D_CMD_RECALIBRATE, timeoutMs);
}

bool XM125Distance::measure(uint32_t timeoutMs) {
    if (!_writeReg(XM125_REG_COMMAND, XM125D_CMD_MEASURE_DISTANCE)) return false;

    uint32_t status = 0;
    if (!_waitNotBusy(timeoutMs, status)) return false;
    if (statusError(status)) return false;

    uint32_t result = 0;
    if (!_readReg(XM125D_REG_DISTANCE_RESULT, result)) return false;

    _numDistances      = static_cast<uint8_t>(result & XM125D_RESULT_NUM_DISTANCES_MASK);
    _nearStartEdge     = (result & XM125D_RESULT_NEAR_START_EDGE) != 0;
    _calibrationNeeded = (result & XM125D_RESULT_CALIBRATION_NEEDED) != 0;
    _measureError      = (result & XM125D_RESULT_MEASURE_ERROR) != 0;
    _temperature       = static_cast<int16_t>(result >> XM125D_RESULT_TEMPERATURE_SHIFT);

    if (_numDistances > XM125D_MAX_PEAKS) _numDistances = XM125D_MAX_PEAKS;

    for (uint8_t i = 0; i < _numDistances; i++) {
        uint32_t dist = 0, strength = 0;
        _readReg(XM125D_REG_PEAK0_DISTANCE + i, dist);
        _readReg(XM125D_REG_PEAK0_STRENGTH + i, strength);
        _peakDistanceMm[i] = dist;
        _peakStrength[i]   = static_cast<int32_t>(strength);
    }

    return !_measureError;
}

uint32_t XM125Distance::getPeakDistanceMm(uint8_t idx) const {
    if (idx >= _numDistances) return 0;
    return _peakDistanceMm[idx];
}

int32_t XM125Distance::getPeakStrength(uint8_t idx) const {
    if (idx >= _numDistances) return 0;
    return _peakStrength[idx];
}
