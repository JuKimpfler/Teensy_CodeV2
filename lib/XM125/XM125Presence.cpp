/**
 * XM125Presence.cpp  —  Implementierung des Presence-Detectors.
 * Siehe XM125Presence.h für Dokumentation.
 */

#include "XM125Presence.h"

bool XM125Presence::writeConfig()
{
    _lastFailedConfigRegister = 0xFFFF;

    const struct
    {
        uint16_t reg;
        uint32_t value;
    } regs[] = {
        {XM125P_REG_SWEEPS_PER_FRAME, config.sweepsPerFrame},
        {XM125P_REG_INTER_FRAME_PRESENCE_TIMEOUT, config.interFramePresenceTimeoutS},
        {XM125P_REG_INTER_PHASE_BOOST_ENABLED, config.interPhaseBoostEnabled ? 1u : 0u},
        {XM125P_REG_INTRA_DETECTION_ENABLED, config.intraDetectionEnabled ? 1u : 0u},
        {XM125P_REG_INTER_DETECTION_ENABLED, config.interDetectionEnabled ? 1u : 0u},
        {XM125P_REG_FRAME_RATE, config.frameRateMilliHz},
        {XM125P_REG_INTRA_DETECTION_THRESHOLD, config.intraDetectionThreshold},
        {XM125P_REG_INTER_DETECTION_THRESHOLD, config.interDetectionThreshold},
        {XM125P_REG_INTER_FRAME_DEVIATION_TC, config.interFrameDeviationTimeConstMs},
        {XM125P_REG_INTER_FRAME_FAST_CUTOFF, config.interFrameFastCutoffMilliHz},
        {XM125P_REG_INTER_FRAME_SLOW_CUTOFF, config.interFrameSlowCutoffMilliHz},
        {XM125P_REG_INTRA_FRAME_TIME_CONST, config.intraFrameTimeConstMs},
        {XM125P_REG_INTRA_OUTPUT_TIME_CONST, config.intraOutputTimeConstMs},
        {XM125P_REG_INTER_OUTPUT_TIME_CONST, config.interOutputTimeConstMs},
        {XM125P_REG_AUTO_PROFILE_ENABLED, config.autoProfileEnabled ? 1u : 0u},
        {XM125P_REG_AUTO_STEP_LENGTH_ENABLED, config.autoStepLengthEnabled ? 1u : 0u},
        {XM125P_REG_MANUAL_PROFILE, config.manualProfile},
        {XM125P_REG_MANUAL_STEP_LENGTH, config.manualStepLength},
        {XM125P_REG_START, config.startMm},
        {XM125P_REG_END, config.endMm},
        {XM125P_REG_RESET_FILTERS_ON_PREPARE, config.resetFiltersOnPrepare ? 1u : 0u},
        {XM125P_REG_HWAAS, config.hwaas},
        {XM125P_REG_DETECTION_ON_GPIO, config.detectionOnGpio ? 1u : 0u},
    };

    for (const auto &r : regs)
    {
        if (!_writeReg(r.reg, r.value))
        {
            _lastFailedConfigRegister = r.reg;
            return false;
        }
        delay(1); // kurze Verarbeitungspause zwischen Register-Writes
    }
    return true;
}

bool XM125Presence::applyConfiguration(uint32_t timeoutMs)
{
    if (!writeConfig())
        return false;
    return _executeCommand(XM125P_CMD_APPLY_CONFIGURATION, timeoutMs);
}

bool XM125Presence::start(uint32_t timeoutMs)
{
    return _executeCommand(XM125P_CMD_START_DETECTOR, timeoutMs);
}

bool XM125Presence::stop(uint32_t timeoutMs)
{
    return _executeCommand(XM125P_CMD_STOP_DETECTOR, timeoutMs);
}

bool XM125Presence::update()
{
    uint32_t result = 0;
    if (!_readReg(XM125P_REG_PRESENCE_RESULT, result))
        return false;

    _presenceDetected = (result & XM125P_RESULT_PRESENCE_DETECTED) != 0;
    _presenceDetectedSticky = (result & XM125P_RESULT_PRESENCE_DETECTED_STICKY) != 0;
    _detectorError = (result & XM125P_RESULT_DETECTOR_ERROR) != 0;
    _temperature = static_cast<int16_t>(result >> XM125P_RESULT_TEMPERATURE_SHIFT);

    uint32_t v = 0;
    if (_readReg(XM125P_REG_PRESENCE_DISTANCE, v))
        _presenceDistanceMm = v;
    if (_readReg(XM125P_REG_INTRA_PRESENCE_SCORE, v))
        _intraScore = v;
    if (_readReg(XM125P_REG_INTER_PRESENCE_SCORE, v))
        _interScore = v;

    return true;
}
