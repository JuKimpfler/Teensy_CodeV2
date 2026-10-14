/**
 * XM125Distance.cpp  —  Implementierung des Distance-Detectors.
 * Siehe XM125Distance.h für Dokumentation.
 *
 * Der Messbetrieb ist eine nicht blockierende Zustandsmaschine:
 *
 *   IDLE ──startMeasurement()──> RUNNING ──BUSY weg, Result gelesen──> SETTLING
 *                                   │                                     │
 *                                   │ Fehler/Timeout          Pause um,   │
 *                                   ▼                         Peaks da    ▼
 *                                FAILED <────Fehler────────────────────  READY
 *
 * Weitergetrieben wird sie ausschliesslich von isMeasuring(); ein Aufruf macht
 * genau einen Schritt und hoechstens einen Buszugriff. Nirgends steht ein
 * delay() -- die beiden noetigen Pausen (BUSY-Anlauf, Peak-Register) werden
 * ueber micros()-Zeitstempel ausgesessen, waehrenddessen kann der Aufrufer
 * andere Sensoren bedienen.
 *
 * Die Einrichtung davor blockiert dagegen bewusst: writeConfig(),
 * applyConfigAndCalibrate() und hardCalibration() (Neustart + komplette
 * Konfiguration) laufen einmal beim Hochfahren bzw. in Kalibrierskripten.
 */

#include "XM125Distance.h"

bool XM125Distance::writeConfig()
{
    _lastFailedConfigRegister = 0xFFFF;

    const struct
    {
        uint16_t reg;
        uint32_t value;
    } regs[] = {
        {XM125D_REG_START, config.startMm},
        {XM125D_REG_END, config.endMm},
        {XM125D_REG_MAX_STEP_LENGTH, config.maxStepLength},
        {XM125D_REG_CLOSE_RANGE_LEAKAGE_CANCEL, config.closeRangeLeakageCancellation ? 1u : 0u},
        {XM125D_REG_SIGNAL_QUALITY, config.signalQuality},
        {XM125D_REG_MAX_PROFILE, config.maxProfile},
        {XM125D_REG_THRESHOLD_METHOD, config.thresholdMethod},
        {XM125D_REG_PEAK_SORTING, config.peakSorting},
        {XM125D_REG_NUM_FRAMES_RECORDED_THRESHOLD, config.numFramesRecordedThreshold},
        {XM125D_REG_FIXED_AMPLITUDE_THRESHOLD, config.fixedAmplitudeThreshold},
        {XM125D_REG_THRESHOLD_SENSITIVITY, config.thresholdSensitivity},
        {XM125D_REG_REFLECTOR_SHAPE, config.reflectorShape},
        {XM125D_REG_FIXED_STRENGTH_THRESHOLD, config.fixedStrengthThreshold},
        {XM125D_REG_MEASURE_ON_WAKEUP, config.measureOnWakeup ? 1u : 0u},
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

bool XM125Distance::applyConfigAndCalibrate(uint32_t timeoutMs)
{
    if (!writeConfig())
        return false;
    return _executeCommand(XM125D_CMD_APPLY_CONFIG_AND_CALIBRATE, timeoutMs);
}

bool XM125Distance::applyConfiguration(uint32_t timeoutMs)
{
    if (!writeConfig())
        return false;
    return _executeCommand(XM125D_CMD_APPLY_CONFIGURATION, timeoutMs);
}

bool XM125Distance::calibrate(uint32_t timeoutMs)
{
    return _executeCommand(XM125D_CMD_CALIBRATE, timeoutMs);
}

bool XM125Distance::recalibrate(uint32_t timeoutMs)
{
    return _executeCommand(XM125D_CMD_RECALIBRATE, timeoutMs);
}

bool XM125Distance::hardCalibration(const XM125DistanceConfig &cfg,
                                    uint32_t timeoutMs,
                                    uint32_t resetTimeoutMs)
{
    // Uebernehmen, bevor irgendetwas schiefgehen kann: `config` soll danach in
    // jedem Fall den Stand zeigen, der zuletzt gewollt war -- sonst wuerde ein
    // spaeteres applyConfiguration() heimlich die alten Werte schreiben.
    config = cfg;

    // Der Neustart macht jede laufende Messung hinfaellig: das Kommando ist
    // mit dem alten Modulzustand verschwunden, ein spaeteres isMeasuring()
    // wuerde auf ein Ergebnis warten, das nie kommt.
    _state = XM125MeasureState::IDLE;
    _result.valid = false;
    _peaksToRead = 0;

    _lastFailedConfigRegister = 0xFFFF; // scheitert der Neustart, lag es nicht am Schreiben
    if (!resetModule(resetTimeoutMs))
        return false;

    // Nach dem Neustart stehen alle Register auf Werksvorgabe -- writeConfig()
    // in applyConfigAndCalibrate() schreibt ohnehin jedes einzelne davon neu.
    return applyConfigAndCalibrate(timeoutMs);
}

bool XM125Distance::hardCalibration(uint32_t startMm,
                                    uint32_t endMm,
                                    uint32_t maxStepLength,
                                    bool closeRangeLeakageCancellation,
                                    uint32_t signalQuality,
                                    uint32_t maxProfile,
                                    uint32_t thresholdMethod,
                                    uint32_t peakSorting,
                                    uint32_t numFramesRecordedThreshold,
                                    uint32_t fixedAmplitudeThreshold,
                                    uint32_t thresholdSensitivity,
                                    uint32_t reflectorShape,
                                    uint32_t fixedStrengthThreshold,
                                    bool measureOnWakeup)
{
    XM125DistanceConfig cfg; // Defaults stehen in der Struktur, hier wird alles gesetzt
    cfg.startMm = startMm;
    cfg.endMm = endMm;
    cfg.maxStepLength = maxStepLength;
    cfg.closeRangeLeakageCancellation = closeRangeLeakageCancellation;
    cfg.signalQuality = signalQuality;
    cfg.maxProfile = maxProfile;
    cfg.thresholdMethod = thresholdMethod;
    cfg.peakSorting = peakSorting;
    cfg.numFramesRecordedThreshold = numFramesRecordedThreshold;
    cfg.fixedAmplitudeThreshold = fixedAmplitudeThreshold;
    cfg.thresholdSensitivity = thresholdSensitivity;
    cfg.reflectorShape = reflectorShape;
    cfg.fixedStrengthThreshold = fixedStrengthThreshold;
    cfg.measureOnWakeup = measureOnWakeup;

    return hardCalibration(cfg);
}

/* ── Messbetrieb ──────────────────────────────────────────────────────────── */

bool XM125Distance::startMeasurement()
{
    _startUs = micros();

    if (!_writeReg(XM125_REG_COMMAND, XM125D_CMD_MEASURE_DISTANCE))
    {
        // Siehe Kommentar in XM125Base::_executeCommand: ein gemeldeter
        // Schreibfehler kann durch Clock-Stretching der internen MCU direkt
        // nach Kommando-Empfang entstehen, obwohl das Kommando ankam. Sofort
        // gegenpruefen, ob der Sensor bereits BUSY ist.
        uint32_t probe = 0;
        if (!_readReg(XM125_REG_DETECTOR_STATUS, probe) || !statusBusy(probe))
        {
            _fail();
            return false;
        }
    }

    _peaksToRead = 0;
    _state = XM125MeasureState::RUNNING;
    return true;
}

bool XM125Distance::isMeasuring()
{
    switch (_state)
    {
    case XM125MeasureState::RUNNING:
        _pollRunning();
        break;
    case XM125MeasureState::SETTLING:
        _pollSettling();
        break;
    default:
        break; // IDLE / READY / FAILED: nichts zu tun, kein Buszugriff
    }

    return (_state == XM125MeasureState::RUNNING) ||
           (_state == XM125MeasureState::SETTLING);
}

void XM125Distance::_pollRunning()
{
    // Die interne MCU braucht einen Moment, bis sie BUSY setzt. Wer frueher
    // fragt, sieht BUSY=0 und wuerde das Ergebnis der vorherigen Messung als
    // neu ausgeben -- deshalb die erste Abfrage aussetzen (ohne Buszugriff).
    uint32_t elapsedUs = micros() - _startUs;
    if (elapsedUs < XM125D_BUSY_SETTLE_US)
        return;

    uint32_t status = 0;
    if (!_readReg(XM125_REG_DETECTOR_STATUS, status))
    {
        _fail();
        return;
    }

    if (statusBusy(status))
    {
        if (elapsedUs > _measureTimeoutMs * 1000UL)
            _fail(); // Sensor antwortet nicht mehr
        return;
    }
    if (statusError(status))
    {
        _fail();
        return;
    }

    // BUSY ist zurueckgegangen -> das Ergebnisregister ist ab jetzt gueltig.
    if (!_readResultRegister())
    {
        _fail();
        return;
    }

    if (_peaksToRead == 0)
    {
        _finish();
        return;
    } // nichts abzuholen -> direkt fertig

    _settleUs = micros();
    _state = XM125MeasureState::SETTLING;
}

void XM125Distance::_pollSettling()
{
    // Reines Abwarten der Peak-Register-Pause: kein Buszugriff, kein delay().
    if ((micros() - _settleUs) < XM125D_PEAK_SETTLE_US)
        return;

    if (!_readPeaks())
    {
        _fail();
        return;
    }
    _finish();
}

bool XM125Distance::_readResultRegister()
{
    uint32_t result = 0;
    if (!_readReg(XM125D_REG_DISTANCE_RESULT, result))
        return false;

    _result.numDistances = static_cast<uint8_t>(result & XM125D_RESULT_NUM_DISTANCES_MASK);
    _result.nearStartEdge = (result & XM125D_RESULT_NEAR_START_EDGE) != 0;
    _result.calibrationNeeded = (result & XM125D_RESULT_CALIBRATION_NEEDED) != 0;
    _result.measureError = (result & XM125D_RESULT_MEASURE_ERROR) != 0;
    _result.temperature = static_cast<int16_t>(result >> XM125D_RESULT_TEMPERATURE_SHIFT);

    if (_result.numDistances > XM125D_MAX_PEAKS)
        _result.numDistances = XM125D_MAX_PEAKS;

    _peaksToRead = (_result.numDistances < _maxPeaksToFetch) ? _result.numDistances
                                                             : _maxPeaksToFetch;

    // Peaks, die diese Runde nicht geholt werden (weil es keine gibt oder weil
    // setMaxPeaksToFetch() sie ausblendet), auf 0 setzen -- sonst stuenden dort
    // noch die Werte der vorherigen Messung.
    for (uint8_t i = _peaksToRead; i < XM125D_MAX_PEAKS; i++)
    {
        _result.peakDistanceRaw[i] = 0;
        _result.peakDistanceReg[i] = 0;
        _result.peakStrength[i] = 0;
    }
    return true;
}

bool XM125Distance::_readPeaks()
{
    for (uint8_t i = 0; i < _peaksToRead; i++)
    {
        uint32_t dist = 0, strength = 0;
        if (!_readReg(XM125D_REG_PEAK0_DISTANCE + i, dist))
            return false;
        if (!_readReg(XM125D_REG_PEAK0_STRENGTH + i, strength))
            return false;

        // Diese (nicht-offizielle/angepasste) Firmware kodiert die Peak-Distanz
        // abweichend vom Acconeer-Standard-Registerprotokoll: nur das oberste
        // Byte traegt den Wert. Der genaue Skalierungsfaktor (mm pro Count,
        // ggf. mit Offset oder relativ zu config.startMm/endMm) ist NICHT
        // zuverlaessig bekannt -- zwei bisherige Annahmen dazu waren falsch.
        // Bis zur Kalibrierung mit bekannten Distanzen wird hier der rohe
        // Byte-Wert unveraendert als "Counts" gespeichert, siehe
        // getPeakDistanceRaw() / getPeakDistanceMm() in XM125Distance.h.
        _result.peakDistanceReg[i] = dist; // ungekuerzt, fuer die Kalibriermessung
        _result.peakDistanceRaw[i] = static_cast<uint16_t>(dist);
        _result.peakStrength[i] = static_cast<int32_t>(strength);
    }
    return true;
}

void XM125Distance::_finish()
{
    _result.valid = !_result.measureError;
    _result.timestampMs = millis();
    _result.durationMs = static_cast<uint16_t>((micros() - _startUs) / 1000UL);
    _state = XM125MeasureState::READY;
}

void XM125Distance::_fail()
{
    // Zahlenwerte bleiben stehen (letzter guter Stand), aber sie sind ab jetzt
    // als nicht mehr aktuell markiert -- siehe XM125DistanceResult::valid.
    _result.valid = false;
    _state = XM125MeasureState::FAILED;
}

bool XM125Distance::measureBlocking(uint32_t timeoutMs)
{
    const uint32_t savedTimeout = _measureTimeoutMs;
    _measureTimeoutMs = timeoutMs;

    bool started = startMeasurement();
    if (started)
    {
        while (isMeasuring())
        {
            yield(); // Bus wird in isMeasuring() ohnehin nur einmal je Runde belegt
        }
    }

    _measureTimeoutMs = savedTimeout;
    return started && _result.valid;
}

/* ── Ergebniszugriffe ─────────────────────────────────────────────────────── */

uint8_t XM125Distance::getPeakDistanceRaw(uint8_t idx) const
{
    if (idx >= _result.numDistances)
        return 0;
    return _result.peakDistanceRaw[idx];
}

uint32_t XM125Distance::getPeakDistanceReg(uint8_t idx) const
{
    if (idx >= _result.numDistances)
        return 0;
    return _result.peakDistanceReg[idx];
}

uint32_t XM125Distance::getPeakDistanceMm(uint8_t idx) const
{
    // Unkalibriert (siehe Warnung im Header): Platzhalter 1:1, bis der
    // echte Skalierungsfaktor per Kalibriermessung bestimmt ist.
    return getPeakDistanceRaw(idx);
}

int32_t XM125Distance::getPeakStrength(uint8_t idx) const
{
    if (idx >= _result.numDistances)
        return 0;
    return _result.peakStrength[idx];
}
