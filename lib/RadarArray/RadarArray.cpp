/**
 * RadarArray.cpp  —  Implementierung, siehe RadarArray.h fuer die Doku.
 */

#include "RadarArray.h"

RadarArray::RadarArray()
{
    // Abstimmung auf eine gerade Holzwand bis 2,5 m, Werte siehe params.h.
    config.startMm             = RADAR_START_MM;
    config.endMm               = RADAR_END_MM;   // > 2500, damit die Wand nicht auf der Bereichskante liegt
    config.maxProfile          = XM125D_PROFILE5;            // laengste Pulse = groesste Reichweite
    config.thresholdMethod     = XM125D_THRESHOLD_CFAR;      // adaptiv, unabhaengig von der Umgebung
    config.thresholdSensitivity = RADAR_THRESHOLD_SENSITIVITY;
    config.signalQuality       = RADAR_SIGNAL_QUALITY;
    config.peakSorting         = XM125D_SORT_STRONGEST;
    // PLANAR: bei einer flaechigen Wand faellt die Echo-Amplitude langsamer mit der
    // Distanz als bei einem punktfoermigen Reflektor. Ohne diese Angabe setzt der
    // Detektor die Schwelle am Bereichsende zu hoch an und die Wand faellt raus.
    config.reflectorShape      = XM125D_REFLECTOR_PLANAR;
    config.closeRangeLeakageCancellation = false; // sauberer Nahbereich ab startMm
}

/* -------------- System Worker -------------- */

uint8_t RadarArray::begin(TwoWire &wire)
{
    _wire = &wire;

    uint8_t ok = 0;
    for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
    {
        _sensors[i].configure(RADAR_SENSOR_ADDRS[i], RADAR_WAKE_PIN, RADAR_INT_PIN);
        _ready[i]       = _setupSens(i);
        _lastSetupMs[i] = millis();
        if (_ready[i])
            ok++;
    }

    _startRound(); // erste Messung laeuft schon, waehrend der Rest hochfaehrt
    return ok;
}

void RadarArray::update(void)
{
    // Nicht blockierend: hier wird nie auf ein Messergebnis gewartet.
    // Je Aufruf wird bei jedem laufenden Sensor genau einmal nachgesehen
    // (isMeasuring() = hoechstens ein Registerlesezugriff). Sobald die Runde
    // komplett ist, startet sofort die naechste -- bei allen Sensoren
    // gleichzeitig, siehe _startRound().
    bool roundRunning = false;

    for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
    {
        if (!_pending[i])
            continue;

        if (_sensors[i].isMeasuring())
        {
            roundRunning = true;
            continue;
        }

        _pending[i] = false;
        if (_sensors[i].hasResult())
            _takeResult(i);
        else
            _measureFailed(i);
    }

    if (!roundRunning)
        _startRound();
}

void RadarArray::calib(void)
{
    for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
    {
        _sensors[i].config = config;
        _ready[i] = _sensors[i].applyConfigAndCalibrate();
        _data[i].isCon = _ready[i];
        _pending[i] = false;
    }

    _startRound();
}

/* -------------- Intern -------------- */

bool RadarArray::_setupSens(uint8_t idx)
{
    if (idx >= RADAR_NUM_SENSORS)
        return false;

    XM125Distance &radar = _sensors[idx];

    _data[idx].isCon = false;

    if (!radar.begin(*_wire))
        return false;

    // Messbetrieb: nie auf ein Ergebnis warten, und pro Messung nur die Peaks
    // holen, die auch ausgewertet werden (jeder Peak = zwei Registerlesezugriffe).
    radar.setMeasureTimeout(RADAR_MEASURE_TIMEOUT_MS);
    radar.setMaxPeaksToFetch(RADAR_PEAKS_TO_FETCH);

    radar.config = config;

    if (!radar.applyConfigAndCalibrate())
    {
        uint16_t failedReg = radar.getLastFailedConfigRegister();
        if (failedReg != 0xFFFF)
        { /*-> I2C-Schreibfehler bei Register 0xfailedReg*/
        }
        return false;
    }

    _fails[idx]      = 0;
    _data[idx].isCon = true;
    return true;
}

void RadarArray::_startRound(void)
{
    // Alle Sensoren zusammen anstossen: startMeasurement() schreibt nur das
    // Kommando (< 1 ms), sie messen also praktisch zeitgleich und die Messdauer
    // faellt einmal fuer alle an statt einmal pro Sensor.
    for (uint8_t i = 0; i < RADAR_NUM_SENSORS; i++)
    {
        if (!_ready[i])
        {
            _retrySetup(i);
            continue;
        }

        _pending[i] = _sensors[i].startMeasurement();
        if (!_pending[i])
            _measureFailed(i);
    }
}

void RadarArray::_takeResult(uint8_t idx)
{
    _fails[idx] = 0;

    // Peak 0: bei peakSorting = CLOSEST das naechstgelegene Objekt.
    // Peaks == 0 (keine Wand erkannt) liefert Dist/Strenght = 0, damit kein
    // alter Wert stehen bleibt und "reagiert nicht" sichtbar wird.
    const XM125DistanceResult &res = _sensors[idx].getLastResult();
    _data[idx].Peaks    = res.numDistances;
    _data[idx].Dist     = (res.numDistances > 0) ? res.peakDistanceRaw[0] : 0;
    _data[idx].Strenght = (res.numDistances > 0) ? res.peakStrength[0] : 0;
    _data[idx].isCon    = true;
    _data[idx].Time     = millis(); // Zeitstempel fuer die Altersbewertung, siehe giveAge()
}

void RadarArray::_measureFailed(uint8_t idx)
{
    _fails[idx]++;
    if (_fails[idx] < RADAR_MAX_CONSEC_FAILURES)
        return;

    // Sensor gilt als tot. Hier NICHT neu einrichten -- _setupSens() blockiert
    // bis zu ~2 s. Das uebernimmt _retrySetup() gedrosselt beim naechsten
    // Rundenstart.
    _fails[idx]      = 0;
    _ready[idx]      = false;
    _data[idx].isCon = false;
}

void RadarArray::_retrySetup(uint8_t idx)
{
    // _setupSens() blockiert (wakeUp-Timeout + Kalibrierung, bis ~2 s).
    // Ein abgezogener Sensor wuerde damit jeden Zyklus ausbremsen, deshalb
    // hoechstens alle RADAR_REINIT_INTERVAL_MS ein Versuch.
    if ((millis() - _lastSetupMs[idx]) < RADAR_REINIT_INTERVAL_MS)
        return;

    _lastSetupMs[idx] = millis();
    _ready[idx]       = _setupSens(idx);
}
