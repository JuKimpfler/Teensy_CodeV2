#pragma once

/**
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  XM125Distance.h  —  Acconeer XM125 Distance-Detector (I²C-Firmware)     ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  Registerkarte & Kommandos lt. Acconeer "I2C Distance Detector User      ║
 * ║  Guide". Alle Distanzangaben in Millimetern (Firmware rechnet intern     ║
 * ║  bereits von Metern um).                                                 ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * Beispiel-Verwendung:
 *   XM125Distance radar;
 *   radar.begin(Wire1);
 *   radar.config.startMm = 200;
 *   radar.config.endMm   = 4000;
 *   radar.applyConfigAndCalibrate();
 *   ...
 *   if (radar.measure()) {
 *     for (uint8_t i = 0; i < radar.getNumDistances(); i++)
 *       Serial.println(radar.getPeakDistanceMm(i));
 *   }
 */

#include "XM125.h"

// ── Applikationsspezifische Register ──────────────────────────────────────────
static constexpr uint16_t XM125D_REG_DISTANCE_RESULT   = 0x0010; ///< R: Ergebnis-Flags + Peak-Anzahl
static constexpr uint16_t XM125D_REG_PEAK0_DISTANCE     = 0x0011; ///< R: Peak0..9 Distanz, je +1 (siehe Hinweis unten)
static constexpr uint16_t XM125D_REG_PEAK0_STRENGTH     = 0x001B; ///< R: Peak0..9 Stärke,   je +1
static constexpr uint8_t  XM125D_MAX_PEAKS              = 10;

static constexpr uint16_t XM125D_REG_START                          = 0x0040; ///< R/W: Messbereich Start [mm] (Default 250)
static constexpr uint16_t XM125D_REG_END                            = 0x0041; ///< R/W: Messbereich Ende  [mm] (Default 3000)
static constexpr uint16_t XM125D_REG_MAX_STEP_LENGTH                = 0x0042; ///< R/W: 0 = automatisch (Default 0)
static constexpr uint16_t XM125D_REG_CLOSE_RANGE_LEAKAGE_CANCEL     = 0x0043; ///< R/W: bool (Default true)
static constexpr uint16_t XM125D_REG_SIGNAL_QUALITY                 = 0x0044; ///< R/W: 1000x-Einheit (Default 15000)
static constexpr uint16_t XM125D_REG_MAX_PROFILE                    = 0x0045; ///< R/W: PROFILE1..5   (Default PROFILE5)
static constexpr uint16_t XM125D_REG_THRESHOLD_METHOD               = 0x0046; ///< R/W: siehe ThresholdMethod (Default CFAR)
static constexpr uint16_t XM125D_REG_PEAK_SORTING                   = 0x0047; ///< R/W: siehe PeakSorting (Default STRONGEST)
static constexpr uint16_t XM125D_REG_NUM_FRAMES_RECORDED_THRESHOLD  = 0x0048; ///< R/W: (Default 100)
static constexpr uint16_t XM125D_REG_FIXED_AMPLITUDE_THRESHOLD      = 0x0049; ///< R/W: 1000x-Einheit (Default 100000)
static constexpr uint16_t XM125D_REG_THRESHOLD_SENSITIVITY          = 0x004A; ///< R/W: 0..1000, 1000x-Einheit (Default 500)
static constexpr uint16_t XM125D_REG_REFLECTOR_SHAPE                = 0x004B; ///< R/W: siehe ReflectorShape (Default GENERIC)
static constexpr uint16_t XM125D_REG_FIXED_STRENGTH_THRESHOLD       = 0x004C; ///< R/W: 1000x-Einheit (Default 0)
static constexpr uint16_t XM125D_REG_MEASURE_ON_WAKEUP              = 0x0080; ///< R/W: bool (Default false)

// ── Kommandos (0x0100) ────────────────────────────────────────────────────────
static constexpr uint32_t XM125D_CMD_APPLY_CONFIG_AND_CALIBRATE = 1;
static constexpr uint32_t XM125D_CMD_MEASURE_DISTANCE           = 2;
static constexpr uint32_t XM125D_CMD_APPLY_CONFIGURATION        = 3;
static constexpr uint32_t XM125D_CMD_CALIBRATE                  = 4;
static constexpr uint32_t XM125D_CMD_RECALIBRATE                = 5;

// ── Enums für Konfigurationsregister ──────────────────────────────────────────
enum XM125DMaxProfile : uint32_t {
    XM125D_PROFILE1 = 1, XM125D_PROFILE2 = 2, XM125D_PROFILE3 = 3,
    XM125D_PROFILE4 = 4, XM125D_PROFILE5 = 5,
};
enum XM125DThresholdMethod : uint32_t {
    XM125D_THRESHOLD_FIXED_AMPLITUDE = 1,
    XM125D_THRESHOLD_RECORDED        = 2,
    XM125D_THRESHOLD_CFAR            = 3,
    XM125D_THRESHOLD_FIXED_STRENGTH  = 4,
};
enum XM125DPeakSorting : uint32_t {
    XM125D_SORT_CLOSEST   = 1,
    XM125D_SORT_STRONGEST = 2,
};
enum XM125DReflectorShape : uint32_t {
    XM125D_REFLECTOR_GENERIC = 1,
    XM125D_REFLECTOR_PLANAR  = 2,
};

// ── Detector-Status Bits, spezifisch für Distance-Firmware (Reg 0x0003) ──────
static constexpr uint32_t XM125D_STATUS_RSS_REGISTER_OK   = (1UL << 0);
static constexpr uint32_t XM125D_STATUS_CONFIG_CREATE_OK  = (1UL << 1);
static constexpr uint32_t XM125D_STATUS_SENSOR_CREATE_OK  = (1UL << 2);
static constexpr uint32_t XM125D_STATUS_DETECTOR_CREATE_OK = (1UL << 3);
static constexpr uint32_t XM125D_STATUS_CONFIG_APPLY_ERROR    = (1UL << 23);
static constexpr uint32_t XM125D_STATUS_SENSOR_CALIBRATE_ERROR   = (1UL << 24);
static constexpr uint32_t XM125D_STATUS_DETECTOR_CALIBRATE_ERROR = (1UL << 25);
// XM125_STATUS_DETECTOR_ERROR (Bit 28) und XM125_STATUS_BUSY (Bit 31) siehe XM125.h

// ── Distance-Result Bitfelder (Reg 0x0010) ────────────────────────────────────
static constexpr uint32_t XM125D_RESULT_NUM_DISTANCES_MASK   = 0x0000000FUL; ///< Bit 0-3
static constexpr uint32_t XM125D_RESULT_NEAR_START_EDGE      = (1UL << 8);
static constexpr uint32_t XM125D_RESULT_CALIBRATION_NEEDED   = (1UL << 9);
static constexpr uint32_t XM125D_RESULT_MEASURE_ERROR        = (1UL << 10);
static constexpr uint32_t XM125D_RESULT_TEMPERATURE_SHIFT    = 16; ///< Bit 16-31

// ═════════════════════════════════════════════════════════════════════════════
//  DistanceConfig  —  Konfigurationswerte, Default lt. Acconeer-Datenblatt
// ═════════════════════════════════════════════════════════════════════════════
struct XM125DistanceConfig {
    uint32_t startMm                     = 250;
    uint32_t endMm                       = 3000;
    uint32_t maxStepLength                = 0;                        ///< 0 = automatisch
    bool     closeRangeLeakageCancellation = true;
    uint32_t signalQuality                = 15000;
    uint32_t maxProfile                   = XM125D_PROFILE5;
    uint32_t thresholdMethod              = XM125D_THRESHOLD_CFAR;
    uint32_t peakSorting                  = XM125D_SORT_STRONGEST;
    uint32_t numFramesRecordedThreshold    = 100;
    uint32_t fixedAmplitudeThreshold      = 100000;
    uint32_t thresholdSensitivity          = 500;
    uint32_t reflectorShape                = XM125D_REFLECTOR_GENERIC;
    uint32_t fixedStrengthThreshold       = 0;
    bool     measureOnWakeup              = false;
};

// ═════════════════════════════════════════════════════════════════════════════
//  XM125Distance  —  Distance-Detector-Applikation
// ═════════════════════════════════════════════════════════════════════════════
class XM125Distance : public XM125Base {
public:
    explicit XM125Distance(uint8_t i2cAddr = XM125_I2C_ADDR_DEFAULT, int8_t wakePin = -1, int8_t intPin = -1)
        : XM125Base(i2cAddr, wakePin, intPin) {}

    XM125DistanceConfig config; ///< Vor applyConfigAndCalibrate() anpassen

    /**
     * Schreibt alle Werte aus `config` in die Konfigurationsregister.
     * Bricht beim ersten fehlgeschlagenen Register-Write ab (fail-fast).
     * Bei Misserfolg liefert getLastFailedConfigRegister() die Register-
     * adresse, an der die I²C-Übertragung fehlgeschlagen ist.
     */
    bool writeConfig();

    /** Registeradresse des zuletzt fehlgeschlagenen writeConfig()-Schreibvorgangs, 0xFFFF = keiner. */
    uint16_t getLastFailedConfigRegister() const { return _lastFailedConfigRegister; }

    /** writeConfig() + Kommando APPLY_CONFIG_AND_CALIBRATE, blockiert bis fertig. */
    bool applyConfigAndCalibrate(uint32_t timeoutMs = XM125_DEFAULT_APPLY_TIMEOUT_MS);

    /** Nur anwenden, ohne Neukalibrierung (z.B. nach Neustart mit alter Kalibrierung). */
    bool applyConfiguration(uint32_t timeoutMs = XM125_DEFAULT_APPLY_TIMEOUT_MS);

    /** Sensor/Detektor neu kalibrieren (z.B. wenn CALIBRATION_NEEDED gesetzt ist). */
    bool calibrate(uint32_t timeoutMs = XM125_DEFAULT_APPLY_TIMEOUT_MS);
    bool recalibrate(uint32_t timeoutMs = XM125_DEFAULT_APPLY_TIMEOUT_MS);

    /**
     * Startet eine Messung, wartet bis fertig und liest das Ergebnisregister
     * sowie alle vorhandenen Peaks ein.
     * @return true wenn Messung ohne Fehler abgeschlossen wurde.
     */
    bool measure(uint32_t timeoutMs = XM125_DEFAULT_MEASURE_TIMEOUT_MS);

    // ── Zerlegte Messung fuer mehrere Sensoren parallel ───────────────────
    // measure() blockiert waehrend der gesamten Messung. Sollen mehrere
    // Sensoren gleichzeitig messen, stattdessen: bei allen startMeasurement(),
    // dann alle pollMeasurement() bis fertig, dann alle fetchResult(). Die
    // Wartezeit faellt so nur einmal an statt einmal pro Sensor.
    // Siehe systemlib::readRadarGroup().

    /** Sendet das Messkommando und kehrt sofort zurueck (wartet NICHT). */
    bool startMeasurement();

    /**
     * Fragt einmal ab, ob die laufende Messung fertig ist -- blockiert nicht.
     * @param done  wird true, sobald BUSY zurueckgegangen ist
     * @return false bei I2C-Fehler oder gesetztem DETECTOR_ERROR
     */
    bool pollMeasurement(bool& done);

    /**
     * Liest Ergebnisregister und Peaks der abgeschlossenen Messung ein.
     * Erst aufrufen, wenn pollMeasurement() `done` gemeldet hat.
     */
    bool fetchResult();

    // ── Zugriff auf zuletzt eingelesenes Ergebnis (nach measure()) ────────
    uint8_t  getNumDistances()      const { return _numDistances; }
    bool     getNearStartEdge()     const { return _nearStartEdge; }
    bool     getCalibrationNeeded() const { return _calibrationNeeded; }
    bool     getMeasureError()      const { return _measureError; }
    int16_t  getTemperature()       const { return _temperature; } ///< °C, geringe Absolutgenauigkeit lt. Datenblatt

    /**
     * ACHTUNG - UNKALIBRIERT: Diese (nicht-offizielle/angepasste) Firmware
     * kodiert die Peak-Distanz abweichend vom Acconeer-Standardprotokoll:
     * nur 1 Byte traegt den Wert (0-255 "Counts"). Der genaue Umrechnungs-
     * faktor in eine echte Distanz (mm pro Count, mit/ohne Offset, evtl.
     * relativ zu config.startMm/endMm) ist NICHT zuverlaessig bekannt und
     * muss per Kalibriermessung (Objekt auf exakt bekannter Distanz)
     * bestimmt werden, bevor getPeakDistanceMm() sinnvolle Werte liefert.
     * Bis dahin: nutze getPeakDistanceRaw() fuer die Kalibrierung.
     */
    uint8_t  getPeakDistanceRaw(uint8_t idx) const;
    /** Siehe Warnung bei getPeakDistanceRaw() -- aktuell 1:1 unkalibrierter Rohwert, NICHT echte mm! */
    uint32_t getPeakDistanceMm(uint8_t idx) const;
    /**
     * Ungefiltertes 32-bit-Wort des Peak-Distanz-Registers, genau so wie es vom
     * Sensor gelesen wurde. getPeakDistanceRaw() gibt davon nur das oberste Byte
     * zurueck -- welche der 32 Bit die Distanz wirklich tragen, ist bisher nicht
     * geklaert (siehe Warnung oben). Fuer die Kalibrierung deshalb dieses Wort
     * bei bekannten Distanzen protokollieren, nicht den 8-bit-Wert.
     * Genutzt von lib/XM125/examples/XM125_WallCalibration.cpp.
     */
    uint32_t getPeakDistanceReg(uint8_t idx) const;
    /** Rohe Signalstärke des Peaks idx (1000x-Einheit lt. Datenblatt, RSS-intern). */
    int32_t  getPeakStrength(uint8_t idx) const;

private:
    uint16_t _lastFailedConfigRegister = 0xFFFF;

    uint8_t  _numDistances      = 0;
    bool     _nearStartEdge     = false;
    bool     _calibrationNeeded = false;
    bool     _measureError      = false;
    int16_t  _temperature       = 0;
    uint8_t  _peakDistanceRaw[XM125D_MAX_PEAKS] = {};
    uint32_t _peakDistanceReg[XM125D_MAX_PEAKS] = {}; ///< ungekuerztes Registerwort, siehe getPeakDistanceReg()
    int32_t  _peakStrength[XM125D_MAX_PEAKS]    = {};
};
