#pragma once

/**
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  XM125Presence.h  —  Acconeer XM125 Presence-Detector (I²C-Firmware)     ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  Registerkarte & Kommandos lt. Acconeer "I2C Presence Detector User      ║
 * ║  Guide". Erkennt Anwesenheit über schnelle (Intra) und langsame (Inter)  ║
 * ║  Bewegungen im konfigurierten Distanzfenster.                            ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * Beispiel-Verwendung:
 *   XM125Presence radar;
 *   radar.begin(Wire1);
 *   radar.config.startMm = 300;
 *   radar.config.endMm   = 2000;
 *   radar.applyConfiguration();
 *   radar.start();
 *   ...
 *   if (radar.update() && radar.isPresenceDetected()) {
 *     Serial.println(radar.getPresenceDistanceMm());
 *   }
 */

#include "XM125.h"

// ── Ergebnisregister ───────────────────────────────────────────────────────────
static constexpr uint16_t XM125P_REG_PRESENCE_RESULT    = 0x0010; ///< R: Detektions-Flags + Temperatur
static constexpr uint16_t XM125P_REG_PRESENCE_DISTANCE   = 0x0011; ///< R: Distanz der Detektion [mm]
static constexpr uint16_t XM125P_REG_INTRA_PRESENCE_SCORE = 0x0012; ///< R: Score schnelle Bewegung
static constexpr uint16_t XM125P_REG_INTER_PRESENCE_SCORE = 0x0013; ///< R: Score langsame Bewegung

// ── Konfigurationsregister (Default lt. Acconeer-Datenblatt) ──────────────────
static constexpr uint16_t XM125P_REG_SWEEPS_PER_FRAME              = 0x0040; ///< Default 16
static constexpr uint16_t XM125P_REG_INTER_FRAME_PRESENCE_TIMEOUT  = 0x0041; ///< [s], Default 3
static constexpr uint16_t XM125P_REG_INTER_PHASE_BOOST_ENABLED     = 0x0042; ///< bool, Default false
static constexpr uint16_t XM125P_REG_INTRA_DETECTION_ENABLED       = 0x0043; ///< bool, Default true
static constexpr uint16_t XM125P_REG_INTER_DETECTION_ENABLED       = 0x0044; ///< bool, Default true
static constexpr uint16_t XM125P_REG_FRAME_RATE                    = 0x0045; ///< [mHz], Default 12000
static constexpr uint16_t XM125P_REG_INTRA_DETECTION_THRESHOLD     = 0x0046; ///< 1000x-Einheit, Default 1300
static constexpr uint16_t XM125P_REG_INTER_DETECTION_THRESHOLD     = 0x0047; ///< 1000x-Einheit, Default 1000
static constexpr uint16_t XM125P_REG_INTER_FRAME_DEVIATION_TC      = 0x0048; ///< [ms], Default 500
static constexpr uint16_t XM125P_REG_INTER_FRAME_FAST_CUTOFF       = 0x0049; ///< [mHz], Default 6000
static constexpr uint16_t XM125P_REG_INTER_FRAME_SLOW_CUTOFF       = 0x004A; ///< [mHz], Default 200
static constexpr uint16_t XM125P_REG_INTRA_FRAME_TIME_CONST        = 0x004B; ///< [ms], Default 150
static constexpr uint16_t XM125P_REG_INTRA_OUTPUT_TIME_CONST       = 0x004C; ///< [ms], Default 300
static constexpr uint16_t XM125P_REG_INTER_OUTPUT_TIME_CONST       = 0x004D; ///< [ms], Default 2000
static constexpr uint16_t XM125P_REG_AUTO_PROFILE_ENABLED          = 0x004E; ///< bool, Default true
static constexpr uint16_t XM125P_REG_AUTO_STEP_LENGTH_ENABLED      = 0x004F; ///< bool, Default true
static constexpr uint16_t XM125P_REG_MANUAL_PROFILE                = 0x0050; ///< PROFILE1..5, Default PROFILE4
static constexpr uint16_t XM125P_REG_MANUAL_STEP_LENGTH             = 0x0051; ///< Default 72
static constexpr uint16_t XM125P_REG_START                          = 0x0052; ///< [mm], Default 300
static constexpr uint16_t XM125P_REG_END                            = 0x0053; ///< [mm], Default 2500
static constexpr uint16_t XM125P_REG_RESET_FILTERS_ON_PREPARE       = 0x0054; ///< bool, Default true
static constexpr uint16_t XM125P_REG_HWAAS                          = 0x0055; ///< Default 32
static constexpr uint16_t XM125P_REG_DETECTION_ON_GPIO              = 0x0080; ///< bool, Default false

// ── Kommandos (0x0100) ────────────────────────────────────────────────────────
static constexpr uint32_t XM125P_CMD_APPLY_CONFIGURATION = 1;
static constexpr uint32_t XM125P_CMD_START_DETECTOR      = 2;
static constexpr uint32_t XM125P_CMD_STOP_DETECTOR       = 3;

enum XM125PProfile : uint32_t {
    XM125P_PROFILE1 = 1, XM125P_PROFILE2 = 2, XM125P_PROFILE3 = 3,
    XM125P_PROFILE4 = 4, XM125P_PROFILE5 = 5,
};

// ── Detector-Status Bits, spezifisch für Presence-Firmware (Reg 0x0003) ──────
static constexpr uint32_t XM125P_STATUS_RSS_REGISTER_OK     = (1UL << 0);
static constexpr uint32_t XM125P_STATUS_CONFIG_CREATE_OK    = (1UL << 1);
static constexpr uint32_t XM125P_STATUS_SENSOR_CREATE_OK    = (1UL << 2);
static constexpr uint32_t XM125P_STATUS_SENSOR_CALIBRATE_OK = (1UL << 3);
static constexpr uint32_t XM125P_STATUS_DETECTOR_CREATE_OK  = (1UL << 4);
static constexpr uint32_t XM125P_STATUS_DETECTOR_BUFFER_OK  = (1UL << 5);
static constexpr uint32_t XM125P_STATUS_SENSOR_BUFFER_OK    = (1UL << 6);
static constexpr uint32_t XM125P_STATUS_CONFIG_APPLY_OK     = (1UL << 7);
// Fehler-Bits spiegeln die Success-Bits ab Bit 16 (z.B. RSS_REGISTER_ERROR = Bit 16)
// XM125_STATUS_DETECTOR_ERROR (Bit 28) und XM125_STATUS_BUSY (Bit 31) siehe XM125.h

// ── Presence-Result Bitfelder (Reg 0x0010) ────────────────────────────────────
static constexpr uint32_t XM125P_RESULT_PRESENCE_DETECTED         = (1UL << 0);
static constexpr uint32_t XM125P_RESULT_PRESENCE_DETECTED_STICKY  = (1UL << 1);
static constexpr uint32_t XM125P_RESULT_DETECTOR_ERROR            = (1UL << 15);
static constexpr uint32_t XM125P_RESULT_TEMPERATURE_SHIFT         = 16; ///< Bit 16-31

// ═════════════════════════════════════════════════════════════════════════════
//  PresenceConfig  —  Konfigurationswerte, Default lt. Acconeer-Datenblatt
// ═════════════════════════════════════════════════════════════════════════════
struct XM125PresenceConfig {
    uint32_t sweepsPerFrame               = 16;
    uint32_t interFramePresenceTimeoutS   = 3;
    bool     interPhaseBoostEnabled       = false;
    bool     intraDetectionEnabled        = true;
    bool     interDetectionEnabled        = true;
    uint32_t frameRateMilliHz             = 12000;
    uint32_t intraDetectionThreshold      = 1300;
    uint32_t interDetectionThreshold      = 1000;
    uint32_t interFrameDeviationTimeConstMs = 500;
    uint32_t interFrameFastCutoffMilliHz  = 6000;
    uint32_t interFrameSlowCutoffMilliHz  = 200;
    uint32_t intraFrameTimeConstMs        = 150;
    uint32_t intraOutputTimeConstMs       = 300;
    uint32_t interOutputTimeConstMs       = 2000;
    bool     autoProfileEnabled           = true;
    bool     autoStepLengthEnabled        = true;
    uint32_t manualProfile                = XM125P_PROFILE4;
    uint32_t manualStepLength             = 72;
    uint32_t startMm                      = 300;
    uint32_t endMm                        = 2500;
    bool     resetFiltersOnPrepare        = true;
    uint32_t hwaas                        = 32;
    bool     detectionOnGpio              = false;
};

// ═════════════════════════════════════════════════════════════════════════════
//  XM125Presence  —  Presence-Detector-Applikation
// ═════════════════════════════════════════════════════════════════════════════
class XM125Presence : public XM125Base {
public:
    explicit XM125Presence(uint8_t i2cAddr = XM125_I2C_ADDR_DEFAULT, int8_t wakePin = -1, int8_t intPin = -1)
        : XM125Base(i2cAddr, wakePin, intPin) {}

    XM125PresenceConfig config; ///< Vor applyConfiguration() anpassen (danach nicht mehr änderbar!)

    /**
     * Schreibt alle Werte aus `config` in die Konfigurationsregister.
     * Bricht beim ersten fehlgeschlagenen Register-Write ab (fail-fast).
     * Bei Misserfolg liefert getLastFailedConfigRegister() die Register-
     * adresse, an der die I²C-Übertragung fehlgeschlagen ist.
     */
    bool writeConfig();

    /** Registeradresse des zuletzt fehlgeschlagenen writeConfig()-Schreibvorgangs, 0xFFFF = keiner. */
    uint16_t getLastFailedConfigRegister() const { return _lastFailedConfigRegister; }

    /** writeConfig() + Kommando APPLY_CONFIGURATION, blockiert bis fertig. */
    bool applyConfiguration(uint32_t timeoutMs = XM125_DEFAULT_APPLY_TIMEOUT_MS);

    /** Startet den Detektor (danach liefert update() laufend neue Ergebnisse). */
    bool start(uint32_t timeoutMs = XM125_DEFAULT_APPLY_TIMEOUT_MS);

    /** Stoppt den Detektor. */
    bool stop(uint32_t timeoutMs = XM125_DEFAULT_APPLY_TIMEOUT_MS);

    /**
     * Liest das aktuelle Ergebnisregister ein (non-blocking, kein Kommando
     * nötig — der Detektor misst nach start() kontinuierlich selbstständig).
     * @return true wenn das Register erfolgreich gelesen wurde.
     */
    bool update();

    bool     isPresenceDetected()       const { return _presenceDetected; }
    bool     isPresenceDetectedSticky() const { return _presenceDetectedSticky; }
    bool     hasDetectorError()         const { return _detectorError; }
    int16_t  getTemperature()           const { return _temperature; } ///< °C, nur relative Genauigkeit
    uint32_t getPresenceDistanceMm()    const { return _presenceDistanceMm; }
    uint32_t getIntraPresenceScore()    const { return _intraScore; }
    uint32_t getInterPresenceScore()    const { return _interScore; }

private:
    uint16_t _lastFailedConfigRegister = 0xFFFF;

    bool     _presenceDetected       = false;
    bool     _presenceDetectedSticky = false;
    bool     _detectorError          = false;
    int16_t  _temperature            = 0;
    uint32_t _presenceDistanceMm     = 0;
    uint32_t _intraScore             = 0;
    uint32_t _interScore              = 0;
};
