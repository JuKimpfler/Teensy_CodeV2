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
 * ── Die Library wartet nie auf ein Messergebnis ───────────────────────────
 * Eine Messung dauert je nach Konfiguration 20-200 ms. Diese Zeit darf der
 * Roboterzyklus nicht mit Warten verbringen, deshalb besteht der Messbetrieb
 * aus genau drei Kommandos:
 *
 *   1. startMeasurement()   Messung starten   -- ein Registerschreibzugriff,
 *                                                kehrt sofort zurueck
 *   2. isMeasuring()        laeuft eine Messung? -- treibt die interne
 *                                                Zustandsmaschine weiter und
 *                                                kostet hoechstens EINEN
 *                                                Registerlesezugriff
 *   3. getLastResult()      letztes Messergebnis abfragen -- reiner Zugriff
 *                                                auf den Cache, kein I²C
 *
 * Sobald isMeasuring() false liefert, hat die Library das Ergebnis bereits
 * eingelesen; getLastResult() ist dann aktuell. Zwischen zwei Runden bleibt
 * das letzte Ergebnis abrufbar -- auch waehrend die naechste Messung laeuft.
 *
 * Beispiel-Verwendung (ein Sensor):
 *   XM125Distance radar;
 *   radar.begin(Wire1);
 *   radar.config.startMm = 200;
 *   radar.config.endMm   = 4000;
 *   radar.applyConfigAndCalibrate();
 *   radar.startMeasurement();
 *   ...
 *   void loop() {
 *     if (!radar.isMeasuring()) {                  // fertig?
 *       const XM125DistanceResult& r = radar.getLastResult();
 *       if (r.valid && r.numDistances > 0) {
 *         Serial.println(r.peakDistanceRaw[0]);
 *       }
 *       radar.startMeasurement();                  // naechste Runde
 *     }
 *   }
 *
 * Mehrere Sensoren messen gleichzeitig, indem bei allen erst
 * startMeasurement() aufgerufen wird und danach reihum isMeasuring():
 *   for (auto& r : sensoren) r.startMeasurement();  // alle zusammen los
 *   ...
 *   bool laeuft = false;
 *   for (auto& r : sensoren) if (r.isMeasuring()) laeuft = true;
 *   if (!laeuft) { ... }        // alle Ergebnisse liegen vor
 * So faellt die Messdauer einmal fuer alle Sensoren an statt einmal pro
 * Sensor. Umgesetzt in systemlib::readRadar().
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
static constexpr uint16_t XM125D_REG_PEAK_SORTING                   = 0x0047; ///< R/W: siehe PeakSorting (Default STRONGEST). FARTHEST/WEAKEST sind
                                                                                ///< firmwareseitige Erweiterungen: die RSS-Bibliothek im Sensor kennt nur
                                                                                ///< CLOSEST/STRONGEST, die Firmware dreht die bereits sortierte Peak-Liste
                                                                                ///< dafuer intern um (siehe distance_reg_peak_sorting_is_reversed() in der
                                                                                ///< Sensor-Firmware). Fuer diese Library ist das transparent -- einfach den
                                                                                ///< Enum-Wert setzen, die Peaks kommen bereits richtig sortiert zurueck.
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

// ── Timing der Zustandsmaschine (nicht blockierend, siehe .cpp) ──────────────
/**
 * Wartezeit nach dem Messkommando, bevor das erste Mal auf BUSY geprueft wird.
 * Die interne MCU des Moduls setzt BUSY nicht in derselben Mikrosekunde, in der
 * das Kommando ankommt. Ohne diese Sperre koennte der erste Poll BUSY=0 sehen
 * und das Ergebnisregister der VORHERIGEN Messung als neu ausgeben.
 */
static constexpr uint32_t XM125D_BUSY_SETTLE_US = 2000;
/**
 * Wartezeit zwischen Ergebnisregister und Peak-Registern. Das Result-Register
 * (Peak-Anzahl, Temperatur, Flags) ist gueltig, sobald BUSY zurueckgeht; die
 * Peak-Detailregister braucht die Firmware erkennbar ein paar Millisekunden
 * laenger -- ohne diese Pause wurden unplausible Werte beobachtet. Die Pause
 * wird nicht verschlafen, sondern ueber die Zustandsmaschine ausgesessen
 * (Zustand SETTLING kostet keinen einzigen Buszugriff).
 */
static constexpr uint32_t XM125D_PEAK_SETTLE_US = 5000;

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
    XM125D_SORT_FARTHEST  = 3, ///< CLOSEST rueckwaerts   (am weitesten entfernter Peak zuerst)
    XM125D_SORT_WEAKEST   = 4, ///< STRONGEST rueckwaerts (schwaechster Peak zuerst)
};
enum XM125DReflectorShape : uint32_t {
    XM125D_REFLECTOR_GENERIC = 1,
    XM125D_REFLECTOR_PLANAR  = 2,
};

/** Zustand der Messung eines Sensors, abfragbar ueber getState(). */
enum class XM125MeasureState : uint8_t {
    IDLE,     ///< noch nie gemessen bzw. nach begin()
    RUNNING,  ///< Kommando abgesetzt, der Sensor rechnet
    SETTLING, ///< BUSY ist weg, die Peak-Register brauchen noch ein paar ms
    READY,    ///< Ergebnis vollstaendig eingelesen (siehe getLastResult())
    FAILED,   ///< abgebrochen: I²C-Fehler, DETECTOR_ERROR oder Timeout
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

// ── Werksvorgaben der Konfigurationsregister (lt. Acconeer-Datenblatt) ──────
// An genau einer Stelle definiert, damit XM125DistanceConfig und die
// Parameterliste von hardCalibration() nicht auseinanderlaufen koennen.
static constexpr uint32_t XM125D_DEFAULT_START_MM                      = 150;
static constexpr uint32_t XM125D_DEFAULT_END_MM                        = 3000;
static constexpr uint32_t XM125D_DEFAULT_MAX_STEP_LENGTH               = 0;      ///< 0 = automatisch
static constexpr bool     XM125D_DEFAULT_CLOSE_RANGE_LEAKAGE_CANCEL    = false;
static constexpr uint32_t XM125D_DEFAULT_SIGNAL_QUALITY                = 35000;
static constexpr uint32_t XM125D_DEFAULT_MAX_PROFILE                   = XM125D_PROFILE3;
static constexpr uint32_t XM125D_DEFAULT_THRESHOLD_METHOD              = XM125D_THRESHOLD_CFAR;
static constexpr uint32_t XM125D_DEFAULT_PEAK_SORTING                  = XM125D_SORT_FARTHEST;
static constexpr uint32_t XM125D_DEFAULT_NUM_FRAMES_RECORDED_THRESHOLD = 100;
static constexpr uint32_t XM125D_DEFAULT_FIXED_AMPLITUDE_THRESHOLD     = 100000;
static constexpr uint32_t XM125D_DEFAULT_THRESHOLD_SENSITIVITY         = 500;
static constexpr uint32_t XM125D_DEFAULT_REFLECTOR_SHAPE               = XM125D_REFLECTOR_PLANAR;
static constexpr uint32_t XM125D_DEFAULT_FIXED_STRENGTH_THRESHOLD      = -25;
static constexpr bool     XM125D_DEFAULT_MEASURE_ON_WAKEUP             = false;

// ═════════════════════════════════════════════════════════════════════════════
//  DistanceConfig  —  Konfigurationswerte, Default lt. Acconeer-Datenblatt
// ═════════════════════════════════════════════════════════════════════════════
struct XM125DistanceConfig {
    uint32_t startMm                     = XM125D_DEFAULT_START_MM;
    uint32_t endMm                       = XM125D_DEFAULT_END_MM;
    uint32_t maxStepLength                = XM125D_DEFAULT_MAX_STEP_LENGTH;  ///< 0 = automatisch
    bool     closeRangeLeakageCancellation = XM125D_DEFAULT_CLOSE_RANGE_LEAKAGE_CANCEL;
    uint32_t signalQuality                = XM125D_DEFAULT_SIGNAL_QUALITY;
    uint32_t maxProfile                   = XM125D_DEFAULT_MAX_PROFILE;
    uint32_t thresholdMethod              = XM125D_DEFAULT_THRESHOLD_METHOD;
    uint32_t peakSorting                  = XM125D_DEFAULT_PEAK_SORTING;
    uint32_t numFramesRecordedThreshold    = XM125D_DEFAULT_NUM_FRAMES_RECORDED_THRESHOLD;
    uint32_t fixedAmplitudeThreshold      = XM125D_DEFAULT_FIXED_AMPLITUDE_THRESHOLD;
    uint32_t thresholdSensitivity          = XM125D_DEFAULT_THRESHOLD_SENSITIVITY;
    uint32_t reflectorShape                = XM125D_DEFAULT_REFLECTOR_SHAPE;
    uint32_t fixedStrengthThreshold       = XM125D_DEFAULT_FIXED_STRENGTH_THRESHOLD;
    bool     measureOnWakeup              = XM125D_DEFAULT_MEASURE_ON_WAKEUP;
};

// ═════════════════════════════════════════════════════════════════════════════
//  XM125DistanceResult  —  vollstaendiges Ergebnis EINER Messung
// ═════════════════════════════════════════════════════════════════════════════
/**
 * Wird von der Library gefuellt, sobald eine Messung durchgelaufen ist, und
 * bleibt bis zum naechsten Ergebnis unveraendert stehen -- auch waehrend schon
 * die naechste Messung laeuft. getLastResult() liefert genau diese Struktur.
 *
 * Nach einem Fehlversuch (I²C, DETECTOR_ERROR, Timeout, MEASURE_ERROR) steht
 * `valid` auf false, die Zahlenwerte bleiben aber auf dem letzten guten Stand.
 * Wer nur frische Werte verarbeiten will, prueft `valid` und/oder `timestampMs`.
 */
struct XM125DistanceResult {
    bool     valid             = false; ///< true = die Werte stammen aus einer fehlerfreien Messung
    uint32_t timestampMs       = 0;     ///< millis() beim Einlesen des Ergebnisses
    uint16_t durationMs        = 0;     ///< von startMeasurement() bis Ergebnis (Diagnose)

    uint8_t  numDistances      = 0;     ///< Anzahl gefundener Peaks (0..10)
    bool     nearStartEdge     = false; ///< Objekt dicht an config.startMm
    bool     calibrationNeeded = false; ///< Sensor fordert recalibrate() an
    bool     measureError      = false; ///< Messung selbst meldet einen Fehler
    int16_t  temperature       = 0;     ///< °C, geringe Absolutgenauigkeit lt. Datenblatt

    /** Siehe Warnung bei getPeakDistanceRaw(): unkalibrierte "Counts", keine mm. */
    uint16_t  peakDistanceRaw[XM125D_MAX_PEAKS] = {};
    /** Ungekuerztes 32-bit-Registerwort je Peak -- Grundlage der Rohwert-Kalibrierung. */
    uint32_t peakDistanceReg[XM125D_MAX_PEAKS] = {};
    /** Signalstaerke je Peak (1000x-Einheit lt. Datenblatt, RSS-intern). */
    int32_t  peakStrength[XM125D_MAX_PEAKS]    = {};
};

// ═════════════════════════════════════════════════════════════════════════════
//  XM125Distance  —  Distance-Detector-Applikation
// ═════════════════════════════════════════════════════════════════════════════
class XM125Distance : public XM125Base {
public:
    explicit XM125Distance(uint8_t i2cAddr = XM125_I2C_ADDR_DEFAULT, int8_t wakePin = -1, int8_t intPin = -1)
        : XM125Base(i2cAddr, wakePin, intPin) {}

    XM125DistanceConfig config; ///< Vor applyConfigAndCalibrate() anpassen

    // ── Einrichtung (blockiert, laeuft nur beim Start) ────────────────────

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

    /** Sensor/Detektor neu kalibrieren (z.B. wenn calibrationNeeded gesetzt ist). */
    bool calibrate(uint32_t timeoutMs = XM125_DEFAULT_APPLY_TIMEOUT_MS);
    bool recalibrate(uint32_t timeoutMs = XM125_DEFAULT_APPLY_TIMEOUT_MS);

    // ── Harte Neukalibrierung: Neustart + komplette Konfiguration ─────────

    /**
     * HARTE NEUKALIBRIERUNG. Startet den Sensor neu und laedt danach ALLE
     * Konfigurationsregister neu hoch, gefolgt von einer vollstaendigen
     * Kalibrierung. Drei Schritte in einem Aufruf:
     *
     *   1. resetModule()               Modul neu starten, warten bis bereit
     *   2. writeConfig()               alle 14 Konfigurationsregister schreiben
     *   3. APPLY_CONFIG_AND_CALIBRATE  anwenden, Sensor+Detektor kalibrieren
     *
     * Wofuer: Nach einem DETECTOR_ERROR, bei haengendem BUSY oder wenn ein
     * Sensor dauerhaft Unsinn liefert, reicht applyConfigAndCalibrate() nicht
     * mehr -- erst der Neustart raeumt den internen Zustand des Moduls ab. Da
     * ein Neustart die Register auf die Werksvorgaben zuruecksetzt, werden
     * hier grundsaetzlich ALLE Werte neu geschrieben, nicht nur die vom
     * Default abweichenden.
     *
     * ACHTUNG - BLOCKIERT bis zu resetTimeoutMs + timeoutMs (typisch ~1-3 s).
     * Gehoert ins Hochfahren oder in ein Kalibrierskript, NICHT in den
     * Roboterzyklus. Waehrend der Kalibrierung darf sich vor dem Sensor nichts
     * bewegen -- die Umgebung wird als Referenz aufgenommen.
     *
     * Eine laufende Messung wird verworfen (Zustand IDLE, letztes Ergebnis
     * ungueltig); startMeasurement() muss danach neu aufgerufen werden. Die
     * reinen Library-Einstellungen setMeasureTimeout() und setMaxPeaksToFetch()
     * bleiben unberuehrt -- sie stehen im Treiber, nicht im Sensor.
     *
     * @param cfg             kompletter Satz Konfigurationswerte; wird in
     *                        `config` uebernommen, auch wenn es schiefgeht.
     * @param timeoutMs       Zeitlimit fuer Anwenden + Kalibrieren.
     * @param resetTimeoutMs  Zeitlimit fuer den Neustart.
     * @return true nur, wenn Neustart, Registerschreiben und Kalibrierung alle
     *         durchgelaufen sind. Bei false zeigt getLastFailedConfigRegister()
     *         das Register, an dem das Schreiben scheiterte (0xFFFF = es lag
     *         nicht am Schreiben, sondern am Neustart oder an der Kalibrierung).
     */
    bool hardCalibration(const XM125DistanceConfig& cfg,
                         uint32_t timeoutMs      = XM125_DEFAULT_APPLY_TIMEOUT_MS,
                         uint32_t resetTimeoutMs = XM125_RESET_TIMEOUT_MS);

    /**
     * Gleiche Funktion, nur mit Einzelparametern statt Struktur: jeder
     * Konfigurationswert ist einzeln setzbar, alles Weggelassene bekommt die
     * Werksvorgabe aus dem Datenblatt (XM125D_DEFAULT_*).
     *
     *   radar.hardCalibration();                     // alles auf Werksvorgabe
     *   radar.hardCalibration(150, 2500);            // nur Messbereich anders
     *   radar.hardCalibration(150, 2500, 0, false);  // + Leakage-Kompensation aus
     *
     * Weiter hinten liegende Werte einzeln zu setzen wird schnell unleserlich
     * ("was war noch der neunte Parameter?"). Dafuer die Variante mit
     * XM125DistanceConfig nehmen -- dort steht jeder Wert bei seinem Namen:
     *
     *   XM125DistanceConfig cfg;                 // startet auf den Werksvorgaben
     *   cfg.startMm        = RADAR_START_MM;
     *   cfg.reflectorShape = XM125D_REFLECTOR_PLANAR;
     *   radar.hardCalibration(cfg);
     *
     * Zeitlimits sind hier bewusst nicht anhaengbar -- die Parameterliste ist
     * lang genug; wer sie braucht, nimmt die Struktur-Variante.
     */
    bool hardCalibration(uint32_t startMm,
                         uint32_t endMm                         = XM125D_DEFAULT_END_MM,
                         uint32_t maxStepLength                 = XM125D_DEFAULT_MAX_STEP_LENGTH,
                         bool     closeRangeLeakageCancellation = XM125D_DEFAULT_CLOSE_RANGE_LEAKAGE_CANCEL,
                         uint32_t signalQuality                 = XM125D_DEFAULT_SIGNAL_QUALITY,
                         uint32_t maxProfile                    = XM125D_DEFAULT_MAX_PROFILE,
                         uint32_t thresholdMethod               = XM125D_DEFAULT_THRESHOLD_METHOD,
                         uint32_t peakSorting                   = XM125D_DEFAULT_PEAK_SORTING,
                         uint32_t numFramesRecordedThreshold    = XM125D_DEFAULT_NUM_FRAMES_RECORDED_THRESHOLD,
                         uint32_t fixedAmplitudeThreshold       = XM125D_DEFAULT_FIXED_AMPLITUDE_THRESHOLD,
                         uint32_t thresholdSensitivity          = XM125D_DEFAULT_THRESHOLD_SENSITIVITY,
                         uint32_t reflectorShape                = XM125D_DEFAULT_REFLECTOR_SHAPE,
                         uint32_t fixedStrengthThreshold        = XM125D_DEFAULT_FIXED_STRENGTH_THRESHOLD,
                         bool     measureOnWakeup               = XM125D_DEFAULT_MEASURE_ON_WAKEUP);

    /** Ohne Argumente: Neustart + komplette Werksvorgabe (eigene Ueberladung,
     *  damit der Aufruf hardCalibration() nicht zwischen den beiden oberen
     *  Varianten mehrdeutig wird). */
    bool hardCalibration() { return hardCalibration(XM125D_DEFAULT_START_MM); }

    // ═══ Messbetrieb: die drei Kommandos ══════════════════════════════════

    /**
     * MESSUNG STARTEN. Sendet MEASURE_DISTANCE und kehrt sofort zurueck --
     * es wird NICHT auf das Ergebnis gewartet (Kosten: ein Registerschreib-
     * zugriff, < 1 ms). Ein bereits vorliegendes Ergebnis bleibt bis zum
     * Eintreffen des neuen ueber getLastResult() abrufbar.
     * @return false, wenn das Kommando den Sensor nicht erreicht hat
     *         (Zustand wechselt dann auf FAILED).
     */
    bool startMeasurement();

    /**
     * LAEUFT EINE MESSUNG? Treibt dabei die Zustandsmaschine weiter: prueft
     * BUSY, holt bei Bedarf Ergebnis- und Peak-Register ab und setzt den
     * Zustand auf READY oder FAILED. Blockiert nie.
     *
     * Kosten je Aufruf: hoechstens ein Registerlesezugriff (Zustand RUNNING),
     * in SETTLING gar keiner, und in IDLE/READY/FAILED ebenfalls keiner --
     * mehrfaches Aufrufen kostet also nichts, sobald die Messung durch ist.
     *
     * @return true solange die Messung laeuft; false sobald das Ergebnis
     *         vollstaendig eingelesen ist ODER der Versuch fehlgeschlagen
     *         ist (unterscheiden ueber hasResult() bzw. getState()).
     */
    bool isMeasuring();

    /**
     * LETZTES MESSERGEBNIS ABFRAGEN. Reiner Cache-Zugriff, kein I²C, keine
     * Wartezeit -- darf beliebig oft aufgerufen werden.
     */
    const XM125DistanceResult& getLastResult() const { return _result; }

    /** true, wenn das zwischengespeicherte Ergebnis aus einer fehlerfreien Messung stammt. */
    bool hasResult() const { return _result.valid; }

    /** Alter des zwischengespeicherten Ergebnisses in ms (0, wenn noch nie eines vorlag). */
    uint32_t getResultAgeMs() const {
        return (_result.timestampMs == 0) ? 0 : (millis() - _result.timestampMs);
    }

    /** Zustand der Zustandsmaschine, ohne sie weiterzutreiben (kein I²C). */
    XM125MeasureState getState() const { return _state; }

    // ── Stellschrauben fuer die Messdauer ─────────────────────────────────

    /**
     * Ab wann eine laufende Messung als verloren gilt: isMeasuring() liefert
     * dann false und der Zustand wechselt auf FAILED. Default:
     * XM125_DEFAULT_MEASURE_TIMEOUT_MS.
     */
    void setMeasureTimeout(uint32_t timeoutMs) { _measureTimeoutMs = timeoutMs; }
    uint32_t getMeasureTimeout() const { return _measureTimeoutMs; }

    /**
     * Wie viele Peaks nach der Messung wirklich aus dem Sensor geholt werden
     * (Default: alle 10). Jeder Peak kostet zwei I²C-Registerlesezugriffe --
     * wer nur den staerksten/naechsten Peak auswertet, setzt hier 1 und spart
     * den Rest. `numDistances` im Ergebnis bleibt die echte Anzahl; die Werte
     * der nicht geholten Peaks stehen auf 0.
     */
    void setMaxPeaksToFetch(uint8_t maxPeaks) {
        _maxPeaksToFetch = (maxPeaks > XM125D_MAX_PEAKS) ? XM125D_MAX_PEAKS : maxPeaks;
    }
    uint8_t getMaxPeaksToFetch() const { return _maxPeaksToFetch; }

    /**
     * NUR FUER KALIBRIER-/TESTSKRIPTE: startet eine Messung und wartet
     * blockierend, bis das Ergebnis vorliegt (startMeasurement() +
     * isMeasuring()-Schleife). Im Roboterzyklus nicht verwenden -- das ist
     * genau die Wartezeit, die die drei Kommandos oben vermeiden.
     * @return true, wenn ein gueltiges Ergebnis vorliegt.
     */
    bool measureBlocking(uint32_t timeoutMs = XM125_DEFAULT_MEASURE_TIMEOUT_MS);

    // ── Bequeme Einzelzugriffe auf getLastResult() (kein I²C) ─────────────
    uint8_t  getNumDistances()      const { return _result.numDistances; }
    bool     getNearStartEdge()     const { return _result.nearStartEdge; }
    bool     getCalibrationNeeded() const { return _result.calibrationNeeded; }
    bool     getMeasureError()      const { return _result.measureError; }
    int16_t  getTemperature()       const { return _result.temperature; } ///< °C, geringe Absolutgenauigkeit lt. Datenblatt

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

    XM125MeasureState   _state  = XM125MeasureState::IDLE;
    XM125DistanceResult _result;

    uint32_t _measureTimeoutMs = XM125_DEFAULT_MEASURE_TIMEOUT_MS;
    uint8_t  _maxPeaksToFetch  = XM125D_MAX_PEAKS;

    uint32_t _startUs      = 0; ///< micros() beim Absetzen des Messkommandos
    uint32_t _settleUs     = 0; ///< micros() beim Lesen des Ergebnisregisters
    uint8_t  _peaksToRead  = 0; ///< in dieser Runde noch abzuholende Peaks

    /** Ein Schritt in RUNNING: BUSY pruefen und ggf. Ergebnisregister lesen. */
    void _pollRunning();
    /** Ein Schritt in SETTLING: nach Ablauf der Pause die Peak-Register lesen. */
    void _pollSettling();
    /** Ergebnisregister 0x0010 einlesen und zerlegen. */
    bool _readResultRegister();
    /** Peak-Register der abgeschlossenen Messung einlesen. */
    bool _readPeaks();
    /** Runde erfolgreich abschliessen (Zustand READY). */
    void _finish();
    /** Runde abbrechen (Zustand FAILED, Ergebnis als nicht mehr gueltig markieren). */
    void _fail();
};
