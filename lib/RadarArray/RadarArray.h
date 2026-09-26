#pragma once
#include <Arduino.h>
#include <Wire.h>

#include "params.h"
#include <XM125Distance.h>

/**
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  RadarArray.h  —  alle XM125-Radarsensoren in EINEM Objekt              ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  Kapselt die komplette Ansteuerung der Radarsensoren: Einrichtung,      ║
 * ║  gleichzeitige Messrunden, Fehlerzaehlung und Wiederanlauf toter        ║
 * ║  Sensoren. Nach aussen bleiben genau drei Aufrufe uebrig:               ║
 * ║                                                                          ║
 * ║      Radar.begin();   // einmal beim Hochfahren   (blockiert)           ║
 * ║      Radar.update();  // einmal je Zyklus         (blockiert NIE)       ║
 * ║      Radar.calib();   // bei Bedarf               (blockiert)           ║
 * ║                                                                          ║
 * ║  Messwerte kommen ueber den Index-Operator, benannt nach der Einbau-    ║
 * ║  position:                                                              ║
 * ║                                                                          ║
 * ║      uint16_t d = Radar[RadarVo].Dist;                                  ║
 * ║      if (Radar[RadarLi].isCon) { ... }                                  ║
 * ║                                                                          ║
 * ║  Abstimmung (Reichweite, Empfindlichkeit, ...) steht in params.h und    ║
 * ║  landet beim Anlegen in `config`. Wer sie zur Laufzeit aendern will:    ║
 * ║  `config` anpassen und danach calib() aufrufen.                         ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * Alle Sensoren messen GLEICHZEITIG: update() stoesst eine Runde bei allen
 * Sensoren an und sieht in den Folgezyklen nur noch nach, ob die Ergebnisse
 * da sind. Gewartet wird nirgends, die Messdauer faellt also einmal fuer
 * alle Sensoren an statt einmal pro Sensor.
 *
 * Hinweis: Wire.begin() (bzw. Wire1.begin()) muss VOR begin() gelaufen sein.
 */

/// Einbauposition -> Sensorindex. Reihenfolge wie RADAR_SENSOR_ADDRS in params.h.
enum RadarPos : uint8_t
{
    RadarVo = 0, ///< vorne
    RadarLi = 1, ///< links
    RadarHi = 2, ///< hinten
    RadarRe = 3  ///< rechts
};

/// Messwerte EINES Sensors — immer Peak 0, der Rest wird nicht ausgewertet.
struct RadarData
{
    uint16_t Dist     = 0;     ///< Rohwert von Peak 0 ("Counts", NICHT mm — siehe XM125.md)
    int32_t  Strenght = 0;     ///< Signalstaerke von Peak 0
    uint8_t  Peaks    = 0;     ///< Anzahl erkannter Peaks (0 = nichts gesehen)
    bool     isCon    = false; ///< true = Sensor antwortet und ist konfiguriert
    uint32_t Time     = 0;     ///< millis() der letzten uebernommenen Messung (0 = noch nie gemessen)
};

class RadarArray
{
public:
    RadarArray();

    /// Konfiguration ALLER Sensoren. Vorbelegt aus params.h, siehe calib().
    XM125DistanceConfig config;

    /* -------------- System Worker -------------- */

    /**
     * Alle Sensoren einrichten (Adressen aus params.h) und die erste Messrunde
     * anstossen. BLOCKIERT — pro Sensor bis ~2 s, nur beim Hochfahren aufrufen.
     * @return Anzahl der Sensoren, die geantwortet haben.
     */
    uint8_t begin(TwoWire &wire = I2C_RADAR);

    /**
     * Einmal je Zyklus aufrufen. Holt fertige Ergebnisse ab und startet die
     * naechste Runde, sobald alle Sensoren durch sind. BLOCKIERT NIE
     * (hoechstens ein Registerzugriff je Sensor).
     */
    void update(void);
    float filterRadar(int sensorIndex, float rawValue, unsigned long nowMs);
    /**
     * Alle Sensoren mit dem aktuellen Inhalt von `config` neu konfigurieren
     * und kalibrieren. BLOCKIERT (~1 s je Sensor), nicht im Roboterzyklus.
     */
    void calib(void);

    /* -------------- give -------------- */

    /// Messwerte einer Einbauposition: Radar[RadarVo].Dist
    RadarData       &operator[](uint8_t pos)       { return _data[_clamp(pos)]; }
    const RadarData &operator[](uint8_t pos) const { return _data[_clamp(pos)]; }

    /// true, wenn der Sensor antwortet und konfiguriert ist.
    bool giveCon(uint8_t pos) const { return _data[_clamp(pos)].isCon; }

    /**
     * Alter des Messwerts in ms: wie lange ist die letzte uebernommene Messung
     * her. Ein Sensor, der noch nie gemessen hat, liefert millis() -- also einen
     * sehr grossen Wert, der den Messwert von selbst als unbrauchbar ausweist.
     */
    uint32_t giveAge(uint8_t pos) const { return millis() - _data[_clamp(pos)].Time; }

    /// Anzahl der verbauten Sensoren (RADAR_NUM_SENSORS).
    uint8_t giveCount(void) const { return RADAR_NUM_SENSORS; }

    /// Direktzugriff auf den Treiber — nur fuer Sonderfaelle (Temperatur, Diagnose).
    XM125Distance &giveSensor(uint8_t pos) { return _sensors[_clamp(pos)]; }

private:
    XM125Distance _sensors[RADAR_NUM_SENSORS];
    RadarData     _data[RADAR_NUM_SENSORS];

    bool     _ready[RADAR_NUM_SENSORS]      = {}; ///< Sensor ist eingerichtet
    bool     _pending[RADAR_NUM_SENSORS]    = {}; ///< Sensor misst gerade in dieser Runde
    uint8_t  _fails[RADAR_NUM_SENSORS]      = {}; ///< Fehlschlaege in Folge
    uint32_t _lastSetupMs[RADAR_NUM_SENSORS] = {}; ///< Bremse fuer den blockierenden Wiederanlauf

    TwoWire *_wire = &I2C_RADAR;

    static constexpr uint8_t RADAR_MAX_CONSEC_FAILURES = 5;

    static uint8_t _clamp(uint8_t pos) { return (pos < RADAR_NUM_SENSORS) ? pos : 0; }

    bool _setupSens(uint8_t idx);   ///< blockierende Einrichtung EINES Sensors
    void _startRound(void);         ///< Messung bei ALLEN Sensoren gleichzeitig anstossen
    void _takeResult(uint8_t idx);  ///< fertiges Ergebnis nach _data[idx] uebernehmen
    void _measureFailed(uint8_t idx); ///< Fehlerzaehler, markiert den Sensor ggf. als tot
    void _retrySetup(uint8_t idx);  ///< gedrosselter Wiederanlauf eines toten Sensors
};
