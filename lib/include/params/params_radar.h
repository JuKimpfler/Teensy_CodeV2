#pragma once
#include <Arduino.h>

/**
 * params_radar.h  --  XM125-Radarsensoren (lib/RadarArray, lib/XM125)
 *
 * Abstimmung auf eine gerade Holzwand bis 2,5 m: Eine planare Holzflaeche in
 * 0,2 m ... 2,5 m Abstand soll bei jeder Messung als Peak auftauchen. Holz
 * reflektiert deutlich schwaecher als Metall, deshalb sind Signalqualitaet und
 * CFAR-Empfindlichkeit gegenueber den Acconeer-Defaults angehoben.
 *
 * Nachmessen mit lib/XM125/examples/XM125_Calib_Sensitivity.cpp (Mode 's' =
 * Parameter-Sweep). Sieht der Sensor die Wand ueberhaupt nicht, vorher
 * XM125_Calib_Alignment.cpp (Ausrichtung) und XM125_Calib_Module.cpp
 * (Kalibrierung laeuft durch?).
 */

/* -------------- Verkabelung -------------- */
static constexpr uint8_t RADAR_NUM_SENSORS = 4;
static constexpr uint8_t RADAR_SENSOR_ADDRS[RADAR_NUM_SENSORS] = {0x58, 0x56, 0x55, 0x52}; ///< Reihenfolge = Index in RadarPos
static constexpr int8_t RADAR_WAKE_PIN = -1;                                               ///< WAKE_UP bewusst unverkabelt
static constexpr int8_t RADAR_INT_PIN = -1;                                                ///< MCU_INT bewusst unverkabelt

/* -------------- Abstimmung -------------- */
static constexpr uint32_t RADAR_START_MM = 150;            ///< naeher am Sensor liegt der Roboter selbst
static constexpr uint32_t RADAR_END_MM = 2700;             ///< 200 mm Reserve, damit 3000 mm nicht auf der Bereichskante liegt
static constexpr uint32_t RADAR_SIGNAL_QUALITY = 30000;    ///< 30.0 statt Default 15.0 -> mehr SNR am Bereichsende
static constexpr uint32_t RADAR_THRESHOLD_SENSITIVITY = 0; ///< 0.00 statt Default 0.50 -> niedrigste CFAR-Schwelle
static constexpr uint32_t RADAR_MEASURE_TIMEOUT_MS = 3000; ///< PROFILE5 + hohe Signalqualitaet braucht laenger als der 1000-ms-Default

/* -------------- Messrunden -------------- */
// Alle Sensoren messen GLEICHZEITIG: RadarArray::update() stoesst eine Runde
// bei allen Sensoren an und sieht in den Folgezyklen nur noch nach, ob die
// Ergebnisse da sind. Die Messdauer faellt einmal fuer alle an. Nachteil:
// Uebersprechen im selben Frequenzband -- nachmessen mit
// lib/XM125/examples/XM125_Calib_Crosstalk.cpp.
static constexpr uint8_t RADAR_PEAKS_TO_FETCH = 1;        ///< ausgewertet wird nur Peak 0; jeder Peak kostet zwei Registerzugriffe
static constexpr uint32_t RADAR_REINIT_INTERVAL_MS = 500; ///< Mindestabstand zwischen zwei Wiederanlaeufen eines toten Sensors (blockiert bis ~2 s)
static constexpr uint8_t RADAR_MAX_CONSEC_FAILURES = 5;   ///< so viele Fehlmessungen in Folge -> Sensor gilt als tot

/// Statusabfragen (je eine I2C-Transaktion pro Sensor) beginnen erst nach diesem Anteil der zuletzt
/// beobachteten Rundendauer. 1.0 = erst ganz zum Schluss, 0.0 = wie bisher in jedem Zyklus.
static constexpr float RADAR_POLL_START_FRACTION = 0.7f;

/* -------------- Ausreisserfilter (RadarArray::filterRadar) -------------- */
static constexpr float RADAR_FILTER_MAX_SPEED_MM_S = 3000.0f; ///< schneller kann sich der Abstand physikalisch nicht aendern (~1.27 m/s Roboter)
static constexpr float RADAR_FILTER_SPEED_MARGIN = 1.15f;     ///< Reserve auf die erlaubte Aenderung
static constexpr float RADAR_FILTER_NOISE_MM = 10.0f;         ///< Zuschlag fuer ~4 mm Messrauschen
static constexpr uint8_t RADAR_FILTER_REJECT_N = 10;          ///< so viele Ausreisser in Folge, dann wird kontrolliert nachgezogen
static constexpr float RADAR_FILTER_DT_MAX_S = 0.5f;          ///< laenger ohne Messung -> Filter ueberspringt den Schritt
static constexpr float RADAR_FILTER_ALPHA = 0.8f;             ///< Positionsgewicht: dem Messwert wird stark vertraut (geringes Rauschen)
static constexpr float RADAR_FILTER_BETA = 0.1f;              ///< Geschwindigkeitsgewicht
