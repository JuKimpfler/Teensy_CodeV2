#pragma once
#include <Arduino.h>

#define I2C_BNO Wire
#define I2C_IR Wire
#define I2C_SW Wire
#define I2C_US Wire
#define I2C_RADAR Wire

#define BNO_ADDRESS 0x28

static constexpr uint8_t RADAR_NUM_SENSORS = 4;
static constexpr uint8_t RADAR_SENSOR_ADDRS[RADAR_NUM_SENSORS] = { 0x58,0x56,0x54,0x52 };
static constexpr int8_t  RADAR_WAKE_PIN = -1;
static constexpr int8_t  RADAR_INT_PIN  = -1;

// ── Radar-Abstimmung: gerade Holzwand bis 2,5 m ───────────────────────────────
// Zielbild: eine planare Holzflaeche in 0,2 m ... 2,5 m Abstand soll bei jeder
// Messung als Peak auftauchen. Holz reflektiert deutlich schwaecher als Metall,
// deshalb sind Signalqualitaet und CFAR-Empfindlichkeit gegenueber den
// Acconeer-Defaults angehoben. Ermittelt/nachpruefbar mit dem Sketch
// lib/XM125/examples/XM125_Calib_Sensitivity.cpp (Mode 's' = Parameter-Sweep). Sieht der
// Sensor die Wand ueberhaupt nicht, vorher lib/XM125/examples/XM125_Calib_Alignment.cpp
// (Ausrichtung) und lib/XM125/examples/XM125_Calib_Module.cpp (Kalibrierung laeuft durch?).
static constexpr uint32_t RADAR_START_MM = 200;   ///< naeher am Sensor liegt der Roboter selbst
static constexpr uint32_t RADAR_END_MM   = 3000;  ///< 200 mm Reserve, damit 2500 mm nicht auf der Bereichskante liegt
static constexpr uint32_t RADAR_SIGNAL_QUALITY        = 35000; ///< 25.0 statt Default 15.0 -> mehr SNR am Bereichsende
static constexpr uint32_t RADAR_THRESHOLD_SENSITIVITY = 400;   ///< 0.75 statt Default 0.50 -> CFAR laesst schwache Wand-Peaks durch
static constexpr uint32_t RADAR_MEASURE_TIMEOUT_MS    = 3000;  ///< PROFILE5 + hohe Signalqualitaet braucht laenger als der 1000ms-Default
//static constexpr uint32_t RADAR_THRESHOLD_STRENGHT = 23;
// Alle Sensoren messen grundsaetzlich GLEICHZEITIG: systemlib::readRadar()
// stoesst eine Runde bei allen Sensoren an und sieht in den Folgezyklen nur
// noch nach, ob die Ergebnisse da sind. Gewartet wird nirgends, die Messdauer
// faellt also einmal fuer alle Sensoren an statt einmal pro Sensor.
// Nachteil: alle Module senden im selben Frequenzband zur selben Zeit, also
// ist Uebersprechen moeglich. Nachmessen, ob das die Trefferquote kostet:
// lib/XM125/examples/XM125_Calib_Crosstalk.cpp (Mode 'g' vergleicht
// nacheinander gegen gleichzeitig, 'i'/'p' zeigen den einzelnen Stoerer).
// Nur wie viele PEAKS je Messung aus dem Sensor geholt werden, ist noch eine
// Stellschraube -- jeder Peak kostet zwei Registerlesezugriffe:
static constexpr uint8_t RADAR_PEAKS_TO_FETCH = 1; ///< ausgewertet wird nur Peak 0
// Ein nicht erreichbarer Sensor wird neu initialisiert -- das blockiert
// (Kalibrierung dauert bis ~2 s), darf also nicht in jedem Zyklus passieren:
static constexpr uint32_t RADAR_REINIT_INTERVAL_MS = 2000;

