#pragma once
#include <Arduino.h>

#define I2C_BNO Wire
#define I2C_IR Wire
#define I2C_SW Wire
#define I2C_US Wire
#define I2C_RADAR Wire

#define BNO_ADDRESS 0x28

static constexpr uint8_t RADAR_NUM_SENSORS = 4;
static constexpr uint8_t RADAR_SENSOR_ADDRS[RADAR_NUM_SENSORS] = { 0x51, 0x52, 0x53, 0x54 };
static constexpr int8_t  RADAR_WAKE_PIN = -1;
static constexpr int8_t  RADAR_INT_PIN  = -1;

// ── Radar-Abstimmung: gerade Holzwand bis 2,5 m ───────────────────────────────
// Zielbild: eine planare Holzflaeche in 0,2 m ... 2,5 m Abstand soll bei jeder
// Messung als Peak auftauchen. Holz reflektiert deutlich schwaecher als Metall,
// deshalb sind Signalqualitaet und CFAR-Empfindlichkeit gegenueber den
// Acconeer-Defaults angehoben. Ermittelt/nachpruefbar mit dem Sketch
// test/XM125_WallCalibration.cpp (Mode 's' = Parameter-Sweep).
static constexpr uint32_t RADAR_START_MM = 200;   ///< naeher am Sensor liegt der Roboter selbst
static constexpr uint32_t RADAR_END_MM   = 2700;  ///< 200 mm Reserve, damit 2500 mm nicht auf der Bereichskante liegt
static constexpr uint32_t RADAR_SIGNAL_QUALITY        = 25000; ///< 25.0 statt Default 15.0 -> mehr SNR am Bereichsende
static constexpr uint32_t RADAR_THRESHOLD_SENSITIVITY = 750;   ///< 0.75 statt Default 0.50 -> CFAR laesst schwache Wand-Peaks durch
static constexpr uint32_t RADAR_MEASURE_TIMEOUT_MS    = 3000;  ///< PROFILE5 + hohe Signalqualitaet braucht laenger als der 1000ms-Default

// Wie viele Sensoren gleichzeitig messen duerfen (systemlib::readRadar).
// Die Wartezeit auf das Messergebnis faellt einmal pro Gruppe an statt einmal
// pro Sensor -- Gruppengroesse 2 halbiert also grob die Radar-Zeit im Zyklus.
// Gleichzeitig senden aber auch alle Sensoren einer Gruppe gleichzeitig, was
// Uebersprechen ermoeglicht. Die Gruppen werden deshalb so gebildet, dass die
// am weitesten voneinander weg blickenden Sensoren zusammen messen:
//   1 = nacheinander           (kein Uebersprechen moeglich, langsamste Variante)
//   2 = Vo+Hi, danach Li+Re    (Paare blicken entgegengesetzt -- Standard)
//   4 = alle gleichzeitig      (schnellste Variante, hoechstes Uebersprech-Risiko)
// Nachmessen ob 4 noch sauber laeuft: test/XM125_WallCalibration.cpp, Mode 'i'.
static constexpr uint8_t RADAR_PARALLEL_GROUP = 2;
static_assert(RADAR_NUM_SENSORS % RADAR_PARALLEL_GROUP == 0,
              "RADAR_PARALLEL_GROUP muss RADAR_NUM_SENSORS teilen");

