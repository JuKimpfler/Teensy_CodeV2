#pragma once
#include <Arduino.h>

/**
 * params_system.h  --  Busse, Adressen, Debug-Modus, Regelzyklus
 *
 * Pins stehen in enum.h, Sensor-Abstimmung in params_radar.h, alles zur
 * Position in params_position.h. Namensschema und Einheiten: docs/Coding_Rules.md.
 */

/* -------------- I2C-Zuordnung -------------- */
// Alle Aliase zeigen aktuell auf Wire. Ein Sensor zieht auf den anderen Bus
// um, indem hier das Alias geaendert wird (z.B. #define I2C_US Wire1).
#define I2C_TNS Wire1 // Teensy-Bus (SDA/SCL_TNS_3V3, Pins 16/17): Port-Expander, INA219
#define I2C_BNO Wire  // ESP-Bus (SDA/SCL_ESP_3V3, Pins 18/19), nur mit geschlossenen Jumpern SDA_Jumper/SCL_Jumper
#define I2C_IR Wire   // wie I2C_BNO; steckt der IR-Ring am Teensy-Intercon J13, hier Wire1 eintragen
#define I2C_US Wire
#define I2C_RADAR Wire // wie I2C_IR; am Teensy-Bus (Wire1) laeuft das Radar nur mit 100 kHz, siehe I2C_WIRE1_CLOCK_HZ

static constexpr uint8_t BNO_ADDRESS = 0x28; ///< I2C-Adresse des BNO055

/* -------------- I2C-Takte -------------- */
// Wire: BNO055, IR-Ring, Schalter-Expander und die vier XM125-Radare. Das Radar streckt den Takt
// (Clock-Stretching) und mag hohe Takte nicht, siehe lib/XM125/doc/11-Hardware-und-Integration.md.
static constexpr uint32_t I2C_WIRE_CLOCK_HZ = 100000;
// Wire1: nur Port-Expander (TCA9534, max. 400 kHz) und die beiden INA219 -- kein Clock-Stretching.
// Bei Lesefehlern an Wire1 (Akku-/Motorwerte 0, Schalter springen) zurueck auf 100000.
static constexpr uint32_t I2C_WIRE1_CLOCK_HZ = 400000;

/// Die Motor-/System-Expander werden nur bei Aenderung des Ausgabebytes geschrieben, spaetestens aber nach so vielen ms.
static constexpr uint32_t MOTOR_EXP_REFRESH_MS = 50;

/* -------------- Strommessung (INA219, Blatt 2 und 13) -------------- */
static constexpr float INA_MAX_CURRENT_A = 15.0f;    ///< Messbereich je Chip (wie bisher)
static constexpr float INA_MAIN_SHUNT_OHM = 0.005f;  ///< 0x40, Akku
static constexpr float INA_MOTOR_SHUNT_OHM = 0.005f; ///< 0x41, Motorversorgung
static constexpr float INA_DRIB_SHUNT_OHM = 0.010f;  ///< 0x45, Dribbler

/* -------------- Freigaben ueber den System-Expander (0x23) -------------- */
/// LINE_EN_T: Pegel, mit dem die Linien-Platine freigegeben wird. -1 = nicht treiben (Pin bleibt Eingang).
/// Die Polaritaet ergibt sich aus dem Schaltplan der Linien-Platine und ist hier nicht bekannt.
static constexpr int8_t LINE_EN_LEVEL = -1;

/* -------------- Debug-Modus -------------- */
// D_Mode_Game: Build ohne Debug-Signale (Laufzeit-Optimierung)
// D_Mode_Test: Build mit PDS / seriellen Ausgaben (kann die Laufzeit verlaengern)
// Umschalten per build_flags in platformio.ini:  -DDPDS_Mode=D_Mode_Game
#define D_Mode_Test 0
#define D_Mode_Game 1

#ifndef DPDS_Mode
#define DPDS_Mode D_Mode_Test
#endif

/* -------------- Linien-Sensoren -------------- */
static constexpr uint8_t LINE_NUM_SENSORS = 48;

/// Schwelle je Linien-Sensor (ADC-Counts), 6 Expander a 8 Kanaele.
static constexpr uint16_t LINE_THRESHOLD[LINE_NUM_SENSORS] =
    {
        3000,
        3000,
        3000,
        3000,
        3000,
        3000,
        3000,
        3000, // pex1
        3000,
        3000,
        3000,
        3000,
        3000,
        3000,
        3000,
        3000, // pex2
        3000,
        3000,
        3000,
        3000,
        3000,
        3000,
        3000,
        3000, // pex3
        3000,
        3000,
        3000,
        3000,
        3000,
        3000,
        3000,
        3000, // pex4
        3000,
        3000,
        3000,
        3000,
        3000,
        3000,
        3000,
        3000, // pex5
        3000,
        3000,
        3000,
        3000,
        3000,
        3000,
        3000,
        3000, // pex6
};

/* -------------- Kurs-Regler (Heading) -------------- */
// Stellgroesse omega_cmd liegt auf -1..1, wird in systemlib mit 100 skaliert.
static constexpr float HDG_TS_NOMINAL_S = 0.005f; ///< nur Fallback fuer cfg.Ts, die echte Zykluszeit kommt aus update()
static constexpr float HDG_DT_MIN_S = 1e-5f;      ///< 10 us -- gegen dt = 0
static constexpr float HDG_DT_MAX_S = 0.03f;      ///< 30 ms -- deckelt Ausreisser, falls das Hauptprogramm laenger braucht
static constexpr float HDG_U_MIN = -1.0f;
static constexpr float HDG_U_MAX = 1.0f;
static constexpr float HDG_RATE_LIMIT = 6.0f; ///< begrenzt die Aenderungsrate der Stellgroesse (sanfter Anlauf)

// PID (Standard). Start: nur P, dann I/D schrittweise zuschalten.
static constexpr float HDG_PID_KP = 0.2f;
static constexpr float HDG_PID_KI = 0.0f;
static constexpr float HDG_PID_KD = 0.016f;
static constexpr float HDG_PID_TF_S = 0.02f; ///< D-Anteil filtern (Winkelmessung ist meist verrauscht)

// ADRC (Alternative, setupHeadingController(true)). Winkel = Strecke 2. Ordnung.
static constexpr float HDG_ADRC_WC = 15.0f; ///< Reglerbandbreite: wie schnell ausgerichtet wird
static constexpr float HDG_ADRC_WO = 80.0f; ///< Beobachterbandbreite: ca. 5x wc als Startwert
static constexpr float HDG_ADRC_B0 = 8.0f;  ///< grobe Schaetzung: Wirkung von omega_cmd auf die Winkelbeschleunigung -> per Test anpassen
