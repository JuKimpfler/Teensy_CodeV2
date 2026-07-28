#pragma once

/**
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  XM125.h  —  Acconeer XM125 (A121) Radar Sensor — I²C Basis-Klasse       ║
 * ╠══════════════════════════════════════════════════════════════════════════╣
 * ║  Das XM125-Modul stellt zwei fertige Applikationen bereit, die per      ║
 * ║  I²C-Register angesprochen werden:                                      ║
 * ║   • Distance Detector   → siehe XM125Distance.h                         ║
 * ║   • Presence Detector   → siehe XM125Presence.h                         ║
 * ║  Beide teilen sich das gleiche Low-Level-Protokoll (16-bit Register-    ║
 * ║  adresse + 32-bit Registerwert, Big-Endian), das hier implementiert     ║
 * ║  ist.                                                                    ║
 * ║                                                                          ║
 * ║  Quelle Registerprotokoll: Acconeer "I2C Distance/Presence Detector     ║
 * ║  User Guide" (siehe Datenblatt XM125, Kap. Register Interface).         ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * Ablauf (lt. Acconeer User Guide) für jede Messung:
 *   1. WAKE_UP-Pin high setzen (falls verkabelt) und auf MCU_INT high warten
 *   2. Detector Status lesen, prüfen dass BUSY und ERROR nicht gesetzt sind
 *   3. Konfigurationsregister schreiben
 *   4. APPLY_CONFIGURATION(_AND_CALIBRATE) Kommando senden, auf BUSY=0 pollen
 *   5. Start/Measure-Kommando senden, auf BUSY=0 pollen
 *   6. Ergebnisregister lesen
 *
 * Wichtig: Wire.begin() (bzw. Wire1.begin() etc.) VOR begin() aufrufen!
 *
 * ── WAKE_UP-Pin bei Dauerbetrieb ohne Verkabelung ─────────────────────────
 * WAKE_UP weckt die interne STM32-MCU des Moduls aus dem Hibernate (High =
 * wach, Low = Schlaf). Der Schlafzustand wird ausschließlich durch einen
 * expliziten Host-Eingriff ausgelöst (Pin aktiv auf Low ziehen bzw. ein
 * entsprechendes Low-Power-Kommando) — das Modul schläft NICHT von selbst.
 * Diese Bibliothek besitzt bewusst kein Low-Power-/Sleep-Kommando, und
 * `sleep()` wirkt nur, wenn tatsächlich ein wakePin verkabelt ist. Bleibt
 * WAKE_UP unverkabelt (floatend, wie in diesem Projekt), bleibt das Modul
 * dadurch dauerhaft wach, solange nichts anderes den Pin aktiv auf Low
 * zieht. Restrisiko: Ein offener, floatender Eingang ist grundsätzlich
 * anfälliger für Störeinkopplungen (EMI) als ein fest beschalteter — das
 * genaue Verhalten der internen State-Machine bei Boot/Reset ist aus der
 * öffentlichen Doku nicht zu 100% ableitbar. Für Dauerbetrieb am Netzteil
 * wurde dieses Restrisiko bewusst in Kauf genommen (kein Pull-up verbaut).
 */

#include <Arduino.h>
#include <Wire.h>

// ── I²C Adresse ───────────────────────────────────────────────────────────────
static constexpr uint8_t XM125_I2C_ADDR_DEFAULT = 0x55; ///< Werksadresse (nicht änderbar per Software)

// ── Gemeinsame Register (identisch bei Distance- & Presence-Firmware) ────────
static constexpr uint16_t XM125_REG_VERSION          = 0x0000; ///< R: RSS-Version
static constexpr uint16_t XM125_REG_PROTOCOL_STATUS  = 0x0001; ///< R: Protokoll-Fehlerflags
static constexpr uint16_t XM125_REG_MEASURE_COUNTER  = 0x0002; ///< R: Anzahl Messungen seit Neustart
static constexpr uint16_t XM125_REG_DETECTOR_STATUS  = 0x0003; ///< R: Status-/Fehlerflags des Detektors
static constexpr uint16_t XM125_REG_COMMAND          = 0x0100; ///< W: Kommandoregister

// ── Gemeinsame Detector-Status Bits ───────────────────────────────────────────
// (bitgenaue Bedeutung der übrigen Bits unterscheidet sich zwischen Distance-
//  und Presence-Firmware, siehe jeweilige abgeleitete Klasse)
static constexpr uint32_t XM125_STATUS_DETECTOR_ERROR = (1UL << 28); ///< Detektor-Fehler, Neustart nötig
static constexpr uint32_t XM125_STATUS_BUSY            = (1UL << 31); ///< Kommando wird noch ausgeführt

// ── Kommandos, die in beiden Applikationen identisch sind ────────────────────
static constexpr uint32_t XM125_CMD_ENABLE_UART_LOGS  = 32;
static constexpr uint32_t XM125_CMD_DISABLE_UART_LOGS = 33;
static constexpr uint32_t XM125_CMD_LOG_CONFIGURATION = 34;
static constexpr uint32_t XM125_CMD_RESET_MODULE      = 1381192737UL; ///< "Magic Number" lt. Datenblatt

// ── Timeouts ──────────────────────────────────────────────────────────────────
static constexpr uint32_t XM125_DEFAULT_APPLY_TIMEOUT_MS   = 3000; ///< Kalibrierung kann bis zu ~1-2s dauern
static constexpr uint32_t XM125_DEFAULT_MEASURE_TIMEOUT_MS = 1000;
static constexpr uint32_t XM125_WAKEUP_TIMEOUT_MS          = 1500;

// ═════════════════════════════════════════════════════════════════════════════
//  XM125Base  —  Low-Level I²C-Register-Zugriff, von Distance/Presence geerbt
// ═════════════════════════════════════════════════════════════════════════════
class XM125Base {
public:
    /**
     * @param i2cAddr  I²C-Adresse (Werkseinstellung: 0x52)
     * @param wakePin  Pin am MCU, der mit WAKE_UP des XM125 verbunden ist (-1 = nicht verkabelt,
     *                 Sensor bleibt dauerhaft aktiv / WU-Jumper auf 3.3V)
     * @param intPin   Pin am MCU, der mit MCU_INT des XM125 verbunden ist (-1 = nicht verkabelt,
     *                 Ready-Zustand wird dann nur über Busy-Polling per I²C bestimmt)
     */
    explicit XM125Base(uint8_t i2cAddr = XM125_I2C_ADDR_DEFAULT, int8_t wakePin = -1, int8_t intPin = -1);

    /** Initialisiert Pins (falls verkabelt) und prüft I²C-Erreichbarkeit. */
    bool begin(TwoWire& wire = Wire);

    /** Prüft ob der Sensor unter der konfigurierten Adresse antwortet. */
    bool isConnected();

    /**
     * Setzt WAKE_UP high (falls verkabelt) und wartet bis der Sensor bereit ist.
     * Ist MCU_INT verkabelt, wird auf dessen HIGH-Flanke gewartet.
     * Sind weder WAKE_UP noch MCU_INT verkabelt, wird stattdessen bis zum
     * Timeout wiederholt per I²C geprüft, ob der Sensor bereits antwortet
     * (reine Boot-/Anlauf-Absicherung, z.B. nach Power-On) — da das Modul
     * ohne aktives Sleep-Kommando ohnehin durchgehend wach bleibt, ist dies
     * kein "Aufwecken" im eigentlichen Sinn, siehe Hinweis oben im Datei-Header.
     */
    bool wakeUp(uint32_t timeoutMs = XM125_WAKEUP_TIMEOUT_MS);

    /**
     * Setzt WAKE_UP low (falls verkabelt) — Low-Power-Modus.
     * Wirkungslos wenn kein wakePin verkabelt ist (siehe Header-Kommentar).
     * ACHTUNG: Auf Hardware ohne verkabelten wakePin niemals einen
     * eigenen Sleep-Mechanismus ergänzen — ohne Rückweg über den Pin
     * wäre das Modul dann nur noch per Power-Cycle erreichbar.
     */
    void sleep();

    // ── Gemeinsame Register ───────────────────────────────────────────────
    uint32_t getVersionRaw();          ///< Reg 0x0000, Rohwert (Bit-Layout lt. Datenblatt prüfen)
    uint32_t getProtocolStatus();      ///< Reg 0x0001, 0 = kein Protokollfehler
    uint32_t getMeasureCounter();      ///< Reg 0x0002
    uint32_t getDetectorStatus();      ///< Reg 0x0003, Rohwert (siehe abgeleitete Klasse für Bits)

    /** true wenn BUSY-Bit (31) im zuletzt gelesenen Status gesetzt ist. */
    static bool statusBusy(uint32_t status)  { return (status & XM125_STATUS_BUSY) != 0; }
    /** true wenn DETECTOR_ERROR-Bit (28) im zuletzt gelesenen Status gesetzt ist. */
    static bool statusError(uint32_t status) { return (status & XM125_STATUS_DETECTOR_ERROR) != 0; }

    uint8_t getAddress() const { return _addr; }

protected:
    TwoWire* _wire;
    uint8_t  _addr;
    int8_t   _wakePin;
    int8_t   _intPin;

    /** Schreibt ein 32-bit Register. reg/value werden Big-Endian übertragen. */
    bool _writeReg(uint16_t reg, uint32_t value);
    /** Liest ein 32-bit Register (Repeated-Start, dann 4 Bytes). */
    bool _readReg(uint16_t reg, uint32_t& value);

    /** Sendet Kommando und wartet bis BUSY-Bit im Detector Status zurückgeht. */
    bool _executeCommand(uint32_t cmd, uint32_t timeoutMs);
    /** Pollt Detector Status bis BUSY=0 oder Timeout. Gibt letzten Status zurück. */
    bool _waitNotBusy(uint32_t timeoutMs, uint32_t& lastStatus);
};
