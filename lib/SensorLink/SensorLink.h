#pragma once
#include <Link.h>
#include <params.h>

/**
 * SensorLink  --  Teensy-Seite des Links zum ESP32-S3
 *
 * Nimmt die Messungen des ESP entgegen (Winkel, Radar, IR-Ring, Tasten, Zustand), gleicht die Zeit ab und
 * rechnet den Winkel mit der Drehrate auf den Abfragezeitpunkt weiter. Alle Funktionen kehren sofort zurueck
 * und blockieren nie; die Zeit (Mikrosekunden, micros()) wird von aussen uebergeben.
 *
 *   SensorLink EspLink;
 *   StreamLinkPort port(UART_ESP);        // UART_ESP.begin(LINK_BAUD) vorher
 *   EspLink.begin(&port, micros());
 *   ...  jeden Zyklus:  EspLink.update(micros());  if (EspLink.giveHeadingValid(micros())) h = EspLink.giveHeading(micros());
 */
class SensorLink
{
public:
    /// Beginnt den Verbindungsaufbau (HELLO). port muss bis zum Programmende leben.
    void begin(LinkPort *port, uint32_t nowUs);

    /// Jeden Zyklus: Rahmen lesen, verarbeiten, HELLO/PING bei Bedarf senden.
    void update(uint32_t nowUs);

    /* -------------- Zustand -------------- */
    bool isAlive(uint32_t nowUs) const;  ///< in den letzten LINK_ALIVE_MS kam ein gueltiger Rahmen
    bool isUsable(uint32_t nowUs) const; ///< lebt, Handshake fertig, Version und Parameter-Pruefsumme passen
    bool giveHandshakeDone() const { return _espHello; }
    bool giveParamsMismatch() const { return _mismatch; } ///< Version oder Parameter-Pruefsumme passen nicht zum ESP
    bool giveBootTimeout(uint32_t nowUs) const;           ///< nach LINK_BOOT_TIMEOUT_MS noch kein HELLO vom ESP

    /* -------------- Winkel -------------- */
    bool giveHeadingValid(uint32_t nowUs) const; ///< Link lebt und die letzte Winkelmessung ist jung
    float giveHeading(uint32_t nowUs) const;     ///< Grad, 0 ... 360, im Uhrzeigersinn positiv, auf nowUs weitergerechnet
    uint32_t giveHeadingAgeUs(uint32_t nowUs) const { return _heading.valid() ? _heading.ageUs(nowUs) : 0xFFFFFFFFu; }
    uint8_t giveImuCalib() const { return _imuCalib; }
    float giveGyroSign() const { return _heading.gyroSign(); }

    /* -------------- Radar, IR-Ring, Tasten -------------- */
    /// true genau einmal je neuer Radar-Runde; localUs = Mitte der Messrunde in Teensy-Zeit (micros())
    bool takeRadar(RadarMsg &out, uint32_t &localUs);
    /// true genau einmal je neuer IR-Messung; localUs = Messzeit in Teensy-Zeit
    bool takeIr(IrMsg &out, uint32_t &localUs);
    uint8_t giveInputBits() const { return _inputBits; } ///< Bit 0 = SWITCH2
    bool giveSwitch2() const { return (_inputBits & 1u) != 0; }
    const HealthMsg &giveHealth() const { return _health; }

    /* -------------- Zeit und Zaehler -------------- */
    bool giveClockValid() const { return _clock.valid(); }
    int32_t giveClockOffsetUs() const { return _clock.offsetUs(); }
    int32_t giveClockDelayUs() const { return _clock.delayUs(); }
    uint32_t giveFramesOk() const { return _node.rx().framesOk(); }
    uint32_t giveCrcErrors() const { return _node.rx().crcErrors(); }
    uint32_t giveLostFrames() const { return _node.rx().lostFrames(); }

    /* -------------- Befehle an den ESP -------------- */
    bool sendCmd(uint8_t cmd, uint8_t arg = 0, uint16_t val = 0);
    bool sendServo(uint16_t microseconds);
    bool sendPiezo(uint16_t hz, uint16_t ms);

private:
    LinkNode _node;
    LinkClock _clock;
    HeadingTracker _heading;
    uint32_t _beginUs = 0;
    uint32_t _lastRxUs = 0;
    bool _anyRx = false;
    bool _espHello = false;
    bool _mismatch = false;
    uint32_t _lastHelloUs = 0;
    uint32_t _lastPingUs = 0;
    uint8_t _pingCount = 0;
    uint8_t _imuCalib = 0;
    uint8_t _inputBits = 0;
    RadarMsg _radar;
    uint32_t _radarLocalUs = 0;
    bool _radarNew = false;
    IrMsg _ir;
    uint32_t _irLocalUs = 0;
    bool _irNew = false;
    HealthMsg _health;

    void _sendHello();
    void _onFrame(uint32_t nowUs);
    uint32_t _toLocal(uint32_t espUs, uint32_t nowUs) const;
};
