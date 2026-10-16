#pragma once
#include <stddef.h>
#include <stdint.h>

#if defined(ARDUINO)
#include <Arduino.h>
#endif

/**
 * Link  --  Protokoll zwischen Teensy 4.0 und ESP32-S3 (UART, 8N1)
 *
 * Reines C++ ohne Arduino-Abhaengigkeit: dieselben Dateien laufen auf dem Teensy, dem ESP und im Rechnertest
 * (test/native). Rahmen:
 *
 *     0xAA 0x55 | Typ | Seq | Laenge | Nutzdaten (0 ... LINK_MAX_PAYLOAD) | CRC16 (LE)
 *
 * Die CRC (CRC-16/CCITT-FALSE, Startwert 0xFFFF) laeuft ueber Typ bis Nutzdaten. Alle Zahlen sind little-endian
 * und werden Byte fuer Byte geschrieben, nie als struct kopiert.
 */

static constexpr uint8_t LINK_SOF1 = 0xAA;
static constexpr uint8_t LINK_SOF2 = 0x55;
static constexpr uint8_t LINK_MAX_PAYLOAD = 64;
static constexpr uint8_t LINK_FRAME_OVERHEAD = 7; ///< 2 Beginn + Typ + Seq + Laenge + 2 CRC
static constexpr uint8_t LINK_MAX_FRAME = LINK_MAX_PAYLOAD + LINK_FRAME_OVERHEAD;

enum LinkType : uint8_t
{
    LinkImu = 0x01,   ///< ESP -> Teensy: Winkel und Drehrate
    LinkRadar = 0x02, ///< ESP -> Teensy: vier Radar-Messwerte
    LinkIr = 0x03,    ///< ESP -> Teensy: IR-Ring
    // 0x04 war fuer einen Maus-Sensor vorgesehen und entfaellt
    LinkInput = 0x05,  ///< ESP -> Teensy: Tasten
    LinkHealth = 0x10, ///< ESP -> Teensy: Zustand der ESP-Firmware
    LinkDebug = 0x11,  ///< ESP -> Teensy: Debugwert fuer PDS
    LinkHello = 0x80,  ///< beide: Version und Parameter-Pruefsumme
    LinkPing = 0x81,   ///< Teensy -> ESP: Zeitabgleich
    LinkPong = 0x82,   ///< ESP -> Teensy: Antwort mit den beiden ESP-Zeiten
    LinkCmd = 0x83,    ///< Teensy -> ESP: Befehl
    LinkAct = 0x84     ///< Teensy -> ESP: Servo, Piezo
};

enum LinkCmdId : uint8_t
{
    LinkCmdBnoCalibSave = 1, ///< BNO055-Kalibrierung im ESP-Speicher ablegen
    LinkCmdBnoCalibLoad = 2, ///< gespeicherte Kalibrierung laden
    LinkCmdBnoReset = 3,     ///< BNO055 neu starten
    LinkCmdRadarCalib = 4    ///< Radar neu kalibrieren (blockiert den ESP rund eine Sekunde je Sensor)
};

enum LinkRole : uint8_t
{
    LinkRoleTeensy = 0,
    LinkRoleEsp = 1
};

/* -------------- Nachrichten -------------- */
// Jede Nachricht: Size = Laenge der Nutzdaten, pack() schreibt Size Bytes, unpack() verlangt genau Size Bytes.

struct ImuMsg
{
    static constexpr uint8_t Size = 9;
    uint32_t TimeUs = 0;    ///< ESP-Zeit der Messung
    int16_t HeadingX16 = 0; ///< Winkel [1/16 Grad], 0 ... 5760, im Uhrzeigersinn positiv
    int16_t GyroZX16 = 0;   ///< Drehrate Z [1/16 Grad/s], gegen den Uhrzeigersinn positiv
    uint8_t Calib = 0;      ///< BNO055 CALIB_STAT (Bits: Sys, Gyro, Acc, Mag je 2 Bit)
    size_t pack(uint8_t *out) const;
    bool unpack(const uint8_t *in, size_t len);
};

struct RadarMsg
{
    static constexpr uint8_t Size = 38;
    uint32_t TimeUs = 0;      ///< ESP-Zeit: Mitte der Messrunde
    uint8_t ConBits = 0;      ///< Bit i = Sensor i antwortet (Reihenfolge RadarPos)
    uint8_t PeakBits = 0;     ///< Bit i = Sensor i hat einen Peak gesehen
    uint16_t Dist[4] = {};    ///< Rohwert von Peak 0, gefiltert
    uint16_t DistRaw[4] = {}; ///< derselbe Wert ungefiltert (Positionsfusion)
    int32_t Strength[4] = {}; ///< Signalstaerke [mdB]
    size_t pack(uint8_t *out) const;
    bool unpack(const uint8_t *in, size_t len);
};

struct IrMsg
{
    static constexpr uint8_t Size = 8;
    uint32_t TimeUs = 0;
    int16_t AngleRaw = 0; ///< Ballwinkel wie im Ring-Register: 1/180 Grad (Grad = AngleRaw / 180)
    uint16_t Dist = 0;    ///< Balldistanz wie im Ring-Register
    size_t pack(uint8_t *out) const;
    bool unpack(const uint8_t *in, size_t len);
};

struct InputMsg
{
    static constexpr uint8_t Size = 1;
    uint8_t Bits = 0; ///< Bit 0 = SWITCH2 (hoch = EIN)
    size_t pack(uint8_t *out) const;
    bool unpack(const uint8_t *in, size_t len);
};

struct HealthMsg
{
    static constexpr uint8_t Size = 15;
    uint32_t UptimeMs = 0;
    uint16_t MaxLoopUs = 0; ///< groesste Schleifenzeit seit dem letzten Rahmen
    uint16_t I2cErrors = 0;
    uint16_t RxErrors = 0; ///< CRC-/Laengenfehler auf der ESP-Seite
    uint32_t FreeHeap = 0;
    uint8_t Flags = 0; ///< Bit 0 = BNO055 antwortet
    size_t pack(uint8_t *out) const;
    bool unpack(const uint8_t *in, size_t len);
};

struct DebugMsg
{
    static constexpr uint8_t Size = 5;
    uint8_t Channel = 0;
    float Value = 0;
    size_t pack(uint8_t *out) const;
    bool unpack(const uint8_t *in, size_t len);
};

struct HelloMsg
{
    static constexpr uint8_t Size = 8;
    uint8_t Version = 0;
    uint8_t Role = 0;        ///< LinkRole
    uint32_t ParamsHash = 0; ///< Pruefsumme der gemeinsamen Parameter (params_link.h)
    uint16_t ImuHz = 0;
    size_t pack(uint8_t *out) const;
    bool unpack(const uint8_t *in, size_t len);
};

struct PingMsg
{
    static constexpr uint8_t Size = 4;
    uint32_t T0 = 0; ///< Teensy-Zeit beim Senden
    size_t pack(uint8_t *out) const;
    bool unpack(const uint8_t *in, size_t len);
};

struct PongMsg
{
    static constexpr uint8_t Size = 12;
    uint32_t T0 = 0; ///< Echo von PingMsg::T0
    uint32_t T1 = 0; ///< ESP-Zeit beim Empfang des Pings
    uint32_t T2 = 0; ///< ESP-Zeit beim Senden dieser Antwort
    size_t pack(uint8_t *out) const;
    bool unpack(const uint8_t *in, size_t len);
};

struct CmdMsg
{
    static constexpr uint8_t Size = 4;
    uint8_t Cmd = 0; ///< LinkCmdId
    uint8_t Arg = 0;
    uint16_t Val = 0;
    size_t pack(uint8_t *out) const;
    bool unpack(const uint8_t *in, size_t len);
};

struct ActMsg
{
    static constexpr uint8_t Size = 5;
    uint8_t Kind = 0; ///< 1 = Servo (Val1 = Mikrosekunden), 2 = Piezo (Val1 = Hz, Val2 = ms)
    uint16_t Val1 = 0;
    uint16_t Val2 = 0;
    size_t pack(uint8_t *out) const;
    bool unpack(const uint8_t *in, size_t len);
};

/* -------------- Rahmen bauen -------------- */
class LinkCodec
{
public:
    static uint16_t crc16(const uint8_t *data, size_t len, uint16_t init = 0xFFFF);

    /// Rahmen in out schreiben (mindestens LINK_MAX_FRAME Bytes). Rueckgabe: Rahmenlaenge, 0 bei zu langen Nutzdaten.
    static size_t encode(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len, uint8_t *out);
};

/* -------------- Rahmen lesen -------------- */
/// Byteweiser Parser. Fehlerhafte Rahmen werden verworfen und gezaehlt; danach sucht der Parser im bereits
/// gelesenen Strom nach dem naechsten gueltigen Beginn (nichts geht verloren, was ein gueltiger Rahmen war).
class LinkParser
{
public:
    /// Ein Byte einspeisen. true: ein gueltiger Rahmen liegt vor (type(), seq(), payload(), length()) bis zum naechsten feed().
    bool feed(uint8_t byte);
    void reset();

    uint8_t type() const { return _type; }
    uint8_t seq() const { return _seq; }
    const uint8_t *payload() const { return _payload; }
    uint8_t length() const { return _length; }

    uint32_t framesOk() const { return _framesOk; }
    uint32_t crcErrors() const { return _crcErrors; }
    uint32_t lengthErrors() const { return _lengthErrors; }
    uint32_t skippedBytes() const { return _skippedBytes; }
    uint32_t lostFrames() const { return _lostFrames; } ///< aus Luecken der Sequenznummer

private:
    uint8_t _buf[LINK_MAX_FRAME];
    uint8_t _n = 0;
    uint8_t _type = 0;
    uint8_t _seq = 0;
    uint8_t _length = 0;
    uint8_t _payload[LINK_MAX_PAYLOAD];
    uint8_t _lastSeq = 0;
    bool _haveSeq = false;
    uint32_t _framesOk = 0;
    uint32_t _crcErrors = 0;
    uint32_t _lengthErrors = 0;
    uint32_t _skippedBytes = 0;
    uint32_t _lostFrames = 0;

    void _drop(uint8_t count);
};

/* -------------- Zeitabgleich -------------- */
/// Rechnet Zeitstempel der Gegenseite in die eigene Zeit um (NTP-Prinzip mit vier Zeitpunkten, alles modulo 2^32).
/// Von den letzten Messungen gilt die mit der kuerzesten Laufzeit.
class LinkClock
{
public:
    static constexpr uint8_t Window = 8;

    void reset();
    /// t0 = eigene Sendezeit, t1 = Empfang beim Partner, t2 = Senden beim Partner, t3 = eigener Empfang (alle in us)
    void addSample(uint32_t t0, uint32_t t1, uint32_t t2, uint32_t t3);
    bool valid() const { return _count > 0; }
    /// Zeit des Partners -> eigene Zeit
    uint32_t toLocalUs(uint32_t remoteUs) const { return remoteUs - (uint32_t)_offset; }
    int32_t offsetUs() const { return _offset; }
    int32_t delayUs() const { return _delay; } ///< Laufzeit des besten Rundlaufs

private:
    struct Sample
    {
        int32_t Offset;
        int32_t Delay;
    };
    Sample _ring[Window];
    uint8_t _count = 0;
    uint8_t _head = 0;
    int32_t _offset = 0;
    int32_t _delay = 0;
};

/* -------------- Kurswinkel mit Drehrate weiterrechnen -------------- */
/// Haelt die letzte Winkelmessung und rechnet sie mit der Drehrate auf einen spaeteren Zeitpunkt weiter.
/// Das Vorzeichen zwischen Drehrate und Winkel wird aus den Messungen nachgelernt, startet aber mit gyroSignDefault.
class HeadingTracker
{
public:
    void begin(float gyroSignDefault, uint32_t extrapMaxUs);
    /// localUs = Zeit der Messung in eigener Zeit, Winkel in Grad (beliebiger Bereich), Drehrate in Grad/s
    void addSample(uint32_t localUs, float headingDeg, float gyroDps);
    bool valid() const { return _valid; }
    /// Winkel zum Zeitpunkt nowUs, 0 ... 360. Mehr als extrapMaxUs hinter der Messung wird nicht weitergerechnet.
    float headingAt(uint32_t nowUs) const;
    uint32_t ageUs(uint32_t nowUs) const { return nowUs - _lastUs; }
    float gyroSign() const { return _sign; }
    bool signLearned() const { return _learned; }

private:
    bool _valid = false;
    bool _learned = false;
    float _sign = -1.0f;
    float _heading = 0;
    float _gyro = 0;
    uint32_t _lastUs = 0;
    uint32_t _extrapMaxUs = 25000;
    int16_t _score = 0; ///< Stimmen fuer das Vorzeichen (positiv: Winkel steigt mit der Drehrate)
};

/// Winkeldifferenz auf -180 ... 180
float linkWrap180(float deg);

/* -------------- Anschluss und Knoten -------------- */
/// Was der Link von der UART braucht. Auf dem Teensy und dem ESP ist das ein Stream (StreamLinkPort), im Rechnertest ein Puffer.
class LinkPort
{
public:
    virtual ~LinkPort() {}
    virtual int readByte() = 0; ///< naechstes Byte oder -1
    virtual size_t writeBytes(const uint8_t *data, size_t len) = 0;
    virtual int writeSpace() { return 1 << 30; } ///< so viele Bytes passen, ohne zu blockieren
};

/// Sendezaehler und Parser an einem Port. Senden blockiert nie: passt der Rahmen nicht in den Sendepuffer, wird er verworfen.
class LinkNode
{
public:
    void begin(LinkPort *port);
    /// Liest hoechstens maxBytes aus dem Port. true: ein gueltiger Rahmen liegt in rx() vor; danach erneut aufrufen.
    bool nextFrame(uint16_t maxBytes = 512);
    const LinkParser &rx() const { return _parser; }

    bool sendRaw(uint8_t type, const uint8_t *payload, uint8_t len);
    template <class Msg>
    bool send(uint8_t type, const Msg &msg)
    {
        uint8_t pl[LINK_MAX_PAYLOAD];
        return sendRaw(type, pl, (uint8_t)msg.pack(pl));
    }
    uint32_t framesSent() const { return _framesSent; }
    uint32_t sendDropped() const { return _sendDropped; }

private:
    LinkPort *_port = nullptr;
    LinkParser _parser;
    uint8_t _seq = 0;
    uint32_t _framesSent = 0;
    uint32_t _sendDropped = 0;
};

/// Die ESP-Seite des Protokolls: beantwortet HELLO und PING und reicht Befehle an die Firmware weiter.
class LinkEsp
{
public:
    void begin(LinkPort *port, uint8_t version, uint32_t paramsHash, uint16_t imuHz);
    /// Eingehende Rahmen verarbeiten und HELLO senden, solange der Teensy noch nicht geantwortet hat. nowUs = ESP-Zeit.
    void poll(uint32_t nowUs);

    template <class Msg>
    bool send(uint8_t type, const Msg &msg) { return _node.send(type, msg); }
    bool takeCmd(CmdMsg &out);
    bool takeAct(ActMsg &out);
    bool teensyKnown() const { return _peerHello; }
    bool paramsMismatch() const { return _mismatch; }
    uint32_t rxErrors() const { return _node.rx().crcErrors() + _node.rx().lengthErrors(); }

private:
    LinkNode _node;
    uint8_t _version = 0;
    uint32_t _hash = 0;
    uint16_t _imuHz = 0;
    uint32_t _lastHelloUs = 0;
    bool _sentHello = false;
    bool _peerHello = false;
    bool _mismatch = false;
    CmdMsg _cmd;
    bool _haveCmd = false;
    ActMsg _act;
    bool _haveAct = false;

    void _sendHello();
};

#if defined(ARDUINO)
/// LinkPort auf einem Arduino-Stream (HardwareSerial)
class StreamLinkPort : public LinkPort
{
public:
    explicit StreamLinkPort(Stream &stream) : _s(&stream) {}
    int readByte() override { return _s->read(); }
    size_t writeBytes(const uint8_t *data, size_t len) override { return _s->write(data, len); }
    int writeSpace() override { return _s->availableForWrite(); }

private:
    Stream *_s;
};
#endif
