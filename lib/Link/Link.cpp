#include "Link.h"
#include <math.h>
#include <string.h>

namespace
{
inline void putU16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}
inline void putU32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)(v >> 24);
}
inline uint16_t getU16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline uint32_t getU32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
} // namespace

/* -------------- Nachrichten -------------- */

size_t ImuMsg::pack(uint8_t *o) const
{
    putU32(o, TimeUs);
    putU16(o + 4, (uint16_t)HeadingX16);
    putU16(o + 6, (uint16_t)GyroZX16);
    o[8] = Calib;
    return Size;
}
bool ImuMsg::unpack(const uint8_t *i, size_t len)
{
    if (len != Size)
        return false;
    TimeUs = getU32(i);
    HeadingX16 = (int16_t)getU16(i + 4);
    GyroZX16 = (int16_t)getU16(i + 6);
    Calib = i[8];
    return true;
}

size_t RadarMsg::pack(uint8_t *o) const
{
    putU32(o, TimeUs);
    o[4] = ConBits;
    o[5] = PeakBits;
    for (int k = 0; k < 4; k++)
    {
        putU16(o + 6 + 2 * k, Dist[k]);
        putU16(o + 14 + 2 * k, DistRaw[k]);
        putU32(o + 22 + 4 * k, (uint32_t)Strength[k]);
    }
    return Size;
}
bool RadarMsg::unpack(const uint8_t *i, size_t len)
{
    if (len != Size)
        return false;
    TimeUs = getU32(i);
    ConBits = i[4];
    PeakBits = i[5];
    for (int k = 0; k < 4; k++)
    {
        Dist[k] = getU16(i + 6 + 2 * k);
        DistRaw[k] = getU16(i + 14 + 2 * k);
        Strength[k] = (int32_t)getU32(i + 22 + 4 * k);
    }
    return true;
}

size_t IrMsg::pack(uint8_t *o) const
{
    putU32(o, TimeUs);
    putU16(o + 4, (uint16_t)AngleRaw);
    putU16(o + 6, Dist);
    return Size;
}
bool IrMsg::unpack(const uint8_t *i, size_t len)
{
    if (len != Size)
        return false;
    TimeUs = getU32(i);
    AngleRaw = (int16_t)getU16(i + 4);
    Dist = getU16(i + 6);
    return true;
}

size_t InputMsg::pack(uint8_t *o) const
{
    o[0] = Bits;
    return Size;
}
bool InputMsg::unpack(const uint8_t *i, size_t len)
{
    if (len != Size)
        return false;
    Bits = i[0];
    return true;
}

size_t HealthMsg::pack(uint8_t *o) const
{
    putU32(o, UptimeMs);
    putU16(o + 4, MaxLoopUs);
    putU16(o + 6, I2cErrors);
    putU16(o + 8, RxErrors);
    putU32(o + 10, FreeHeap);
    o[14] = Flags;
    return Size;
}
bool HealthMsg::unpack(const uint8_t *i, size_t len)
{
    if (len != Size)
        return false;
    UptimeMs = getU32(i);
    MaxLoopUs = getU16(i + 4);
    I2cErrors = getU16(i + 6);
    RxErrors = getU16(i + 8);
    FreeHeap = getU32(i + 10);
    Flags = i[14];
    return true;
}

size_t DebugMsg::pack(uint8_t *o) const
{
    o[0] = Channel;
    uint32_t bits;
    memcpy(&bits, &Value, 4);
    putU32(o + 1, bits);
    return Size;
}
bool DebugMsg::unpack(const uint8_t *i, size_t len)
{
    if (len != Size)
        return false;
    Channel = i[0];
    uint32_t bits = getU32(i + 1);
    memcpy(&Value, &bits, 4);
    return true;
}

size_t HelloMsg::pack(uint8_t *o) const
{
    o[0] = Version;
    o[1] = Role;
    putU32(o + 2, ParamsHash);
    putU16(o + 6, ImuHz);
    return Size;
}
bool HelloMsg::unpack(const uint8_t *i, size_t len)
{
    if (len != Size)
        return false;
    Version = i[0];
    Role = i[1];
    ParamsHash = getU32(i + 2);
    ImuHz = getU16(i + 6);
    return true;
}

size_t PingMsg::pack(uint8_t *o) const
{
    putU32(o, T0);
    return Size;
}
bool PingMsg::unpack(const uint8_t *i, size_t len)
{
    if (len != Size)
        return false;
    T0 = getU32(i);
    return true;
}

size_t PongMsg::pack(uint8_t *o) const
{
    putU32(o, T0);
    putU32(o + 4, T1);
    putU32(o + 8, T2);
    return Size;
}
bool PongMsg::unpack(const uint8_t *i, size_t len)
{
    if (len != Size)
        return false;
    T0 = getU32(i);
    T1 = getU32(i + 4);
    T2 = getU32(i + 8);
    return true;
}

size_t CmdMsg::pack(uint8_t *o) const
{
    o[0] = Cmd;
    o[1] = Arg;
    putU16(o + 2, Val);
    return Size;
}
bool CmdMsg::unpack(const uint8_t *i, size_t len)
{
    if (len != Size)
        return false;
    Cmd = i[0];
    Arg = i[1];
    Val = getU16(i + 2);
    return true;
}

size_t ActMsg::pack(uint8_t *o) const
{
    o[0] = Kind;
    putU16(o + 1, Val1);
    putU16(o + 3, Val2);
    return Size;
}
bool ActMsg::unpack(const uint8_t *i, size_t len)
{
    if (len != Size)
        return false;
    Kind = i[0];
    Val1 = getU16(i + 1);
    Val2 = getU16(i + 3);
    return true;
}

/* -------------- Rahmen -------------- */

uint16_t LinkCodec::crc16(const uint8_t *data, size_t len, uint16_t crc)
{
    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

size_t LinkCodec::encode(uint8_t type, uint8_t seq, const uint8_t *payload, uint8_t len, uint8_t *out)
{
    if (len > LINK_MAX_PAYLOAD)
        return 0;
    out[0] = LINK_SOF1;
    out[1] = LINK_SOF2;
    out[2] = type;
    out[3] = seq;
    out[4] = len;
    if (len > 0)
        memcpy(out + 5, payload, len);
    const uint16_t crc = crc16(out + 2, 3u + len);
    putU16(out + 5 + len, crc);
    return (size_t)LINK_FRAME_OVERHEAD + len;
}

void LinkParser::reset()
{
    _n = 0;
    _haveSeq = false;
}

void LinkParser::_drop(uint8_t count)
{
    if (count >= _n)
    {
        _n = 0;
        return;
    }
    memmove(_buf, _buf + count, _n - count);
    _n = (uint8_t)(_n - count);
}

bool LinkParser::feed(uint8_t byte)
{
    if (_n < LINK_MAX_FRAME)
        _buf[_n++] = byte;

    // Am Anfang des Puffers einen Rahmen suchen; bei einem Fehler um ein Byte weiterschieben und neu versuchen.
    while (_n > 0)
    {
        if (_buf[0] != LINK_SOF1)
        {
            uint8_t skip = 1;
            while (skip < _n && _buf[skip] != LINK_SOF1)
                skip++;
            _skippedBytes += skip;
            _drop(skip);
            continue;
        }
        if (_n < 2)
            return false;
        if (_buf[1] != LINK_SOF2)
        {
            _skippedBytes++;
            _drop(1);
            continue;
        }
        if (_n < 5)
            return false;
        const uint8_t len = _buf[4];
        if (len > LINK_MAX_PAYLOAD)
        {
            _lengthErrors++;
            _skippedBytes++;
            _drop(1);
            continue;
        }
        const uint8_t total = (uint8_t)(LINK_FRAME_OVERHEAD + len);
        if (_n < total)
            return false;

        const uint16_t crcRx = (uint16_t)(_buf[total - 2] | (_buf[total - 1] << 8));
        if (crcRx != LinkCodec::crc16(_buf + 2, 3u + len))
        {
            _crcErrors++;
            _skippedBytes++;
            _drop(1);
            continue;
        }

        _type = _buf[2];
        _seq = _buf[3];
        _length = len;
        memcpy(_payload, _buf + 5, len);
        if (_haveSeq)
            _lostFrames += (uint8_t)(_seq - (uint8_t)(_lastSeq + 1));
        _lastSeq = _seq;
        _haveSeq = true;
        _framesOk++;
        _drop(total);
        return true;
    }
    return false;
}

/* -------------- Zeitabgleich -------------- */

void LinkClock::reset()
{
    _count = 0;
    _head = 0;
    _offset = 0;
    _delay = 0;
}

void LinkClock::addSample(uint32_t t0, uint32_t t1, uint32_t t2, uint32_t t3)
{
    // offset = ((t1 - t0) + (t2 - t3)) / 2 = (t1 - t0) - rtt / 2, mit rtt = (t3 - t0) - (t2 - t1).
    // Nur kleine Differenzen werden vorzeichenbehaftet gerechnet, die grosse (t1 - t0) bleibt modulo 2^32.
    const int32_t processing = (int32_t)(t2 - t1);
    const int32_t rtt = (int32_t)(t3 - t0) - processing;
    if (processing < 0 || rtt < 0)
        return; // unmoeglich: Zeiten stimmen nicht zusammen
    const int32_t offset = (int32_t)((uint32_t)(t1 - t0) - (uint32_t)(rtt / 2));

    _ring[_head] = {offset, rtt};
    _head = (uint8_t)((_head + 1) % Window);
    if (_count < Window)
        _count++;

    uint8_t best = 0;
    for (uint8_t i = 1; i < _count; i++)
        if (_ring[i].Delay < _ring[best].Delay)
            best = i;
    _offset = _ring[best].Offset;
    _delay = _ring[best].Delay;
}

/* -------------- Kurswinkel -------------- */

float linkWrap180(float deg)
{
    if (deg > -180.0f && deg <= 180.0f)
        return deg;
    float r = fmodf(deg + 180.0f, 360.0f);
    if (r < 0)
        r += 360.0f;
    return r - 180.0f;
}

void HeadingTracker::begin(float gyroSignDefault, uint32_t extrapMaxUs)
{
    _sign = gyroSignDefault;
    _extrapMaxUs = extrapMaxUs;
    _valid = false;
    _learned = false;
    _score = 0;
}

void HeadingTracker::addSample(uint32_t localUs, float headingDeg, float gyroDps)
{
    if (_valid)
    {
        const uint32_t dtUs = localUs - _lastUs;
        if (dtUs > 0 && dtUs < 50000)
        {
            const float dt = dtUs * 1e-6f;
            const float dH = linkWrap180(headingDeg - _heading);  // gemessene Aenderung
            const float expected = 0.5f * (gyroDps + _gyro) * dt; // Drehrate * Zeit
            // Nur bei deutlicher Drehung abstimmen: steigt der Winkel mit der Drehrate (+) oder gegenlaeufig (-)?
            if (fabsf(expected) > 0.3f && fabsf(dH) > 0.3f)
            {
                const bool same = (dH > 0) == (expected > 0);
                if (same && _score < 100)
                    _score++;
                else if (!same && _score > -100)
                    _score--;
                if (_score >= 10)
                {
                    _sign = 1.0f;
                    _learned = true;
                }
                else if (_score <= -10)
                {
                    _sign = -1.0f;
                    _learned = true;
                }
            }
        }
    }
    _heading = headingDeg;
    _gyro = gyroDps;
    _lastUs = localUs;
    _valid = true;
}

float HeadingTracker::headingAt(uint32_t nowUs) const
{
    uint32_t dtUs = nowUs - _lastUs;
    if (dtUs > 0x7FFFFFFFu) // Messung liegt in der Zukunft (Zeitabgleich ungenau): nicht rueckwaerts rechnen
        dtUs = 0;
    if (dtUs > _extrapMaxUs)
        dtUs = _extrapMaxUs;
    float h = _heading + _sign * _gyro * (dtUs * 1e-6f);
    h = fmodf(h, 360.0f);
    if (h < 0)
        h += 360.0f;
    return h;
}

/* -------------- Knoten -------------- */

void LinkNode::begin(LinkPort *port)
{
    _port = port;
    _seq = 0;
    _parser.reset();
    _framesSent = 0;
    _sendDropped = 0;
}

bool LinkNode::nextFrame(uint16_t maxBytes)
{
    if (!_port)
        return false;
    for (uint16_t i = 0; i < maxBytes; i++)
    {
        const int b = _port->readByte();
        if (b < 0)
            return false;
        if (_parser.feed((uint8_t)b))
            return true;
    }
    return false;
}

bool LinkNode::sendRaw(uint8_t type, const uint8_t *payload, uint8_t len)
{
    if (!_port)
        return false;
    uint8_t frame[LINK_MAX_FRAME];
    const size_t n = LinkCodec::encode(type, _seq, payload, len, frame);
    if (n == 0 || _port->writeSpace() < (int)n)
    {
        _sendDropped++;
        return false;
    }
    _seq++;
    _port->writeBytes(frame, n);
    _framesSent++;
    return true;
}

/* -------------- ESP-Seite -------------- */

void LinkEsp::begin(LinkPort *port, uint8_t version, uint32_t paramsHash, uint16_t imuHz)
{
    _node.begin(port);
    _version = version;
    _hash = paramsHash;
    _imuHz = imuHz;
    _sentHello = false;
    _peerHello = false;
    _mismatch = false;
    _haveCmd = false;
    _haveAct = false;
}

void LinkEsp::_sendHello()
{
    HelloMsg h;
    h.Version = _version;
    h.Role = LinkRoleEsp;
    h.ParamsHash = _hash;
    h.ImuHz = _imuHz;
    _node.send(LinkHello, h);
}

void LinkEsp::poll(uint32_t nowUs)
{
    while (_node.nextFrame())
    {
        const LinkParser &rx = _node.rx();
        switch (rx.type())
        {
        case LinkHello:
        {
            HelloMsg h;
            if (h.unpack(rx.payload(), rx.length()) && h.Role == LinkRoleTeensy)
            {
                _mismatch = (h.ParamsHash != _hash) || (h.Version != _version);
                _peerHello = true;
                _sendHello(); // der Teensy wartet auf unsere Antwort
            }
            break;
        }
        case LinkPing:
        {
            PingMsg p;
            if (p.unpack(rx.payload(), rx.length()))
            {
                PongMsg r;
                r.T0 = p.T0;
                r.T1 = nowUs; // Empfangszeit (so genau wie die Schleife sie hat)
                r.T2 = nowUs; // Sendezeit; in dieser Schleife liegen beide nahe beieinander
                _node.send(LinkPong, r);
            }
            break;
        }
        case LinkCmd:
            _haveCmd = _cmd.unpack(rx.payload(), rx.length()) || _haveCmd;
            break;
        case LinkAct:
            _haveAct = _act.unpack(rx.payload(), rx.length()) || _haveAct;
            break;
        default:
            break;
        }
    }

    // Bis der Teensy sich meldet: HELLO regelmaessig senden (nach einem ESP-Neustart ist der Teensy dann sofort im Bilde)
    if (!_peerHello && (!_sentHello || (uint32_t)(nowUs - _lastHelloUs) >= 500000u))
    {
        _sendHello();
        _sentHello = true;
        _lastHelloUs = nowUs;
    }
}

bool LinkEsp::takeCmd(CmdMsg &out)
{
    if (!_haveCmd)
        return false;
    out = _cmd;
    _haveCmd = false;
    return true;
}

bool LinkEsp::takeAct(ActMsg &out)
{
    if (!_haveAct)
        return false;
    out = _act;
    _haveAct = false;
    return true;
}
