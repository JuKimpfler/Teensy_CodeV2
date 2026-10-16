#include "SensorLink.h"

void SensorLink::begin(LinkPort *port, uint32_t nowUs)
{
    _node.begin(port);
    _clock.reset();
    _heading.begin(LINK_GYRO_SIGN_DEFAULT, LINK_EXTRAP_MAX_US);
    _beginUs = nowUs;
    _lastRxUs = nowUs;
    _anyRx = false;
    _espHello = false;
    _mismatch = false;
    _pingCount = 0;
    _lastPingUs = nowUs;
    _sendHello();
    _lastHelloUs = nowUs;
}

void SensorLink::_sendHello()
{
    HelloMsg h;
    h.Version = LINK_PROTOCOL_VERSION;
    h.Role = LinkRoleTeensy;
    h.ParamsHash = LINK_PARAMS_HASH;
    h.ImuHz = (uint16_t)(1000u / LINK_IMU_PERIOD_MS);
    _node.send(LinkHello, h);
}

uint32_t SensorLink::_toLocal(uint32_t espUs, uint32_t nowUs) const
{
    // Ohne Zeitabgleich gilt der Ankunftszeitpunkt; mit Abgleich die umgerechnete Messzeit
    return _clock.valid() ? _clock.toLocalUs(espUs) : nowUs;
}

void SensorLink::_onFrame(uint32_t nowUs)
{
    const LinkParser &rx = _node.rx();
    _lastRxUs = nowUs;
    _anyRx = true;

    switch (rx.type())
    {
    case LinkHello:
    {
        HelloMsg h;
        if (h.unpack(rx.payload(), rx.length()) && h.Role == LinkRoleEsp)
        {
            if (_espHello)
            {
                // Der ESP hat neu gestartet: Zeitabgleich und Winkel neu aufbauen
                _clock.reset();
                _heading.begin(LINK_GYRO_SIGN_DEFAULT, LINK_EXTRAP_MAX_US);
                _pingCount = 0;
            }
            _mismatch = (h.ParamsHash != LINK_PARAMS_HASH) || (h.Version != LINK_PROTOCOL_VERSION);
            _espHello = true;
            if (_pingCount == 0)
                _lastPingUs = nowUs - (uint32_t)LINK_PING_FAST_MS * 1000u; // sofort den ersten Ping
        }
        break;
    }
    case LinkPong:
    {
        PongMsg p;
        if (p.unpack(rx.payload(), rx.length()))
            _clock.addSample(p.T0, p.T1, p.T2, nowUs);
        break;
    }
    case LinkImu:
    {
        ImuMsg m;
        if (m.unpack(rx.payload(), rx.length()))
        {
            _heading.addSample(_toLocal(m.TimeUs, nowUs), m.HeadingX16 * (1.0f / 16.0f), m.GyroZX16 * (1.0f / 16.0f));
            _imuCalib = m.Calib;
        }
        break;
    }
    case LinkRadar:
        if (_radar.unpack(rx.payload(), rx.length()))
        {
            _radarLocalUs = _toLocal(_radar.TimeUs, nowUs);
            _radarNew = true;
        }
        break;
    case LinkIr:
        if (_ir.unpack(rx.payload(), rx.length()))
        {
            _irLocalUs = _toLocal(_ir.TimeUs, nowUs);
            _irNew = true;
        }
        break;
    case LinkInput:
    {
        InputMsg m;
        if (m.unpack(rx.payload(), rx.length()))
            _inputBits = m.Bits;
        break;
    }
    case LinkHealth:
        _health.unpack(rx.payload(), rx.length());
        break;
    default:
        break;
    }
}

void SensorLink::update(uint32_t nowUs)
{
    while (_node.nextFrame())
        _onFrame(nowUs);

    if (!_espHello)
    {
        // Warten auf den ESP: HELLO wiederholen
        if ((uint32_t)(nowUs - _lastHelloUs) >= (uint32_t)LINK_HELLO_INTERVAL_MS * 1000u)
        {
            _sendHello();
            _lastHelloUs = nowUs;
        }
        return;
    }

    // Zeitabgleich: zu Beginn schnell, danach 1 Hz
    const uint32_t interval = (uint32_t)((_pingCount < LINK_PING_FAST_COUNT) ? LINK_PING_FAST_MS : LINK_PING_INTERVAL_MS) * 1000u;
    if ((uint32_t)(nowUs - _lastPingUs) >= interval)
    {
        PingMsg p;
        p.T0 = nowUs;
        if (_node.send(LinkPing, p))
        {
            _lastPingUs = nowUs;
            if (_pingCount < 255)
                _pingCount++;
        }
    }
}

bool SensorLink::isAlive(uint32_t nowUs) const
{
    return _anyRx && (uint32_t)(nowUs - _lastRxUs) <= (uint32_t)LINK_ALIVE_MS * 1000u;
}

bool SensorLink::isUsable(uint32_t nowUs) const
{
    return isAlive(nowUs) && _espHello && !_mismatch;
}

bool SensorLink::takeRadar(RadarMsg &out, uint32_t &localUs)
{
    if (!_radarNew)
        return false;
    _radarNew = false;
    out = _radar;
    localUs = _radarLocalUs;
    return true;
}

bool SensorLink::takeIr(IrMsg &out, uint32_t &localUs)
{
    if (!_irNew)
        return false;
    _irNew = false;
    out = _ir;
    localUs = _irLocalUs;
    return true;
}

bool SensorLink::giveBootTimeout(uint32_t nowUs) const
{
    return !_espHello && (uint32_t)(nowUs - _beginUs) >= (uint32_t)LINK_BOOT_TIMEOUT_MS * 1000u;
}

bool SensorLink::giveHeadingValid(uint32_t nowUs) const
{
    return isUsable(nowUs) && _heading.valid() && _heading.ageUs(nowUs) <= (uint32_t)LINK_ALIVE_MS * 1000u;
}

float SensorLink::giveHeading(uint32_t nowUs) const
{
    return _heading.headingAt(nowUs);
}

bool SensorLink::sendCmd(uint8_t cmd, uint8_t arg, uint16_t val)
{
    CmdMsg m;
    m.Cmd = cmd;
    m.Arg = arg;
    m.Val = val;
    return _node.send(LinkCmd, m);
}

bool SensorLink::sendServo(uint16_t microseconds)
{
    ActMsg m;
    m.Kind = 1;
    m.Val1 = microseconds;
    return _node.send(LinkAct, m);
}

bool SensorLink::sendPiezo(uint16_t hz, uint16_t ms)
{
    ActMsg m;
    m.Kind = 2;
    m.Val1 = hz;
    m.Val2 = ms;
    return _node.send(LinkAct, m);
}
