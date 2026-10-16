// Rechnertest fuer lib/Link und lib/SensorLink (kein Teensy, kein ESP). Aufruf: test/native/run.sh
#include <Link.h>
#include <SensorLink.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <random>
#include <vector>

static int g_checks = 0, g_fail = 0;
#define CHECK(cond)                                                    \
    do                                                                 \
    {                                                                  \
        g_checks++;                                                    \
        if (!(cond))                                                   \
        {                                                              \
            g_fail++;                                                  \
            printf("  FEHLER %s:%d  %s\n", __FILE__, __LINE__, #cond); \
        }                                                              \
    } while (0)

/* ---------- Hilfsmittel ---------- */

/// Eine Richtung der UART-Strecke mit Laufzeit: 2 Mbaud = 5 us je Byte. Optional Bitfehler und Verlust.
struct Pipe
{
    struct B
    {
        uint8_t v;
        uint32_t at;
    };
    std::deque<B> q;
    uint32_t lastArrival = 0;
    uint32_t now = 0;
    double flipRate = 0, dropRate = 0;
    std::mt19937 rng{1};
    int readByte()
    {
        if (q.empty() || q.front().at > now)
            return -1;
        int v = q.front().v;
        q.pop_front();
        return v;
    }
    void push(const uint8_t *d, size_t n)
    {
        std::uniform_real_distribution<double> u(0, 1);
        for (size_t i = 0; i < n; i++)
        {
            if (u(rng) < dropRate)
                continue;
            uint8_t v = d[i];
            if (u(rng) < flipRate)
                v ^= (uint8_t)(1u << (rng() % 8));
            lastArrival = std::max(lastArrival, now) + 5;
            q.push_back({v, lastArrival});
        }
    }
};

static std::vector<uint8_t> frame(uint8_t type, uint8_t seq, std::vector<uint8_t> pl)
{
    uint8_t out[LINK_MAX_FRAME];
    size_t n = LinkCodec::encode(type, seq, pl.data(), (uint8_t)pl.size(), out);
    return std::vector<uint8_t>(out, out + n);
}

static int feedAll(LinkParser &p, const std::vector<uint8_t> &d, std::vector<std::vector<uint8_t>> *got = nullptr, std::vector<uint8_t> *types = nullptr)
{
    int frames = 0;
    for (uint8_t b : d)
        if (p.feed(b))
        {
            frames++;
            if (got)
                got->push_back(std::vector<uint8_t>(p.payload(), p.payload() + p.length()));
            if (types)
                types->push_back(p.type());
        }
    return frames;
}

/* ---------- Tests ---------- */

static void testCrc()
{
    printf("CRC16\n");
    const uint8_t d[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(LinkCodec::crc16(d, 9) == 0x29B1); // bekannter Pruefwert fuer CRC-16/CCITT-FALSE
    CHECK(LinkCodec::crc16(d, 0) == 0xFFFF);
}

static void testMessages()
{
    printf("Nachrichten: pack/unpack und Rahmen\n");
    uint8_t buf[LINK_MAX_PAYLOAD];
    LinkParser p;

    ImuMsg a;
    a.TimeUs = 0xDEADBEEF;
    a.HeadingX16 = 5759;
    a.GyroZX16 = -1920;
    a.Calib = 0xE4;
    ImuMsg a2;
    CHECK(a.pack(buf) == ImuMsg::Size && a2.unpack(buf, ImuMsg::Size));
    CHECK(a2.TimeUs == a.TimeUs && a2.HeadingX16 == a.HeadingX16 && a2.GyroZX16 == a.GyroZX16 && a2.Calib == a.Calib);
    CHECK(!a2.unpack(buf, ImuMsg::Size - 1) && !a2.unpack(buf, ImuMsg::Size + 1));

    RadarMsg r;
    r.TimeUs = 123456789;
    r.ConBits = 0b1011;
    r.PeakBits = 0b0110;
    for (int i = 0; i < 4; i++)
    {
        r.Dist[i] = (uint16_t)(1000 + 7 * i);
        r.DistRaw[i] = (uint16_t)(1003 + 5 * i);
        r.Strength[i] = -11000 - 2500 * i;
    }
    RadarMsg r2;
    CHECK(r.pack(buf) == RadarMsg::Size && r2.unpack(buf, RadarMsg::Size) && r2.ConBits == r.ConBits && r2.PeakBits == r.PeakBits);
    for (int i = 0; i < 4; i++)
        CHECK(r2.Dist[i] == r.Dist[i] && r2.DistRaw[i] == r.DistRaw[i] && r2.Strength[i] == r.Strength[i]);

    HelloMsg h;
    h.Version = 1;
    h.Role = LinkRoleEsp;
    h.ParamsHash = 0xA1B2C3D4;
    h.ImuHz = 100;
    HelloMsg h2;
    CHECK(h.pack(buf) == HelloMsg::Size && h2.unpack(buf, HelloMsg::Size) && h2.ParamsHash == h.ParamsHash && h2.ImuHz == 100 && h2.Role == LinkRoleEsp);

    PongMsg po;
    po.T0 = 1;
    po.T1 = 0xFFFFFFFF;
    po.T2 = 0x80000000;
    PongMsg po2;
    CHECK(po.pack(buf) == PongMsg::Size && po2.unpack(buf, PongMsg::Size) && po2.T1 == 0xFFFFFFFF && po2.T2 == 0x80000000);

    DebugMsg dbg;
    dbg.Channel = 7;
    dbg.Value = -123.5f;
    DebugMsg dbg2;
    CHECK(dbg.pack(buf) == DebugMsg::Size && dbg2.unpack(buf, DebugMsg::Size) && dbg2.Value == -123.5f && dbg2.Channel == 7);

    CmdMsg c;
    c.Cmd = LinkCmdBnoCalibSave;
    c.Arg = 3;
    c.Val = 0xABCD;
    CmdMsg c2;
    CHECK(c.pack(buf) == CmdMsg::Size && c2.unpack(buf, CmdMsg::Size) && c2.Val == 0xABCD);

    HealthMsg hl;
    hl.UptimeMs = 4000000000u;
    hl.MaxLoopUs = 65535;
    hl.I2cErrors = 3;
    hl.RxErrors = 4;
    hl.FreeHeap = 200000;
    hl.Flags = 1;
    HealthMsg hl2;
    CHECK(hl.pack(buf) == HealthMsg::Size && hl2.unpack(buf, HealthMsg::Size) && hl2.UptimeMs == hl.UptimeMs && hl2.FreeHeap == 200000);

    // Rahmen: gueltig, Laengen
    ImuMsg m = a;
    size_t n = m.pack(buf);
    uint8_t fr[LINK_MAX_FRAME];
    size_t fn = LinkCodec::encode(LinkImu, 5, buf, (uint8_t)n, fr);
    CHECK(fn == n + LINK_FRAME_OVERHEAD);
    int frames = 0;
    for (size_t i = 0; i < fn; i++)
        frames += p.feed(fr[i]);
    CHECK(frames == 1 && p.type() == LinkImu && p.seq() == 5 && p.length() == n);
    uint8_t big[LINK_MAX_PAYLOAD + 1] = {};
    CHECK(LinkCodec::encode(LinkImu, 0, big, LINK_MAX_PAYLOAD + 1, fr) == 0);
    CHECK(LinkCodec::encode(LinkImu, 0, big, LINK_MAX_PAYLOAD, fr) == LINK_MAX_FRAME);
}

static void testParserResync()
{
    printf("Parser: Muell, Bitfehler, abgeschnittene Rahmen\n");
    std::mt19937 rng(5);
    auto A = frame(LinkImu, 0, {1, 2, 3, 4, 5, 6, 7, 8, 9});
    auto B = frame(LinkHealth, 1, std::vector<uint8_t>(15, 0x33));
    auto C = frame(LinkCmd, 2, {9, 8, 7, 6});

    { // Muell vor, zwischen und nach den Rahmen (auch mit Beginn-Bytes im Muell)
        LinkParser p;
        std::vector<uint8_t> s = {0x00, LINK_SOF1, 0x12, LINK_SOF1, LINK_SOF2, 0xFF, 0xFF};
        s.insert(s.end(), A.begin(), A.end());
        s.insert(s.end(), {LINK_SOF1, LINK_SOF1, LINK_SOF2, 99});
        s.insert(s.end(), B.begin(), B.end());
        s.insert(s.end(), C.begin(), C.end());
        std::vector<uint8_t> types;
        CHECK(feedAll(p, s, nullptr, &types) == 3);
        CHECK(types == std::vector<uint8_t>({LinkImu, LinkHealth, LinkCmd}));
    }
    { // Ein Bit gekippt: dieser Rahmen faellt aus, der naechste kommt durch
        LinkParser p;
        auto A2 = A;
        A2[6] ^= 0x10;
        std::vector<uint8_t> s = A2;
        s.insert(s.end(), B.begin(), B.end());
        std::vector<uint8_t> types;
        CHECK(feedAll(p, s, nullptr, &types) == 1 && types[0] == LinkHealth);
        CHECK(p.crcErrors() >= 1);
    }
    { // Rahmen abgeschnitten (Laenge verspricht mehr), dann beginnt ein gueltiger Rahmen mitten darin
        LinkParser p;
        std::vector<uint8_t> s(A.begin(), A.begin() + 5);
        s.back() = 40; // Laenge 40 angekuendigt
        s.insert(s.end(), B.begin(), B.end());
        s.insert(s.end(), C.begin(), C.end());
        s.insert(s.end(), 40, (uint8_t)0x00);
        std::vector<uint8_t> types;
        feedAll(p, s, nullptr, &types);
        CHECK(types.size() == 2 && types[0] == LinkHealth && types[1] == LinkCmd);
    }
    { // Laenge groesser als erlaubt
        LinkParser p;
        std::vector<uint8_t> s = {LINK_SOF1, LINK_SOF2, 1, 0, 200};
        s.insert(s.end(), A.begin(), A.end());
        CHECK(feedAll(p, s) == 1 && p.lengthErrors() >= 1);
    }
    { // Sequenzluecken zaehlen
        LinkParser p;
        std::vector<uint8_t> s;
        for (int seq : {10, 11, 14, 15, 20})
        {
            auto f = frame(LinkInput, (uint8_t)seq, {1});
            s.insert(s.end(), f.begin(), f.end());
        }
        CHECK(feedAll(p, s) == 5 && p.lostFrames() == 2 + 4);
        // Zaehler laeuft ueber 255
        LinkParser q;
        std::vector<uint8_t> t;
        for (int seq : {254, 255, 0, 1})
        {
            auto f = frame(LinkInput, (uint8_t)seq, {1});
            t.insert(t.end(), f.begin(), f.end());
        }
        CHECK(feedAll(q, t) == 4 && q.lostFrames() == 0);
    }
}

static void testFuzz()
{
    printf("Parser: Zufallsdaten und Bitfehler (nie ein verfaelschter Rahmen)\n");
    std::mt19937 rng(99);
    // 1) reiner Zufall: jeder gelieferte Rahmen muss eine gueltige CRC haben
    {
        LinkParser p;
        long delivered = 0;
        for (long i = 0; i < 3000000; i++)
            if (p.feed((uint8_t)rng()))
            {
                delivered++;
                uint8_t chk[LINK_MAX_FRAME];
                size_t n = LinkCodec::encode(p.type(), p.seq(), p.payload(), p.length(), chk);
                CHECK(n == (size_t)p.length() + LINK_FRAME_OVERHEAD);
            }
        printf("  3 000 000 Zufallsbytes, davon als Rahmen angenommen: %ld\n", delivered);
        CHECK(delivered < 10); // 2^-16 je Versuch an einer Beginn-Stelle: praktisch nie
    }
    // 2) gueltiger Strom mit Bitfehlern: alles, was ankommt, muss mit einem gesendeten Rahmen uebereinstimmen
    {
        LinkParser p;
        long sent = 0, ok = 0, wrong = 0;
        std::vector<std::vector<uint8_t>> payloads;
        for (int i = 0; i < 20000; i++)
        {
            std::vector<uint8_t> pl((size_t)(rng() % 30));
            for (auto &x : pl)
                x = (uint8_t)rng();
            payloads.push_back(pl);
        }
        std::vector<uint8_t> stream;
        for (size_t i = 0; i < payloads.size(); i++)
        {
            auto f = frame(LinkInput, (uint8_t)i, payloads[i]);
            stream.insert(stream.end(), f.begin(), f.end());
            sent++;
        }
        for (auto &b : stream)
            if (rng() % 200 == 0)
                b ^= (uint8_t)(1u << (rng() % 8));
        for (uint8_t b : stream)
            if (p.feed(b))
            {
                ok++;
                std::vector<uint8_t> got(p.payload(), p.payload() + p.length());
                // gehoert zu irgendeinem gesendeten Rahmen mit dieser Sequenznummer?
                bool found = false;
                for (size_t k = p.seq(); k < payloads.size(); k += 256)
                    if (payloads[k] == got)
                    {
                        found = true;
                        break;
                    }
                if (!found)
                    wrong++;
            }
        printf("  %ld Rahmen gesendet, %ld angekommen, %ld verfaelscht\n", sent, ok, wrong);
        CHECK(wrong == 0 && ok > sent / 2);
    }
}

static void testClock()
{
    printf("Zeitabgleich\n");
    std::mt19937 rng(7);
    for (int trial = 0; trial < 200; trial++)
    {
        LinkClock c;
        uint32_t offset = (uint32_t)rng(); // ESP-Zeit = Teensy-Zeit + offset, beliebig inkl. Ueberlauf
        uint32_t t = (uint32_t)rng();
        double worst = 0;
        for (int i = 0; i < 20; i++)
        {
            uint32_t up = 50 + rng() % 300, down = 50 + rng() % 300, proc = 5 + rng() % 40;
            uint32_t t0 = t;
            uint32_t t1 = t0 + up + offset;
            uint32_t t2 = t1 + proc;
            uint32_t t3 = t2 - offset + down;
            c.addSample(t0, t1, t2, t3);
            t += 100000 + rng() % 1000;
        }
        int32_t err = (int32_t)((uint32_t)c.offsetUs() - offset);
        worst = std::abs((double)err);
        CHECK(c.valid() && worst < 200); // Fehler hoechstens die halbe Hin-/Rueck-Asymmetrie
        CHECK(c.toLocalUs(1234u + offset) - 1234u < 200u || 1234u - c.toLocalUs(1234u + offset) < 200u);
    }
    LinkClock c;
    c.addSample(100, 50, 40, 200); // t2 < t1: unmoeglich -> verworfen
    CHECK(!c.valid());
}

static void testHeading()
{
    printf("Winkel: Weiterrechnen und Vorzeichen lernen\n");
    for (float conv : {-1.0f, +1.0f}) // wahre Beziehung: Winkel' = conv * Drehrate
    {
        HeadingTracker h;
        h.begin(-1.0f, 25000);
        double errExtra = 0, errPlain = 0;
        long n = 0;
        float angle = 350;
        float rate = 0; // Grad/s des Winkels
        uint32_t tUs = 1000000;
        float lastMeasured = 0;
        for (int step = 0; step < 600; step++)
        {
            rate = 150.0f * sinf(step * 0.02f); // schwingende Drehung
            float gyro = rate / conv;           // Drehrate Z so, dass Winkel' = conv * Drehrate
            angle += rate * 0.01f;
            float a = fmodf(angle + 3600, 360);
            // BNO055: 1/16 Grad
            float aq = roundf(a * 16) / 16, gq = roundf(gyro * 16) / 16;
            tUs += 10000;
            h.addSample(tUs, aq, gq);
            lastMeasured = aq;
            if (step > 100)
                for (uint32_t dt : {2000u, 5000u, 8000u})
                {
                    float truth = fmodf(a + rate * dt * 1e-6f + 3600, 360);
                    double e1 = linkWrap180(h.headingAt(tUs + dt) - truth);
                    double e2 = linkWrap180(lastMeasured - truth);
                    errExtra += e1 * e1;
                    errPlain += e2 * e2;
                    n++;
                }
        }
        double rmsE = sqrt(errExtra / n), rmsP = sqrt(errPlain / n);
        printf("  Vorzeichen %+.0f: gelernt=%d (%+.0f), Fehler mit Drehrate %.3f Grad gegen %.3f Grad ohne\n", conv, h.signLearned(), h.gyroSign(), rmsE, rmsP);
        CHECK(h.signLearned() && h.gyroSign() == conv);
        CHECK(rmsE < 0.25 * rmsP && rmsE < 0.25);
    }
    HeadingTracker h; // Grenzen
    h.begin(-1.0f, 25000);
    h.addSample(1000, 10.0f, 100.0f);
    float far = h.headingAt(1000 + 1000000); // eine Sekunde spaeter: nur bis 25 ms
    CHECK(fabsf(far - (10.0f - 100.0f * 0.025f)) < 0.01f);
    CHECK(fabsf(h.headingAt(500) - 10.0f) < 0.001f); // Zeit liegt vor der Messung: nicht rueckwaerts
    h.addSample(2000, 359.0f, 0.0f);
    CHECK(fabsf(linkWrap180(h.headingAt(2000) - 359.0f)) < 0.001f);
    CHECK(h.headingAt(2000) >= 0 && h.headingAt(2000) < 360);
    CHECK(fabsf(linkWrap180(190.0f) + 170.0f) < 1e-4f && fabsf(linkWrap180(-190.0f) - 170.0f) < 1e-4f);
}

/// ESP + Teensy ueber zwei Strecken mit Laufzeit, 5 Sekunden.
struct Sim
{
    Pipe toTeensy, toEsp;
    LinkEsp esp;
    SensorLink teensy;
    uint32_t espOffset;
    uint32_t espHash = LINK_PARAMS_HASH;
    float conv = -1.0f;
    double trueAngle = 100, trueRate = 0;
    uint32_t tUs = 5000000;
    uint32_t nextImu = 0;
    bool espRunning = true;
    double sumErr2 = 0, maxAge = 0;
    long samples = 0, valid = 0;
    uint32_t maxHeadingAgeUs = 0;
    uint32_t nextRadar = 0, nextIr = 0;
    bool switch2 = false;
    uint32_t radarExpectedLocal = 0, irExpectedLocal = 0;
    long radarFrames = 0, irFrames = 0, badFrames = 0;
    double radarErrMaxUs = 0, irErrMaxUs = 0;

    Sim(uint32_t offset, uint32_t hash) : espOffset(offset), espHash(hash)
    {
        esp.begin(&espPort, LINK_PROTOCOL_VERSION, espHash, 100);
        teensy.begin(&teensyPort, tUs);
        nextImu = tUs + 10000;
        nextRadar = tUs + 50000;
        nextIr = tUs + 10000;
    }
    // Zwei Sichten auf die Strecken: lesen aus der eigenen Richtung, schreiben in die des Partners
    struct Side : LinkPort
    {
        Pipe *rx;
        Pipe *tx;
        Side(Pipe *r, Pipe *t) : rx(r), tx(t) {}
        int readByte() override { return rx->readByte(); }
        size_t writeBytes(const uint8_t *d, size_t n) override
        {
            tx->push(d, n);
            return n;
        }
    };
    Side teensyPort{&toTeensy, &toEsp};
    Side espPort{&toEsp, &toTeensy};

    uint32_t espTime() const { return tUs + espOffset; }

    void run(uint32_t durationUs)
    {
        const uint32_t end = tUs + durationUs;
        uint32_t nextTeensy = tUs, nextEsp = tUs;
        for (; tUs < end; tUs += 20)
        {
            toTeensy.now = toEsp.now = tUs;
            if (espRunning && tUs >= nextEsp)
            {
                nextEsp += 200;
                esp.poll(espTime());
                if (tUs >= nextImu)
                {
                    nextImu += 10000;
                    trueRate = 120.0 * sin((tUs - 5000000) * 1e-6 * 2.0);
                    // Winkel zum Messzeitpunkt (jetzt)
                    ImuMsg m;
                    m.TimeUs = espTime();
                    double a = fmod(trueAngle + 3600, 360);
                    m.HeadingX16 = (int16_t)lround(a * 16);
                    m.GyroZX16 = (int16_t)lround(trueRate / conv * 16);
                    m.Calib = 0xFF;
                    esp.send(LinkImu, m);
                }
                if (tUs >= nextRadar)
                {
                    nextRadar += 50000;
                    RadarMsg r;
                    r.TimeUs = espTime() - 25000; // Mitte der Messrunde
                    r.ConBits = 0x0F;
                    r.PeakBits = 0x0D;
                    for (int k = 0; k < 4; k++)
                    {
                        r.Dist[k] = (uint16_t)(1000 + k);
                        r.DistRaw[k] = (uint16_t)(1010 + k);
                        r.Strength[k] = -11000 - k;
                    }
                    radarExpectedLocal = tUs - 25000;
                    esp.send(LinkRadar, r);
                }
                if (tUs >= nextIr)
                {
                    nextIr += 10000;
                    IrMsg m;
                    m.TimeUs = espTime();
                    m.AngleRaw = -123;
                    m.Dist = 45;
                    irExpectedLocal = tUs;
                    esp.send(LinkIr, m);
                }
                if (!switch2 && tUs >= 7000000)
                {
                    switch2 = true;
                    InputMsg im;
                    im.Bits = 1;
                    esp.send(LinkInput, im);
                }
            }
            trueAngle += trueRate * 20e-6;
            if (tUs >= nextTeensy)
            {
                nextTeensy += 500;
                teensy.update(tUs);
                RadarMsg rm;
                IrMsg ir;
                uint32_t lu = 0;
                while (teensy.takeRadar(rm, lu))
                {
                    radarFrames++;
                    radarErrMaxUs = std::max(radarErrMaxUs, std::abs((double)(int32_t)(lu - radarExpectedLocal)));
                    if (rm.DistRaw[2] != 1012 || rm.Strength[3] != -11003 || rm.ConBits != 0x0F || rm.PeakBits != 0x0D || rm.Dist[1] != 1001)
                        badFrames++;
                }
                while (teensy.takeIr(ir, lu))
                {
                    irFrames++;
                    irErrMaxUs = std::max(irErrMaxUs, std::abs((double)(int32_t)(lu - irExpectedLocal)));
                    if (ir.AngleRaw != -123 || ir.Dist != 45)
                        badFrames++;
                }
                if (teensy.giveHeadingValid(tUs))
                {
                    valid++;
                    double truth = fmod(trueAngle + 3600, 360);
                    double e = linkWrap180((float)(teensy.giveHeading(tUs) - truth));
                    sumErr2 += e * e;
                    maxHeadingAgeUs = std::max(maxHeadingAgeUs, teensy.giveHeadingAgeUs(tUs));
                }
                samples++;
            }
        }
    }
};

static void testEndToEnd()
{
    printf("ESP <-> Teensy: Verbindung, Zeit, Winkel\n");
    for (float conv : {-1.0f, +1.0f})
    {
        Sim s(0xF0000000u, LINK_PARAMS_HASH); // ESP-Zeit liegt weit vor der Teensy-Zeit, mit Ueberlauf
        s.conv = conv;
        s.run(1000000);
        CHECK(s.teensy.giveHandshakeDone() && !s.teensy.giveParamsMismatch());
        CHECK(s.esp.teensyKnown());
        CHECK(s.teensy.giveClockValid());
        int32_t offErr = (int32_t)((uint32_t)s.teensy.giveClockOffsetUs() - s.espOffset);
        printf("  conv %+.0f: Zeitversatz-Fehler %d us, Laufzeit %d us\n", conv, offErr, s.teensy.giveClockDelayUs());
        CHECK(std::abs(offErr) < 400);
        s.run(4000000);
        double rms = sqrt(s.sumErr2 / std::max(1L, s.valid));
        printf("  Winkelfehler RMS %.3f Grad, aeltestes Winkelalter %.2f ms, gueltig %ld/%ld\n", rms, s.maxHeadingAgeUs / 1000.0, s.valid, s.samples);
        CHECK(rms < 0.5);
        CHECK(s.maxHeadingAgeUs < 12000);
        CHECK(s.teensy.giveGyroSign() == conv && s.teensy.giveCrcErrors() == 0);
        printf("  Radar: %ld Runden, Zeitfehler max %.0f us | IR: %ld Messungen, Zeitfehler max %.0f us | SWITCH2 %d\n", s.radarFrames, s.radarErrMaxUs, s.irFrames, s.irErrMaxUs, s.teensy.giveSwitch2());
        CHECK(s.radarFrames >= 90 && s.irFrames >= 450 && s.badFrames == 0);
        CHECK(s.radarErrMaxUs < 400 && s.irErrMaxUs < 400);
        CHECK(s.teensy.giveSwitch2());
    }

    { // falsche Parameter-Pruefsumme
        Sim s(1234, LINK_PARAMS_HASH ^ 1u);
        s.run(800000);
        CHECK(s.teensy.giveHandshakeDone() && s.teensy.giveParamsMismatch());
        CHECK(s.esp.paramsMismatch());
        CHECK(!s.teensy.isUsable(s.tUs) && !s.teensy.giveHeadingValid(s.tUs)); // falsche Parameter: Daten nicht benutzen
    }
    { // Link faellt aus: nach LINK_ALIVE_MS nicht mehr gueltig
        Sim s(777, LINK_PARAMS_HASH);
        s.run(1500000);
        CHECK(s.teensy.giveHeadingValid(s.tUs));
        s.espRunning = false;
        s.run(LINK_ALIVE_MS * 1000u + 5000);
        CHECK(!s.teensy.isAlive(s.tUs) && !s.teensy.giveHeadingValid(s.tUs));
    }
    { // ESP startet neu: Teensy baut Zeitabgleich und Winkel neu auf
        Sim s(100000, LINK_PARAMS_HASH);
        s.run(1500000);
        s.espOffset = 0x12345678u; // andere ESP-Uhr nach dem Neustart
        s.esp.begin(&s.espPort, LINK_PROTOCOL_VERSION, LINK_PARAMS_HASH, 100);
        s.run(1500000);
        int32_t offErr = (int32_t)((uint32_t)s.teensy.giveClockOffsetUs() - s.espOffset);
        CHECK(s.teensy.giveHeadingValid(s.tUs) && std::abs(offErr) < 400);
    }
    { // 1 % der Bytes mit Bitfehlern, 0,5 % verloren: nie ein falscher Winkel, Verbindung bleibt
        Sim s(4242, LINK_PARAMS_HASH);
        s.toTeensy.flipRate = 0.01;
        s.toTeensy.dropRate = 0.005;
        s.toEsp.flipRate = 0.01;
        s.run(1000000);
        s.sumErr2 = 0;
        s.valid = 0;
        s.run(4000000);
        double rms = sqrt(s.sumErr2 / std::max(1L, s.valid));
        printf("  Stoerung: Winkelfehler RMS %.3f Grad, CRC-Fehler %u, Luecken %u\n", rms, s.teensy.giveCrcErrors(), s.teensy.giveLostFrames());
        CHECK(s.teensy.giveCrcErrors() > 0 && rms < 1.0);
    }
}

int main()
{
    testCrc();
    testMessages();
    testParserResync();
    testFuzz();
    testClock();
    testHeading();
    testEndToEnd();
    printf("\n%d Pruefungen, %d Fehler\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
