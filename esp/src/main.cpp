// ESP32-S3-Firmware (Sensor-Koprozessor)
//
// Was der ESP am I2C-Bus liest, bestimmen die Debug-Flags aus lib/include/params/params_sources.h (muessen dem Teensy-
// Stand entsprechen):  HEADING_SOURCE = BNO055,  RADAR_SOURCE = vier XM125,  IR_SOURCE = IR-Ring.
// Ist keines davon auf ESP gestellt, fasst die Firmware den Bus nicht an (der Teensy ist dann dort Master).
//
//   BNO055      --> alle LINK_IMU_PERIOD_MS : Winkel + Drehrate mit Zeitstempel       --> Teensy (UART)
//   Radar       --> je Messrunde (~50 ms)   : vier Werte, Zeitstempel = Rundenmitte   --> Teensy
//   IR-Ring     --> alle LINK_IR_PERIOD_MS  : Ballwinkel + Distanz                     --> Teensy
//   SWITCH2     --> bei Aenderung (und alle LINK_INPUT_REFRESH_MS)                     --> Teensy
//   Teensy      --> HELLO / PING / CMD / ACT --> hier beantwortet bzw. ausgefuehrt (Servo, Piezo, Neustarts)
//
// Die Protokolllogik (Rahmen, Antworten auf HELLO/PING) steckt in lib/Link und ist auf dem Rechner getestet
// (test/native/run.sh). Diese Datei verbindet sie mit der Hardware.
#include <Arduino.h>
#include <Link.h>
#include <Wire.h>
#include <enum_esp.h>
#include <esp_timer.h>
#include <params.h>

#define ESP_HAS_BNO (HEADING_SOURCE == SENSOR_SOURCE_ESP)
#define ESP_HAS_RADAR (RADAR_SOURCE == SENSOR_SOURCE_ESP)
#define ESP_HAS_IR (IR_SOURCE == SENSOR_SOURCE_ESP)
#define ESP_USES_I2C (ESP_HAS_BNO || ESP_HAS_RADAR || ESP_HAS_IR)

#if ESP_HAS_BNO
#include <BNO055.h>
#endif
#if ESP_HAS_RADAR
#include <RadarArray.h>
#endif
#if ESP_HAS_IR
#include <ir_ring.h>
#endif

static StreamLinkPort linkPort(Serial1);
static LinkEsp link;

#if ESP_HAS_BNO
static BNO055 bno(BNO_ADDRESS, &Wire);
static bool bnoOk = false;
static uint8_t calibStat = 0;
static uint32_t nextImuUs = 0;
#endif
#if ESP_HAS_RADAR
static RadarArray radar;
static uint32_t lastRadarRound = 0;
static uint8_t radarCount = 0;
#endif
#if ESP_HAS_IR
static I2C_bus i2cIR(Wire);
static Device_handle irHandle = {i2cIR, I2C_target{0x0A}};
static IR_ring irRing(irHandle);
static uint32_t nextIrUs = 0;
#endif

static uint32_t nextHealthUs = 0;
static uint32_t nextInputUs = 0;
static uint32_t maxLoopUs = 0;
static uint32_t lastLoopStartUs = 0;
static uint16_t i2cErrors = 0;
static bool switch2Last = false;
static uint32_t switch2ChangeUs = 0;
static bool servoAttached = false;

static inline uint32_t nowUs() { return (uint32_t)esp_timer_get_time(); }
static inline bool due(uint32_t now, uint32_t at) { return (int32_t)(now - at) >= 0; }

/* -------------- Aktoren -------------- */

static constexpr uint32_t SERVO_FREQ_HZ = 50;
static constexpr uint8_t SERVO_RESOLUTION_BITS = 14;
static constexpr uint32_t SERVO_PERIOD_US = 20000;

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
#define LEDC_NEW_API 1
#else
#define LEDC_NEW_API 0
#endif

static void servoWrite(uint16_t us)
{
    if (us == 0) // 0 = keine Impulse mehr (Servo entspannt)
    {
        if (servoAttached)
        {
#if LEDC_NEW_API
            ledcDetach(ESP_PIN_SERVO);
#else
            ledcDetachPin(ESP_PIN_SERVO);
#endif
            servoAttached = false;
        }
        return;
    }
    if (!servoAttached)
    {
#if LEDC_NEW_API
        ledcAttach(ESP_PIN_SERVO, SERVO_FREQ_HZ, SERVO_RESOLUTION_BITS);
#else
        ledcSetup(0, SERVO_FREQ_HZ, SERVO_RESOLUTION_BITS);
        ledcAttachPin(ESP_PIN_SERVO, 0);
#endif
        servoAttached = true;
    }
    us = (uint16_t)constrain((int)us, 500, 2500);
    const uint32_t duty = (uint32_t)((uint64_t)us * ((1u << SERVO_RESOLUTION_BITS) - 1u) / SERVO_PERIOD_US);
#if LEDC_NEW_API
    ledcWrite(ESP_PIN_SERVO, duty);
#else
    ledcWrite(0, duty);
#endif
}

static void handleCommands()
{
    CmdMsg c;
    while (link.takeCmd(c))
    {
        switch (c.Cmd)
        {
#if ESP_HAS_BNO
        case LinkCmdBnoReset:
            bno.reset();
            delay(700); // BNO055 braucht nach einem Reset diese Zeit
            bno.begin();
            bnoOk = true;
            break;
#endif
#if ESP_HAS_RADAR
        case LinkCmdRadarCalib:
            radar.calib(); // blockiert rund eine Sekunde je Sensor
            break;
#endif
        default: // Kalibrierung speichern/laden: noch nicht umgesetzt; Befehle fuer nicht belegte Geraete ignorieren
            break;
        }
    }
    ActMsg a;
    while (link.takeAct(a))
    {
        if (a.Kind == 1)
            servoWrite(a.Val1);
        else if (a.Kind == 2 && a.Val1 > 0)
            tone(ESP_PIN_PIEZO, a.Val1, a.Val2);
    }
}

/* -------------- Sensoren -------------- */

#if ESP_HAS_BNO
static void sendImu(uint32_t t)
{
    int16_t gyro = 0, heading = 0;
    if (!bno.readGyroZHeadingRaw(gyro, heading))
    {
        i2cErrors++;
        return; // keine Messung ist besser als eine alte unter neuem Zeitstempel
    }
    ImuMsg m;
    m.TimeUs = t; // Zeitpunkt der Lesung (die Messung liegt hoechstens wenige ms davor)
    m.HeadingX16 = heading;
    m.GyroZX16 = gyro;
    m.Calib = calibStat;
    link.send(LinkImu, m);
}
#endif

#if ESP_HAS_RADAR
static void sendRadarIfNew()
{
    if (radar.giveRoundId() == lastRadarRound)
        return;
    lastRadarRound = radar.giveRoundId();
    RadarMsg m;
    m.TimeUs = radar.giveRoundMidUs(); // gleiche Zeitbasis wie esp_timer (micros() des Arduino-Cores)
    for (uint8_t k = 0; k < RADAR_NUM_SENSORS && k < 4; k++)
    {
        const RadarData &d = radar[k];
        m.Dist[k] = d.Dist;
        m.DistRaw[k] = d.DistRaw;
        m.Strength[k] = d.Strenght;
        if (d.isCon)
            m.ConBits |= (uint8_t)(1u << k);
        if (d.Peaks > 0)
            m.PeakBits |= (uint8_t)(1u << k);
    }
    link.send(LinkRadar, m);
}
#endif

#if ESP_HAS_IR
static void sendIr(uint32_t t)
{
    IrMsg m;
    m.TimeUs = t;
    m.AngleRaw = (int16_t)lroundf(irRing.read_ball_angle() * 180.0f); // zurueck auf den Register-Rohwert
    m.Dist = irRing.read_ball_distance();
    link.send(LinkIr, m);
}
#endif

static void sendInput(uint32_t t, bool force)
{
    const bool now = digitalRead(ESP_PIN_SWITCH2) != 0;
    if (now != switch2Last && due(t, switch2ChangeUs + 20000u)) // 20 ms Entprellung
    {
        switch2Last = now;
        switch2ChangeUs = t;
        force = true;
    }
    if (force)
    {
        InputMsg m;
        m.Bits = switch2Last ? 1 : 0;
        link.send(LinkInput, m);
        nextInputUs = t + LINK_INPUT_REFRESH_MS * 1000u;
    }
}

static void sendHealth(uint32_t t)
{
    HealthMsg h;
    h.UptimeMs = t / 1000u;
    h.MaxLoopUs = (uint16_t)min<uint32_t>(maxLoopUs, 65535u);
    h.I2cErrors = i2cErrors;
    h.RxErrors = (uint16_t)min<uint32_t>(link.rxErrors(), 65535u);
    h.FreeHeap = ESP.getFreeHeap();
    h.Flags = 0;
#if ESP_HAS_BNO
    calibStat = bno.calibStatus(); // eine zusaetzliche I2C-Lesung, nur im Takt der Statusmeldung
    if (bnoOk)
        h.Flags |= 0x01;
#endif
#if ESP_HAS_RADAR
    if (radarCount > 0)
        h.Flags |= 0x02;
#endif
    link.send(LinkHealth, h);
    maxLoopUs = 0;
}

void setup()
{
    Serial.begin(115200); // USB-Debug
    pinMode(ESP_PIN_SWITCH2, INPUT);

    Serial1.setRxBufferSize(1024);
    Serial1.begin(LINK_BAUD, SERIAL_8N1, ESP_PIN_LINK_RX, ESP_PIN_LINK_TX);
    link.begin(&linkPort, LINK_PROTOCOL_VERSION, LINK_PARAMS_HASH, 1000u / LINK_IMU_PERIOD_MS);
    link.poll(nowUs()); // HELLO sofort senden, noch bevor die Sensoren hochfahren (Radar-Start dauert Sekunden)

#if ESP_USES_I2C
    Wire.begin(ESP_PIN_SDA, ESP_PIN_SCL);
    Wire.setClock(I2C_WIRE_CLOCK_HZ); // 100 kHz: BNO055 und die Radare strecken den Takt
    Wire.setTimeOut(20);              // ms, ein haengender Sensor blockiert den Link nicht lange
#endif
#if ESP_HAS_BNO
    bno.begin();
    bnoOk = true;
#endif
#if ESP_HAS_RADAR
    radarCount = radar.begin(Wire); // blockiert pro Sensor bis ~2 s
#endif

    const uint32_t t = nowUs();
#if ESP_HAS_BNO
    nextImuUs = t + LINK_IMU_PERIOD_MS * 1000u;
#endif
#if ESP_HAS_IR
    nextIrUs = t + LINK_IR_PERIOD_MS * 1000u;
#endif
    nextHealthUs = t + LINK_HEALTH_PERIOD_MS * 1000u;
    nextInputUs = t;
    lastLoopStartUs = t;
    switch2Last = digitalRead(ESP_PIN_SWITCH2) != 0;
    switch2ChangeUs = t;
}

void loop()
{
    const uint32_t t = nowUs();
    const uint32_t loopUs = t - lastLoopStartUs;
    lastLoopStartUs = t;
    if (loopUs > maxLoopUs)
        maxLoopUs = loopUs;

    link.poll(t);
    handleCommands();

    // Der BNO055 hat Vorrang: sein Takt bestimmt die Qualitaet des Winkels
#if ESP_HAS_BNO
    if (due(t, nextImuUs))
    {
        nextImuUs += LINK_IMU_PERIOD_MS * 1000u;
        if (due(t, nextImuUs)) // zu spaet dran (z. B. nach einem langen I2C-Zugriff): nicht aufholen
            nextImuUs = t + LINK_IMU_PERIOD_MS * 1000u;
        sendImu(t);
    }
#endif
#if ESP_HAS_IR
    if (due(t, nextIrUs))
    {
        nextIrUs += LINK_IR_PERIOD_MS * 1000u;
        if (due(t, nextIrUs))
            nextIrUs = t + LINK_IR_PERIOD_MS * 1000u;
        sendIr(t);
    }
#endif
#if ESP_HAS_RADAR
    radar.update(); // holt fertige Ergebnisse ab (blockiert nie laenger als ein Registerzugriff je Sensor)
    sendRadarIfNew();
#endif

    sendInput(t, due(t, nextInputUs));

    if (due(t, nextHealthUs))
    {
        nextHealthUs += LINK_HEALTH_PERIOD_MS * 1000u;
        if (due(t, nextHealthUs))
            nextHealthUs = t + LINK_HEALTH_PERIOD_MS * 1000u;
        sendHealth(t);
    }
}
