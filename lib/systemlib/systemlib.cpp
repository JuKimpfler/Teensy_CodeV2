#include <systemlib.h>

/* -------------- Instanzen -------------- */

BNO055 BNO(BNO_ADDRESS, &I2C_BNO);
// PowerDebugger PDS;

elapsedMillis cycleTimer;
elapsedMicros cycleTimer_us;
elapsedMillis lastvaliddata_Radar;

commlib communication;

I2C_bus i2cIR(I2C_IR);
Device_handle ir_ring_handle = {i2cIR, I2C_target{0x0A}};
IR_ring ir_ring(ir_ring_handle);

INA219 INA219_Main(0x40);
INA219 INA219_Motor(0x41);

TCA9534DW SW_EXP(&I2C_SW, 0x22);
TCA9534DW Motor_EXP(&Wire1, 0x20);
TCA9534DW Button_EXP(&Wire1, 0x23);
TCA9534DW Rand_EXP(&Wire1, 0x21);

ADC108S102 ADC_Motor(&SPI1, ADC108S102_Motor_CS);
ADC108S102 ADC_Line1(&SPI, ADC108S102_Motor_CS);
ADC108S102 ADC_Line2(&SPI, ADC108S102_Motor_CS);
ADC108S102 ADC_Line3(&SPI1, ADC108S102_Motor_CS);
ADC108S102 ADC_Line4(&SPI1, ADC108S102_Motor_CS);
ADC108S102 ADC_Line5(&SPI1, ADC108S102_Motor_CS);

constexpr float TS_NOMINAL = 0.005f; // nur Fallback fuer cfg.Ts, siehe loop()
PID *headingControllerP;
ADRC *headingControllerA;
uint32_t lastMicrosContrl = 0;

fancyBTN BTN1(50, 6, 8, true);
fancyBTN BTN2(50, 6, 8, true);
fancyBTN BTN3(50, 6, 8, true);
fancyBTN BTN4(50, 6, 8, true);
fancyBTN BTN5(50, 6, 8, true);

/* -------------- System Worker -------------- */

void systemlib::initialize()
{
    Wire1.begin();
    Wire1.setClock(100000);
    Wire.begin();
    Wire.setClock(100000);

    Rand_EXP.setup(false);
    Motor_EXP.setup(false);
    SW_EXP.setup(true);
    Button_EXP.setup(true);
    readExpanders();
    Motor_EXP.writePin(7,false);

    SPI.begin();
    SPI1.begin();

    USBC_UART.begin(115200);

    INA219_Main.begin(Wire1);
    INA219_Motor.begin(Wire1);

    INA219_Main.setMaxCurrentShunt(15);
    INA219_Motor.setMaxCurrentShunt(15);

    BNO.init();
    calibBNO();

    PDS.init();

    MoDataVL.ID = MotorVL_ID;
    MoDataVR.ID = MotorVR_ID;
    MoDataHL.ID = MotorHL_ID;
    MoDataHR.ID = MotorHR_ID;
    MoDataDR.ID = MotorDR_ID;

    pinMode(START_SWITCH, INPUT);

    Motor_EXP.setup(false);
    Rand_EXP.setup(false);
    SW_EXP.setup(true);
    Button_EXP.setup(true);

    Radar.begin(I2C_RADAR); // richtet alle Sensoren ein und startet die erste Messrunde

    ir_ring.read_gains(IR_Gains);
    ir_ring.read_offsets(IR_Offsets);

    setDebugTimeout(0);

    ADC_Motor.init();

    setupHeadingController();
}

void systemlib::readSensors(void)
{
    // Zyklische Messung der Laufzeit in ms (read -> read)
    cycleTime = cycleTimer;
    cycleTime_us = cycleTimer_us;
    delay(debugTimeout);
    cycleTimer = 0;
    cycleTimer_us = 0;

    readBNO();
    Radar.update();
    calculatePosition();
    // readIR();

    readExpanders();

    if (PDS.linkOk())
        PDS.update();

    updateHeadingController();
    updateMotors();
}

/* -------------- Position -------------- */

/**
 * Alter eines Messwerts -> Gewicht zwischen 1 (brandaktuell) und 0 (uralt).
 * Faellt linear von 1 bei 0 ms auf 0 bei POS_AGE_MAX_MS.
 */
static float posAgeFactor(uint32_t age_ms)
{
    if (age_ms >= POS_AGE_MAX_MS)
        return 0.0f;

    return 1.0f - ((float)age_ms / (float)POS_AGE_MAX_MS);
}

/**
 * Tiefe, um die die Y-Achse an EINEM Feldende durch das Tor kuerzer wird.
 *
 * Beide Tore stehen mittig auf der X-Achse und gleich tief im Feld, der Wert
 * gilt also fuer beide Enden. Ob der Sensor die Torrueckwand oder die Feldwand
 * trifft, haengt allein davon ab, wo der Roboter auf X steht:
 *
 *   Abstand zur Feldmitte + POS_GOAL_EDGE_CM < Torbreite/2  -> sicher Tor
 *   Abstand zur Feldmitte - POS_GOAL_EDGE_CM > Torbreite/2  -> sicher Wand
 *   dazwischen -> nicht entscheidbar, die Radarkeule deckt beides ab
 *
 * Im Uebergangsband wird linear zwischen beiden Faellen geblendet -- der Fehler
 * bleibt damit unter der halben Tortiefe statt sie voll zu treffen -- und
 * sureness faellt auf 0, weil eben nicht feststeht, was der Sensor gesehen hat.
 *
 * @param x_cm      aktuelle X-Position (muss VOR der Y-Achse gerechnet sein)
 * @param sureness  [out] 1 = eindeutig Tor oder eindeutig Wand ... 0 = genau auf der Torkante
 * @return Verkuerzung der Achse an diesem Ende in cm
 */
float systemlib::goalOffset(float x_cm, float &sureness) const
{
    sureness = 1.0f;

    // Kein Tor eingetragen -> nichts zu korrigieren, und vor allem keine
    // Certainty verschenken.
    if (POS_GOAL_DEPTH_CM == 0.0f || POS_GOAL_WIDTH_CM <= 0.0f)
        return 0.0f;

    const float halfGoal = POS_GOAL_WIDTH_CM / 2.0f;
    const float dx       = fabsf(x_cm - (feld_x / 2.0f)); // Abstand zur Feldmitte

    if (dx <= halfGoal - POS_GOAL_EDGE_CM) // klar vor der Toroeffnung
        return POS_GOAL_DEPTH_CM;

    if (dx >= halfGoal + POS_GOAL_EDGE_CM) // klar vor der Feldwand
        return 0.0f;

    // Uebergangsband: t = 1 an der Innenkante (ganz Tor) ... 0 an der Aussenkante
    // (ganz Wand). Genau auf der Torkante (t = 0.5) ist die Entscheidung ein
    // Muenzwurf -> sureness = 0.
    const float t = constrain((halfGoal + POS_GOAL_EDGE_CM - dx) / (2.0f * POS_GOAL_EDGE_CM), 0.0f, 1.0f);
    sureness = fabsf(2.0f * t - 1.0f);

    return POS_GOAL_DEPTH_CM * t;
}

/**
 * Eine Achse aus dem gegenueberliegenden Radar-Paar bestimmen.
 *
 * posLow  misst gegen die Flaeche bei Koordinate offsetLow,
 * posHigh misst gegen die Flaeche bei Koordinate feldSize - offsetHigh.
 * Ohne Tor sind beide Offsets 0 und die Flaechen sind die Feldwaende; steht der
 * Roboter vor einem Tor, ruecken sie um die Tortiefe ins Feld (siehe
 * goalOffset()). Die Strecke zwischen den beiden Flaechen heisst hier span --
 * genau sie messen die zwei Sensoren gemeinsam aus, und daran wird das Paar
 * geprueft:
 *
 *   1. Ein Wert taugt nur, wenn der Sensor antwortet, einen Peak gefunden hat
 *      und die Distanz ueberhaupt ins Feld passt.
 *   2. Misst ein Sensor mehr als span - POS_BLIND_CM, steht der Roboter
 *      dicht an der gegenueberliegenden Wand. Der Sensor dort ist naeher dran
 *      als der Radar-Blindbereich (RADAR_START_MM) -- sein Wert wird verworfen,
 *      weil die Wand fuer ihn zu nah zum Erkennen ist.
 *   3. Bleiben beide uebrig: Summe gegen span pruefen. Passt sie, ist es
 *      ein sauberes Paar -> Mittelwert. Passt sie nicht, hat einer der beiden
 *      etwas anderes als die Wand gesehen. Welcher, verraet die Richtung des
 *      Fehlers: ist die Summe zu KLEIN, hat einer zu kurz gemessen (Gegner oder
 *      Ball im Weg) -> der laengere Weg gewinnt. Ist sie zu GROSS, hat einer
 *      die Wand verpasst (Mehrwegecho) -> der kuerzere Weg gewinnt. In beiden
 *      Faellen bleibt die Plausibilitaet niedrig, es ist geraten.
 *
 * Ist gar nichts brauchbar, bleibt lastPos stehen und certainty wird 0.
 *
 * @param feldSize   Feldgroesse dieser Achse in cm (Wand zu Wand)
 * @param offsetLow  wie weit die Flaeche am Nullpunkt ins Feld ragt (Tor)
 * @param offsetHigh wie weit die Flaeche am anderen Ende ins Feld ragt (Tor)
 * @param lastPos    letzte bekannte Position, Rueckfallwert
 * @param certainty  [out] 0 = unbrauchbar ... 1 = plausibles Paar, brandaktuell
 * @return Position auf dieser Achse in cm
 */
float systemlib::calcAxisPosition(uint8_t posLow, uint8_t posHigh, float feldSize,
                                  float offsetLow, float offsetHigh,
                                  float lastPos, float &certainty)
{
    const RadarData &low  = Radar[posLow];
    const RadarData &high = Radar[posHigh];

    // Strecke, die die beiden Sensoren tatsaechlich gemeinsam ausmessen. Ohne
    // Tor ist das die volle Feldgroesse, sonst je Ende um die Tortiefe kuerzer.
    // Gerechnet wird alles in dieser Strecke (0 = Flaeche am Nullpunkt), erst
    // das Ergebnis wird mit offsetLow zurueck in Feldkoordinaten geschoben.
    const float span = feldSize - offsetLow - offsetHigh;

    // Rohwert -> cm. Der XM125 liefert unkalibrierte Counts, die aktuell 1:1
    // als mm gelesen werden (siehe XM125Distance.h). POS_RADAR_OFFSET_CM holt
    // den Weg von der Sensorflaeche zum Roboter-Mittelpunkt dazu, weil die
    // Koordinaten ab Mittelpunkt zaehlen (siehe Coding_Rules.md).
    const float distLow  = (low.Dist  / 10.0f) + POS_RADAR_OFFSET_CM;
    const float distHigh = (high.Dist / 10.0f) + POS_RADAR_OFFSET_CM;

    const float minDist = (RADAR_START_MM / 10.0f) + POS_RADAR_OFFSET_CM;

    // 1. Grundpruefung je Sensor
    bool lowOk  = low.isCon  && (low.Peaks  > 0) && (distLow  >= minDist) && (distLow  <= span + POS_PAIR_TOL_CM);
    bool highOk = high.isCon && (high.Peaks > 0) && (distHigh >= minDist) && (distHigh <= span + POS_PAIR_TOL_CM);

    // 2. Blindbereich des Gegenuebers
    const bool lowFar  = lowOk  && (distLow  > span - POS_BLIND_CM);
    const bool highFar = highOk && (distHigh > span - POS_BLIND_CM);

    if (lowFar && highFar)
    {
        // Beide melden "fast das ganze Feld" -- das kann nicht sein, keiner der
        // beiden Werte ist zu gebrauchen.
        lowOk  = false;
        highOk = false;
    }
    else if (lowFar)
        highOk = false;
    else if (highFar)
        lowOk = false;

    // 3. Position und Plausibilitaet (alles relativ zur Flaeche am Nullpunkt)
    float pos   = lastPos - offsetLow;
    float plaus = 0.0f;
    uint32_t age = 0;

    if (lowOk && highOk)
    {
        const float err = fabsf((distLow + distHigh) - span);

        if (err <= POS_PAIR_TOL_CM)
        {
            // Paar passt zur Strecke: beide Messungen mitteln.
            pos   = (distLow + (span - distHigh)) / 2.0f;
            plaus = POS_PLAUS_PAIR - (err / POS_PAIR_TOL_CM) * (POS_PLAUS_PAIR - POS_PLAUS_SINGLE);
            age   = max(Radar.giveAge(posLow), Radar.giveAge(posHigh));
        }
        else
        {
            // Summe zu klein -> einer hat ein Hindernis vor der Wand gesehen und
            // damit zu kurz gemessen. Summe zu gross -> einer hat die Wand
            // verpasst und zu weit gemessen. Der jeweils andere ist der bessere.
            const bool sumTooSmall = ((distLow + distHigh) < span);
            const bool useLow      = sumTooSmall ? (distLow >= distHigh) : (distLow <= distHigh);

            pos   = useLow ? distLow : (span - distHigh);
            plaus = POS_PLAUS_CONFLICT;
            age   = Radar.giveAge(useLow ? posLow : posHigh);
        }
    }
    else if (lowOk)
    {
        pos   = distLow;
        plaus = POS_PLAUS_SINGLE;
        age   = Radar.giveAge(posLow);
    }
    else if (highOk)
    {
        pos   = span - distHigh;
        plaus = POS_PLAUS_SINGLE;
        age   = Radar.giveAge(posHigh);
    }
    else
    {
        // Nichts Brauchbares: letzte Position halten und das ehrlich melden.
        certainty = 0.0f;
        return lastPos;
    }

    // 4. Alter einrechnen: frische Werte behalten ihre Plausibilitaet, alte
    //    verlieren sie linear bis auf 0.
    certainty = constrain(plaus * posAgeFactor(age), 0.0f, 1.0f);

    // Zurueck in Feldkoordinaten: der Nullpunkt der Strecke liegt bei offsetLow.
    return offsetLow + constrain(pos, 0.0f, span);
}

/**
 * Feldposition aus den vier Radarwerten. Einmal je Zyklus aus readSensors(),
 * direkt nach Radar.update().
 *
 * Setzt voraus, dass der Roboter zum Feld ausgerichtet ist (Heading-Regler) --
 * jeder Sensor schaut dann senkrecht auf "seine" Wand.
 *
 * Reihenfolge ist nicht beliebig: die Seitenwaende laufen ohne Unterbrechung
 * durch, an X aendern die Tore also nichts. Die Y-Achse dagegen ist um die
 * Tortiefe kuerzer, sobald der Roboter vor einer Toroeffnung steht -- und ob er
 * das tut, sagt allein X. Also erst X, daraus der Tor-Offset, dann Y.
 *
 * Certainty gilt fuer die Position als Ganzes und ist deshalb die schlechtere
 * der beiden Achsen: eine Achse, die keiner kennt, macht auch die andere als
 * Position wertlos. Damit ist auch der Fall abgedeckt, dass X unbrauchbar ist
 * und die Torentscheidung deshalb auf einem alten X-Wert beruht.
 *
 * @return Position (X/Y in cm) mit Certainty 0 ... 1
 */
PositionData systemlib::calculatePosition(void)
{
    float certainty_x = 0.0f;
    float certainty_y = 0.0f;

    float lastX = Position.X;
    float lastY = Position.Y;

    // X: links misst gegen x = 0, rechts gegen x = feld_x. Beide Seitenwaende
    // sind durchgehend, hier gibt es nichts zu korrigieren.
    Position.X = calcAxisPosition(RadarLi, RadarRe, feld_x, 0.0f, 0.0f, Position.X, certainty_x);

    // Tore: stehen an beiden Enden mittig im Weg und machen die Y-Achse dort um
    // die Tortiefe kuerzer. Da beide gleich gebaut sind und mittig sitzen, gilt
    // derselbe Offset fuer vorne und hinten.
    float goalSure = 1.0f;
    const float goalOff = goalOffset(Position.X, goalSure);

    // Y: hinten misst gegen y = 0 (eigenes Tor), vorne gegen y = feld_y.
    Position.Y = calcAxisPosition(RadarHi, RadarVo, feld_y, goalOff, goalOff, Position.Y, certainty_y);

    // Steht der Roboter genau auf der Torkante, ist nicht entschieden, ob die
    // Sensoren Torrueckwand oder Feldwand gesehen haben -- das kostet Y an
    // Certainty, auch wenn das Paar in sich sauber aussieht.
    certainty_y *= POS_PLAUS_GOAL_EDGE + (1.0f - POS_PLAUS_GOAL_EDGE) * goalSure;

    Position.AtGoal   = (goalOff != 0.0f);
    Position.Certainty = min(certainty_x, certainty_y);

    if(Position.Certainty<0.1 && lastvaliddata_Radar < 2000){
        Position.X = lastX;
        Position.Y = lastY;
    }
    else if (Position.Certainty<0.1 && lastvaliddata_Radar >= 2000){
        Position.X = -1;
        Position.Y = -1;
    }

    if (Position.Certainty>=0.1){
        lastvaliddata_Radar = 0;
    }

    return Position;
}

void systemlib::calibRadar()
{
    Radar.calib();
}

/* -------------- BNO -------------- */
void systemlib::calibBNO(void)
{
    BNO_calibration = BNO_eulerZ;
}

void systemlib::readBNO(void)
{
    BNO_eulerZ = BNO.eulHeading();
    BNO_OrbitDir = Util.AMap(BNO_eulerZ - BNO_calibration);
}

/* -------------- IR -------------- */
void systemlib::readIR(void)
{
    IR_rawAngle = ir_ring.read_ball_angle();
    IR_rawDistance = ir_ring.read_ball_distance();

#if false
    ir_ring.read_raw_values(IR_rawSensorData);
    ir_ring.read_tssp(IR_TSSP);
#endif

    IR_Angle = Util.AMap(IR_rawAngle - BNO_OrbitDir);
    IR_Distance = IR_rawDistance; // Hier muss noch Mathe hin
}

void systemlib::writeIRCalibration(uint8_t sensorNumber, int16_t offset, int16_t gain)
{
    ir_ring.write_offset(sensorNumber, offset);
    ir_ring.write_gain(sensorNumber, gain);
}

void systemlib::writeIRCalibration(const int16_t (&offsets)[16], const uint16_t (&gains)[16])
{
    ir_ring.write_offsets(offsets);
    ir_ring.write_gains(gains);
}

void systemlib::calibIR(void)
{
    ir_ring.read_raw_values(IR_rawSensorData); // only needed if disabled in readIR()

    for (int i = 0; i < 16; i++)
    {
        if (IR_rawSensorData[i] > IR_Gains[i])
            IR_Gains[i] = IR_rawSensorData[i];

        if (IR_rawSensorData[i] < IR_Offsets[i])
            IR_Offsets[i] = IR_rawSensorData[i];
    }

    ir_ring.write_offsets(IR_Offsets);
    ir_ring.write_gains(IR_Gains);
}

/* ---------- Communication -----------*/

// send

void systemlib::communicationSend(void)
{
    communication.sendOwnFlush();
}

void systemlib::communicationSend(float positionX, float positionY, float ballAngle, float ballDistance, float goalAngle, float goalDistance)
{
    communication.setOwnFlush(positionX, positionY, ballAngle, ballDistance, goalAngle, goalDistance);
    communication.sendOwnFlush();
}

float systemlib::giveMainVoltage(void)
{
    return (float)INA219_Main.getBusVoltage();
}

float systemlib::giveMotorVoltage(void)
{
    return (float)INA219_Motor.getBusVoltage();
}

float systemlib::giveMainCurrent(void)
{
    return (float)INA219_Main.getCurrent_mA();
}

float systemlib::giveMotorCurrent(void)
{
    INA_filter[0] = INA_filter[1];
    INA_filter[1] = INA_filter[2];
    INA_filter[2] = INA_filter[3];
    INA_filter[3] = INA_filter[4];
    INA_filter[4] = INA_filter[5];
    INA_filter[5] = INA219_Motor.getCurrent_mA();

    int Summe = 0;
    for (int i = 0; i < 6; i++)
    {
        Summe += INA_filter[i];
    }
    Summe = Summe / 6;
    return Summe;
}

void systemlib::setMotorData(uint8_t ID, uint8_t Speed, bool Dir, bool Enable, bool Break){
    if (ID == MoDataVR.ID)
    {
        MoDataVR.Break = Break;
        MoDataVR.Speed = Speed;
        MoDataVR.Enable = Enable;
        MoDataVR.Dir = Dir;
    }
    else if (ID == MoDataHR.ID)
    {
        MoDataHR.Break = Break;
        MoDataHR.Speed = Speed;
        MoDataHR.Enable = Enable;
        MoDataHR.Dir = Dir;
    }
    else if (ID == MoDataVL.ID)
    {
        MoDataVL.Break = Break;
        MoDataVL.Speed = Speed;
        MoDataVL.Enable = Enable;
        MoDataVL.Dir = Dir;
    }
    else if (ID == MoDataHL.ID)
    {
        MoDataHL.Break = Break;
        MoDataHL.Speed = Speed;
        MoDataHL.Enable = Enable;
        MoDataHL.Dir = Dir;
    }
    else if (ID == MoDataDR.ID)
    {
        MoDataDR.Break = Break;
        MoDataDR.Speed = Speed;
        MoDataDR.Enable = Enable;
        MoDataDR.Dir = Dir;
    }
}

void systemlib::setMotorData(uint8_t ID, int8_t Speed, bool Enable, bool Break){
    bool Dir;
    
    if (Speed<0){Dir=false;}
    else{Dir=true;}

    Speed = abs(Speed);

    if (ID == MoDataVR.ID)
    {
        MoDataVR.Break = Break;
        MoDataVR.Speed = Speed;
        MoDataVR.Enable = Enable;
        MoDataVR.Dir = Dir;
    }
    else if (ID == MoDataHR.ID)
    {
        MoDataHR.Break = Break;
        MoDataHR.Speed = Speed;
        MoDataHR.Enable = Enable;
        MoDataHR.Dir = Dir;
    }
    else if (ID == MoDataVL.ID)
    {
        MoDataVL.Break = Break;
        MoDataVL.Speed = Speed;
        MoDataVL.Enable = Enable;
        MoDataVL.Dir = Dir;
    }
    else if (ID == MoDataHL.ID)
    {
        MoDataHL.Break = Break;
        MoDataHL.Speed = Speed;
        MoDataHL.Enable = Enable;
        MoDataHL.Dir = Dir;
    }
    else if (ID == MoDataDR.ID)
    {
        MoDataDR.Break = Break;
        MoDataDR.Speed = Speed;
        MoDataDR.Enable = Enable;
        MoDataDR.Dir = Dir;
    }
}

void systemlib::updateMotors(void)
{
    uint8_t Port_List[8];
    byte out;
    if (MoDataVL.Speed == 0 || giveExpander(0,3))
    {
        MoDataVL.Speed = 0;
        MoDataVL.Enable = false;
        MoDataVL.Break = true;
    }
    if (MoDataVR.Speed == 0 || giveExpander(0,3))
    {
        MoDataVR.Speed = 0;
        MoDataVR.Enable = false;
        MoDataVR.Break = true;
    }
    if (MoDataHL.Speed == 0 || giveExpander(0,3))
    {
        MoDataHL.Speed = 0;
        MoDataHL.Enable = false;
        MoDataHL.Break = true;
    }
    if (MoDataHR.Speed == 0 || giveExpander(0,3))
    {
        MoDataHR.Speed = 0;
        MoDataHR.Enable = false;
        MoDataHR.Break = true;
    }
    if (MoDataDR.Speed == 0 || giveExpander(0,3))
    {
        MoDataDR.Speed = 0;
        MoDataDR.Enable = false;
        MoDataDR.Break = true;
    }

    analogWrite(MotorVL_PWM, abs(MoDataVL.Speed) * 2.55);
    analogWrite(MotorVR_PWM, abs(MoDataVR.Speed) * 2.55);
    analogWrite(MotorHR_PWM, abs(MoDataHR.Speed) * 2.55);
    analogWrite(MotorHL_PWM, abs(MoDataHL.Speed) * 2.55);

    if (MoDataVL.Dir)
    {
        Port_List[MotorVL_P1] = 0;
    }
    else
    {
        Port_List[MotorVL_P1] = 1;
    }
    if (MoDataVR.Dir)
    {
        Port_List[MotorVR_P1] = 0;
    }
    else
    {
        Port_List[MotorVR_P1] = 1;
    }
    if (MoDataHL.Dir)
    {
        Port_List[MotorHL_P1] = 0;
    }
    else
    {
        Port_List[MotorHL_P1] = 1;
    }
    if (MoDataHR.Dir)
    {
        Port_List[MotorHR_P1] = 0;
    }
    else
    {
        Port_List[MotorHR_P1] = 1;
    }

    if (MoDataVL.Enable)
    {
        Port_List[MotorVL_P2] = 1;
    }
    else
    {
        Port_List[MotorVL_P2] = 0;
    }
    if (MoDataVR.Enable)
    {
        Port_List[MotorVR_P2] = 1;
    }
    else
    {
        Port_List[MotorVR_P2] = 0;
    }
    if (MoDataHL.Enable)
    {
        Port_List[MotorHL_P2] = 1;
    }
    else
    {
        Port_List[MotorHL_P2] = 0;
    }
    if (MoDataHR.Enable)
    {
        Port_List[MotorHR_P2] = 1;
    }
    else
    {
        Port_List[MotorHR_P2] = 0;
    }

    // Konvertieren von Liste zu byte
    for (int i = 0; i < 8; i++)
    {
        if (Port_List[i] == 1)
        {
            out |= (1 << (7 - i));
        }
    }

    Motor_EXP.write(out);

    MoDataVL.Speed_real = ADC_Motor.read(MotorVL_AP1, true, true);
    MoDataVL.Current = ADC_Motor.read(MotorVL_AP2, true, false);
    MoDataVR.Speed_real = ADC_Motor.read(MotorVR_AP1, true, true);
    MoDataVR.Current = ADC_Motor.read(MotorVR_AP2, true, false);
    MoDataHL.Speed_real = ADC_Motor.read(MotorHL_AP1, true, true);
    MoDataHL.Current = ADC_Motor.read(MotorHL_AP2, true, false);
    MoDataHR.Speed_real = ADC_Motor.read(MotorHR_AP1, true, true);
    MoDataHR.Current = ADC_Motor.read(MotorHR_AP2, true, false);
}

MotorData systemlib::giveMotorData(uint8_t ID)
{
    MotorData Error_Data;
    Error_Data.ID = 100;

    if (ID == MoDataVR.ID)
    {
        return MoDataVR;
    }
    else if (ID == MoDataHR.ID)
    {
        return MoDataHR;
    }
    else if (ID == MoDataVL.ID)
    {
        return MoDataVL;
    }
    else if (ID == MoDataHL.ID)
    {
        return MoDataHL;
    }
    else if (ID == MoDataDR.ID)
    {
        return MoDataDR;
    }
    else
    {
        return Error_Data;
    }
}

/* -------------- Expanders -------------- */
void systemlib::readExpanders(void)
{
    SW_EXP.read();     // Update the internal state of the port expander
    Button_EXP.read(); // Update the internal state of the port expander

    SWI[0] = SW_EXP.readPin(4);
    SWI[1] = SW_EXP.readPin(3);
    SWI[2] = SW_EXP.readPin(2);
    SWI[3] = SW_EXP.readPin(1);
    SWI[4] = SW_EXP.readPin(0);

    startSwitch = SW_EXP.readPin(5);
    RCJ = Button_EXP.readPin(5);
    RCJ_Disable = SW_EXP.readPin(6);
    RCJ_present = SW_EXP.readPin(7);

    BTN1.update(Button_EXP.readPin(0));
    BTN2.update(Button_EXP.readPin(1));
    BTN3.update(Button_EXP.readPin(2));
    BTN4.update(Button_EXP.readPin(3));
    BTN5.update(Button_EXP.readPin(4));

    fancyBTNstates[0] = BTN1.getStatusCode();
    fancyBTNstates[1] = BTN2.getStatusCode();
    fancyBTNstates[2] = BTN3.getStatusCode();
    fancyBTNstates[3] = BTN4.getStatusCode();
    fancyBTNstates[4] = BTN5.getStatusCode();

    // Read the state of each pin from the port expander
    // Example: bool pin0State = portExpander.getPinState(0);
}

bool systemlib::giveExpander(uint8_t expnum, uint8_t portnum)
{
    if (expnum == 0)
    {
        return SW_EXP.readPin(portnum);
    }
    else if (expnum == 1)
    {
        return Button_EXP.readPin(portnum);
    }
    else
    {
        return false;
    }
}

void systemlib::setupHeadingController(bool useADRC)
{
    _useADRC = useADRC;
    if (_useADRC)
    {
        cfgA.order = ADRCOrder::SECOND; // Winkel = Strecke 2. Ordnung
        cfgA.Ts = TS_NOMINAL;           // nur Fallback, siehe loop()
        cfgA.dtMin = 1e-5f;             // 10 us -- gegen dt=0
        cfgA.dtMax = 0.03f;             // 30 ms -- deckelt Ausreisser, falls das
                                        // Hauptprogramm mal laenger braucht
        cfgA.wc = 15.0f;                // Reglerbandbreite: wie schnell ausgerichtet wird
        cfgA.wo = 80.0f;                // Beobachterbandbreite: ca. 5x wc als Startwert
        cfgA.b0 = 8.0f;                 // grobe Schaetzung: wie stark omega_cmd auf
                                        // die Winkelbeschleunigung wirkt -> per Test anpassen
        cfgA.uMin = -1.0f;
        cfgA.uMax = 1.0f;
        cfgA.angleMode = true;       // WICHTIG: sorgt fuer kuerzesten Drehweg (kein +-pi Sprung)
        cfgA.outputRateLimit = 6.0f; // begrenzt Stellgroessen-Aenderungsrate (sanfter Anlauf)

        headingControllerA = new ADRC(cfgA);
        headingControllerA->reset(DEG_TO_RAD * BNO_OrbitDir);
        lastMicrosContrl = micros();
    }
    else
    {
        cfgP.Ts = TS_NOMINAL; // nur Fallback, siehe loop()
        cfgP.dtMin = 1e-5f;   // 10 us -- gegen dt=0
        cfgP.dtMax = 0.03f;   // 30 ms -- deckelt Ausreisser, falls das
                              // Hauptprogramm mal laenger braucht

        cfgP.Kp = 0.4; // Start: nur P, dann I/D schrittweise zuschalten
        cfgP.Ki = 0;
        cfgP.Kd = 0.016f;

        cfgP.uMin = -1.0f;
        cfgP.uMax = 1.0f;
        cfgP.angleMode = true;               // WICHTIG: kuerzester Drehweg, kein +-pi Sprung
        cfgP.derivativeOnMeasurement = true; // kein Kick bei Sollwinkel-Spruengen
        cfgP.Tf = 0.02f;                     // D-Anteil filtern (Winkelmessung ist meist verrauscht)
        cfgP.outputRateLimit = 6.0f;         // sanfter Anlauf / Ruckbegrenzung

        headingControllerP = new PID(cfgP);
        headingControllerP->reset(DEG_TO_RAD * BNO_OrbitDir);
        lastMicrosContrl = micros();
    }
}

void systemlib::setHeadingControllerConfig()
{
    if (_useADRC)
    {
        headingControllerA->setConfig(cfgA);
    }
    else
    {
        headingControllerP->setConfig(cfgP);
    }
}

float systemlib::giveHeadingController()
{
    if (_useADRC)
    {
        return ADRC_Out;
    }
    else
    {
        return PID_Out;
    }
}

void systemlib::setHeading(float Heading)
{
    _OrbitHeading = Heading;
}

void systemlib::resetHeadingController()
{
    headingControllerA->reset(DEG_TO_RAD * BNO_OrbitDir);
}

void systemlib::updateHeadingController()
{
    float dt = (micros() - lastMicrosContrl) * 1e-6f; // us -> s
    lastMicrosContrl = micros();

    if (_useADRC)
    {
        ADRC_Out = headingControllerA->update(_OrbitHeading, DEG_TO_RAD * giveOrbitDir(), dt);
    }
    else
    {
        PID_Out = headingControllerP->update(_OrbitHeading, DEG_TO_RAD * giveOrbitDir(), dt);
    }
}

bool systemlib::giveStart(void)
{
    if (RCJ_present)
    {
        return RCJ && !RCJ_Disable;
    }
    else
    {
        return startSwitch;
    }
}

void systemlib::moveToPoint(uint16_t X, uint16_t Y, uint8_t Speed,int16_t Andrehwinkel){

    float dx = X - Position.X;
    float dy = Y - Position.Y;

    float distanzCm = sqrtf(dy * dy + dx * dx);
    Speed = Speed*constrain(distanzCm / 40.0f, 0.0f, 1.0f); // 0 = am Ziel, 1 = 40cm oder mehr entfernt

    if(abs(distanzCm)<5){
        Speed=0;
    }

    setHeading(Andrehwinkel*DEG_TO_RAD);
    setOmniWheelDir(Util.AMap(atan2f(dx, dy) * (180.0f / PI)),Speed,giveHeadingController()*100);
}

void systemlib::moveToPoint(uint16_t X, uint16_t Y, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed, int16_t Andrehwinkel){

}

void systemlib::setOmniWheelVek(float vx, float vy, float omega) {
    float w[4];
    static const float wheelAngles[4] = {45,315,135,225};
    for (int i = 0; i < 4; ++i) {
        w[i] = -sinf(wheelAngles[i]*DEG_TO_RAD) * -vx
                  +  cosf(wheelAngles[i]*DEG_TO_RAD) * vy
                  + omega;
        w[i] = constrain(w[i], -100, 100);
        
    }
    setMotorData(MoDataVL.ID,w[0],true,false);
    setMotorData(MoDataVR.ID,w[1],true,false);
    setMotorData(MoDataHL.ID,w[2],true,false);
    setMotorData(MoDataHR.ID,w[3],true,false);
}

void systemlib::setOmniWheelDir(float directionDeg, uint8_t speedPercent, float omega) {
    directionDeg = directionDeg-giveOrbitDir();

    directionDeg = Util.AMap(directionDeg);

    float w[4];
    static const float wheelAngles[4] = {45,315,135,225};
    Serial.println("omega : "+String(omega));
 
    float directionRad = directionDeg * DEG_TO_RAD;
    float speedNorm     = speedPercent / 100.0f; // 0..1
 
    float vx = cosf(directionRad) * speedNorm * 100;
    float vy = sinf(directionRad) * speedNorm * 100;
 
    for (int i = 0; i < 4; ++i) {
        w[i] = -sinf(wheelAngles[i]*DEG_TO_RAD) * -vx
                  +  cosf(wheelAngles[i]*DEG_TO_RAD) * vy
                  + omega;
        w[i] = constrain(w[i], -100, 100);
    }

    setMotorData(MoDataVL.ID,w[0],true,false);
    setMotorData(MoDataVR.ID,w[1],true,false);
    setMotorData(MoDataHL.ID,w[2],true,false);
    setMotorData(MoDataHR.ID,w[3],true,false);
}

void systemlib::moveDir(int16_t Dir,uint8_t Speed,float Andrehwinkel){
    setHeading(Andrehwinkel*DEG_TO_RAD);
    setOmniWheelDir(Dir,Speed,giveHeadingController()*100);
}