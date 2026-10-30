/*
 * ============================================================
 *  CheckSystems.cpp — Selbsttest und Diagnose der Roboter-Hardware
 * ============================================================
 *  Geht die Baugruppen der Platinen der Reihe nach durch und sagt zu jeder,
 *  ob sie funktioniert, ob sie verdaechtig aussieht oder ob sie fehlt.
 *
 *  WICHTIG — woher die Sollwerte kommen:
 *  Dieser Sketch richtet die Hardware SELBST ein und benutzt systemlib NICHT.
 *  Alle Adressen, Busse, Pins und Kanaele stammen aus den aktuellen
 *  Schaltplaenen in pcbs/ (Main V2.0, Bottom, SwitchBoard, IR-Ring), nicht aus
 *  enum.h/params.h/systemlib.cpp — dort stecken noch Werte einer aelteren
 *  Testplatine. Die Treiberbibliotheken aus lib/ werden unveraendert benutzt,
 *  nur mit den Parametern der jetzigen Platinen angelegt.
 *  Der Abschnitt "Hardware-Karte" unten ist damit auch die Vorlage dafuer,
 *  systemlib spaeter nachzuziehen.
 *
 *  Jede Berichtszeile endet mit einem Zustand:
 *      [ OK ]   geprueft und in Ordnung
 *      [WARN]   antwortet, Werte aber nicht plausibel -> nachsehen
 *      [FAIL]   fehlt oder antwortet nicht
 *      [INFO]   nur Messwert/Zustand, nichts zu bewerten
 *      [ -- ]   nicht pruefbar (braucht Mitwirkung oder ist abgeschaltet)
 *
 *  Stufen:
 *      1   Teensy            Takt, Temperatur, RAM, Reset-Grund, Roboter-ID
 *      2   I2C "ESP"  (Wire)   Scan + Soll/Ist: BNO, IR-Ring, Switchboard, Radar
 *      3   I2C "TNS"  (Wire1)  Scan + Soll/Ist: Portexpander, INA219
 *      4   Versorgung        INA219 Akku / Motor / Dribbler
 *      5   Portexpander      IC7/IC9/IC8 + Freigaben + Switchboard
 *      6   BNO055            Chip-IDs, Selbsttest, Kalibrierung, Heading
 *      7   Radar             XM125 x4 + Gegenprobe der Achsenpaare
 *      8   IR-Ring           Sensor-ID, FW/HW, Fehlercode, 16 Rohwerte, TSSP
 *      9   Linien-ADCs       6 x ADC108S102 (A..F) + Decoder-Gegenprobe
 *      10  Motor-ADC         IC10: AO1/AO2 je ESCON-Modul MO1..MO4
 *      11  Schalter/Taster   Switchboard IC1/IC2, Startschalter, RCJ
 *      11b Kontaktdurchlauf  jeder Schalter/Taster einzeln (braucht Mitwirkung)
 *      12  Aktortest         je Motor anfahren; findet auch heraus, welcher
 *                            ESCON-Analogausgang Drehzahl und welcher Strom ist
 *
 *  Stufen 1-11 sind passiv: es wird nur gelesen. Geschaltet werden allein die
 *  Sensor-Freigaben (Linien-LEDs, Lichtschranken-LED, 3V3 des I2C-Intercon) —
 *  ohne die sehen ganze Baugruppen tot aus. Die Motorspannung VMOT bleibt aus;
 *  sie wird nur im Aktortest kurz freigegeben, und der startet nur auf Tasten-
 *  druck.
 *
 *  Bedienung ueber USB-Serial (115200), einzelne Tasten:
 *      a  alles (1-11)      1..9/0/s  einzelne Stufe       h  Hilfe
 *      t  Kontaktdurchlauf  m  Aktortest (aufbocken!)      l  Live-Werte
 *
 *  Zum Flashen nach src/ kopieren; test/ wird von "pio run" nicht gebaut.
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// Bewusst NICHT systemlib.h/PDS.h: beide ziehen enum.h mit den Pinnummern der
// alten Testplatine herein. RadarArray.h bringt params.h (Radar-Abstimmung,
// Feldmasse) mit — das ist von der Platinenrevision unabhaengig.
#include <BNO055.h>
#include <INA219.h>
#include <TCA9534DW.h>
#include <ADC108S102.h>
#include <ir_ring.h>
#include <RadarArray.h>

#define CHK_OUT Serial   ///< USB-Serial; der Debug-UART gehoert dem PDS

/* ============================================================
 *  Hardware-Karte — aus pcbs/ (Stand Main V2.0 vom 18.09.26)
 * ============================================================
 *
 *  ── Teensy 4.0, Main Blatt 1 ────────────────────────────────────────────
 *    0  DBG_RX        Serial1 RX  (PDS-Debugger ueber DBG-INTERCON)
 *    1  DBG_TX        Serial1 TX
 *    2  BLDC_PWM      Dribbler
 *    3  MO1_SPEED     ESCON 1 Sollwert (AI1+)
 *    4  MO2_SPEED     ESCON 2
 *    5  MO3_SPEED     ESCON 3
 *    6  MO4_SPEED     ESCON 4
 *    7  UART1_RX      Serial2 RX
 *    8  UART1_TX      Serial2 TX
 *    9  KICKER
 *   10  SPI_TNS_CS    SPI-INTERCON (Bottom-Platine)
 *   11  SPI_TNS_MOSI  \
 *   12  SPI_TNS_MISO   >  Arduino-Objekt "SPI"
 *   13  SPI_TNS_SCK   /
 *   14  frei
 *   15  UART_DIO_RX
 *   16  SCL_TNS_3V3   Wire1 SCL   (Pad AD_B1_07 = SCL1)  / auch UART_DIO_TX
 *   17  SDA_TNS_3V3   Wire1 SDA   (Pad AD_B1_06 = SDA1)
 *   18  SDA_ESP_3V3   Wire  SDA   (Pad AD_B1_01 = SDA0), ueber Jumper JP1
 *   19  SCL_ESP_3V3   Wire  SCL   (Pad AD_B1_00 = SCL0), ueber Jumper JP2
 *   20  BLDC_DO1      Dribbler-Rueckmeldung
 *   21  ROBO_ID       Roboterkennung (Roboter 1 / 2)
 *   22  RGB_DATA_3V3  WS2812B
 *   23  LDR_IN        Lichtschranke (LDR), Blatt 5
 *   24  SUBCPU_TX     25  SUBCPU_RX
 *   26  BLDC_AO2      27  BLDC_AO1
 *   28  UART2_RX      29  UART2_TX
 *   30  MOTOR_ADC_CS  Chip-Select des Motor-ADC
 *   31  SPI2_DEC_A    \
 *   32  SPI2_DEC_B     >  74HC138 auf der Bottom-Platine (A=LSB)
 *   33  SPI2_DEC_C    /
 *
 *  ── Die zwei I2C-Busse ──────────────────────────────────────────────────
 *  "TNS" = Wire1 (16/17): alles 3V3 auf der Main-Platine.
 *  "ESP" = Wire  (18/19): BNO055 und — ueber den Levelshifter auf Blatt 10 —
 *          der 5V-I2C-INTERCON, an dem die externen Module haengen.
 *          Jumper JP1/JP2 verbinden den Teensy mit diesem Bus; ohne sie
 *          antwortet dort nichts.
 *
 *  ── Portexpander TCA9534DW ──────────────────────────────────────────────
 *  IC7 0x20 @Wire1   P0 MO3_DO1   P1 MO3_DIO3  P2 MO3_DIO4
 *                    P3 MO4_DO1   P4 MO4_DIO3  P5 MO4_DIO4
 *                    P6 SWITCH1*  P7 BUTTON1*          (* = Eingang)
 *  IC9 0x22 @Wire1   P0 MO1_DO1   P1 MO1_DIO3  P2 MO1_DIO4
 *                    P3 MO2_DO1   P4 MO2_DIO3  P5 MO2_DIO4
 *                    P6 BLDC_DIO1 P7 BLDC_DIO2
 *  IC8 0x23 @Wire1   P0 VDRIB_EN  P1 VMOT_EN   P2 LIGHT_BARRIER_EN
 *                    P3 LINE_EN_T P4 UART_DIO1 P5 I2C_INTERCON_3V3_EN
 *  SwitchBoard (am I2C-INTERCON, also @Wire):
 *  IC1 0x26 (JP1-3)  P0 SW5  P1 SW4  P2 SW3  P3 SW2
 *                    P4 SW1  P5 START_SW  P6 RCJ_DISABLE  P7 RCJ_DET
 *  IC2 0x27 (JP4-6)  P0 BTN1 P1 BTN2 P2 BTN3 P3 BTN4
 *                    P4 BTN5 P5 RCJ_OUT1   P6 RCJ_OUT2    P7 PX2_DIO
 *
 *  ── Motor-ADC IC10 (Main Blatt 5): ADC108S102 an SPI, CS = Pin 30 ───────
 *    IN0 MO1_AO1   IN1 MO1_AO2   IN2 MO2_AO1   IN3 MO2_AO2
 *    IN4 MO3_AO1   IN5 MO3_AO2   IN6 MO4_AO1   IN7 MO4_AO2
 *  Welcher der beiden Analogausgaenge Drehzahl und welcher Strom ist, steht
 *  in der ESCON-Konfiguration, nicht im Schaltplan — Stufe 12 ermittelt es.
 *
 *  ── Linien-ADCs (Bottom Blatt 2): 6 x ADC108S102 an SPI ─────────────────
 *  Chip-Select ueber den 74HC138: Decoder-Adresse 0..5 waehlt ADC A..F
 *  (Y0..Y5), 6 den Maussensor (Y6), 7 waehlt nichts aus.
 *  LINE_EN (= LINE_EN_T, IC8 P3) muss HIGH sein, sonst haben die Linien-LEDs
 *  keine Stromversorgung — so steht es auf Bottom Blatt 1.
 *
 *  ── INA219 ──────────────────────────────────────────────────────────────
 *  0x40 Akku (VBAT-4S, hinter LTC4365: UV 13,2 V / OV 24,3 V)  @Wire1
 *  0x41 Motor (VMOT, frei ueber VMOT_EN UND Schalter SW3)      @Wire1
 *  0x45 Dribbler (VDrib, frei ueber VDRIB_EN UND SW19)         @Wire1
 *
 *  ── Nicht auf den aktuellen Platinen ────────────────────────────────────
 *  Portexpander 0x21, Ultraschall SRF08: in keinem der Plaene zu finden.
 *  Der Maussensor haengt auf der Bottom-Platine am Decoder (Adresse 6); ein
 *  zweiter Steckplatz auf Main Blatt 10 gehoert dem Sub-Prozessor, nicht dem
 *  Teensy.
 * ============================================================ */

namespace Pin
{
constexpr uint8_t MO_SPEED[4]  = { 3, 4, 5, 6 }; ///< MO1..MO4, ESCON-Sollwert
constexpr uint8_t BLDC_PWM     = 2;
constexpr uint8_t KICKER       = 9;
constexpr uint8_t SPI_CS_LINE  = 10; ///< CS des SPI-INTERCON (Bottom)
constexpr uint8_t ROBO_ID      = 21;
constexpr uint8_t LDR_IN       = 23; ///< Lichtschranke
constexpr uint8_t MOTOR_ADC_CS = 30;
constexpr uint8_t DEC_A        = 31; ///< 74HC138 A (LSB)
constexpr uint8_t DEC_B        = 32;
constexpr uint8_t DEC_C        = 33;
} // namespace Pin

/* ---- I2C-Adressen ---- */
constexpr uint8_t ADR_IC7      = 0x20; ///< Portexpander MO3/MO4   @Wire1
constexpr uint8_t ADR_IC9      = 0x22; ///< Portexpander MO1/MO2   @Wire1
constexpr uint8_t ADR_IC8      = 0x23; ///< Portexpander Freigaben @Wire1
constexpr uint8_t ADR_INA_AKKU = 0x40;
constexpr uint8_t ADR_INA_MOT  = 0x41;
constexpr uint8_t ADR_INA_DRIB = 0x45;
constexpr uint8_t ADR_BNO      = 0x28; ///< @Wire
constexpr uint8_t ADR_IRRING   = 0x0A; ///< @Wire, Adresse aus der STM32-Firmware
constexpr uint8_t ADR_SB_SW    = 0x26; ///< SwitchBoard IC1, Vorgabe (Jumper!)
constexpr uint8_t ADR_SB_BTN   = 0x27; ///< SwitchBoard IC2, Vorgabe (Jumper!)

/* ---- Bitpositionen auf IC8 (Freigaben) ---- */
constexpr uint8_t EN_VDRIB    = 0;
constexpr uint8_t EN_VMOT     = 1;
constexpr uint8_t EN_BARRIER  = 2;
constexpr uint8_t EN_LINE     = 3;
constexpr uint8_t EN_UART_DIO = 4;
constexpr uint8_t EN_INTERCON = 5;

/// Ruhezustand von IC8: nur Sensor- und LED-Versorgung an, Leistung aus.
/// Ohne LINE_EN sind die Linien-LEDs dunkel, ohne INTERCON fehlt den externen
/// Modulen am 3V3-Intercon die Versorgung — beides wuerde als Defekt aussehen.
constexpr uint8_t EN_SAFE = (1 << EN_BARRIER) | (1 << EN_VDRIB) | (1 << EN_VMOT);

/* ---- ESCON-Steuerleitungen: Annahme ueber die Belegung von DI1/DI2 ----
 * Der Schaltplan zeigt MOx_DO1 -> ESCON DI1, MOx_DIO3 -> DI2, MOx_DIO4 -> DIO3.
 * Was DI1/DI2 bedeuten, legt die ESCON-Konfiguration fest; ueblich ist
 * DI1 = Enable, DI2 = Richtung. Stimmt das bei euch nicht, nur hier aendern. */
constexpr uint8_t ESCON_P_ENABLE = 0; ///< Offset innerhalb des Dreier-Blocks
constexpr uint8_t ESCON_P_DIR    = 1;
constexpr uint8_t ESCON_P_DIO4   = 2;

/* ============================================================
 *  Geraeteobjekte — mit den Parametern der aktuellen Platinen
 * ============================================================ */

INA219 inaAkku(ADR_INA_AKKU);
INA219 inaMotor(ADR_INA_MOT);
INA219 inaDrib(ADR_INA_DRIB);

BNO055 bno(ADR_BNO, &Wire);

TCA9534DW expMO34(&Wire1, ADR_IC7); ///< IC7
TCA9534DW expMO12(&Wire1, ADR_IC9); ///< IC9
TCA9534DW expEN(&Wire1, ADR_IC8);   ///< IC8

// Motor-ADC: eigener Chip-Select, kein Decoder.
ADC108S102 adcMotor(&SPI, Pin::MOTOR_ADC_CS);

// Linien-ADCs: Chip-Select ueber den 74HC138, letztes Argument = Decoderadresse.
ADC108S102 adcLineA(&SPI, Pin::DEC_A, Pin::DEC_B, Pin::DEC_C, 0);
ADC108S102 adcLineB(&SPI, Pin::DEC_A, Pin::DEC_B, Pin::DEC_C, 1);
ADC108S102 adcLineC(&SPI, Pin::DEC_A, Pin::DEC_B, Pin::DEC_C, 2);
ADC108S102 adcLineD(&SPI, Pin::DEC_A, Pin::DEC_B, Pin::DEC_C, 3);
ADC108S102 adcLineE(&SPI, Pin::DEC_A, Pin::DEC_B, Pin::DEC_C, 4);
ADC108S102 adcLineF(&SPI, Pin::DEC_A, Pin::DEC_B, Pin::DEC_C, 5);
ADC108S102 *adcLine[6] = { &adcLineA, &adcLineB, &adcLineC, &adcLineD, &adcLineE, &adcLineF };

I2C_bus       irBus(Wire);
Device_handle irHandle = { irBus, I2C_target{ ADR_IRRING } };
IR_ring       irRing(irHandle);

RadarArray radar; ///< Adressen und Abstimmung aus params.h, Bus = Wire

/* ============================================================
 *  Grenzwerte der Pruefungen
 * ============================================================ */

/* ---- Versorgung (4S-LiPo, LTC4365: UV 13,2 V / OV 24,3 V) ---- */
static constexpr float CHK_AKKU_V_LEER   = 13.6f;
static constexpr float CHK_AKKU_V_MIN    = 12.0f;
static constexpr float CHK_AKKU_V_MAX    = 17.4f;
static constexpr float CHK_AKKU_V_VOLL   = 16.8f;
static constexpr float CHK_VMOT_V_MIN    = 10.0f;
static constexpr float CHK_I_RUHE_MAX_MA = 2500.0f;
static constexpr float CHK_INA_SHUNT_A   = 15.0f; ///< setMaxCurrentShunt(), wie im Betrieb

/* ---- BNO055 ---- */
static constexpr uint8_t CHK_BNO_CHIP_ID   = 0xA0;
static constexpr uint8_t CHK_BNO_ACC_ID    = 0xFB;
static constexpr uint8_t CHK_BNO_MAG_ID    = 0x32;
static constexpr uint8_t CHK_BNO_GYR_ID    = 0x0F;
static constexpr float   CHK_BNO_DRIFT_MAX = 2.0f;

/* ---- ADC108S102: read() rechnet den Rohwert auf 0..3300 mV ---- */
static constexpr uint16_t CHK_ADC_SAMPLES   = 12;
static constexpr uint16_t CHK_ADC_LOW_DEAD  = 8;
static constexpr uint16_t CHK_ADC_HIGH_DEAD = 3290;
static constexpr uint16_t CHK_LINE_SCHWELLE = 3000; ///< Richtwert, vgl. LineSensorSchwelle in params.h

/* ---- ESCON-Analogausgaenge: bipolar, Ruhelage auf der Mitte ---- */
static constexpr int16_t CHK_MOT_MID_MV   = 1650;
static constexpr int16_t CHK_MOT_MID_TOL  = 250;
static constexpr int16_t CHK_MOT_RUN_DIFF = 150;

/* ---- Radar ---- */
static constexpr uint32_t CHK_RADAR_WARMUP_MS = 4000;
static constexpr uint32_t CHK_RADAR_AGE_WARN  = POS_AGE_MAX_MS;

/* ---- Aktortest ---- */
static constexpr uint8_t  CHK_AKTOR_PWM     = 64;  ///< ~25 % von 255, langsam
static constexpr uint32_t CHK_AKTOR_RAMP_MS = 600;
static constexpr uint32_t CHK_AKTOR_STOP_MS = 400;

/* ============================================================
 *  Ausgabe
 * ============================================================ */

enum ChkState : uint8_t { CHK_OK, CHK_WARN, CHK_FAIL, CHK_INFO, CHK_SKIP };

static uint16_t cntOk, cntWarn, cntFail, cntSkip;

static constexpr uint8_t CHK_MAX_MERK = 24;
static String  merkListe[CHK_MAX_MERK];
static uint8_t merkAnzahl;

static const char *stateTag(ChkState st)
{
    switch (st)
    {
    case CHK_OK:   return "[ OK ]";
    case CHK_WARN: return "[WARN]";
    case CHK_FAIL: return "[FAIL]";
    case CHK_INFO: return "[INFO]";
    default:       return "[ -- ]";
    }
}

/// Eine Berichtszeile: Zustand, Name (auf feste Breite gefuellt), Messwert,
/// bei WARN/FAIL zusaetzlich ein Hinweis, wo man suchen sollte.
static void zeile(ChkState st, const char *name, const String &wert, const char *hinweis = nullptr)
{
    switch (st)
    {
    case CHK_OK:   cntOk++;   break;
    case CHK_WARN: cntWarn++; break;
    case CHK_FAIL: cntFail++; break;
    case CHK_SKIP: cntSkip++; break;
    default: break;
    }

    String n = name;
    while (n.length() < 26) n += '.';

    CHK_OUT.print(stateTag(st));
    CHK_OUT.print(' ');
    CHK_OUT.print(n);
    CHK_OUT.print(' ');
    CHK_OUT.println(wert);

    if (hinweis && (st == CHK_WARN || st == CHK_FAIL))
    {
        CHK_OUT.print("       -> ");
        CHK_OUT.println(hinweis);
    }

    if ((st == CHK_WARN || st == CHK_FAIL) && merkAnzahl < CHK_MAX_MERK)
        merkListe[merkAnzahl++] = String(stateTag(st)) + " " + name + ": " + wert;
}

static void ueberschrift(const char *text)
{
    CHK_OUT.println();
    String s = "-- ";
    s += text;
    s += ' ';
    while (s.length() < 64) s += '-';
    CHK_OUT.println(s);
}

static void hinweiszeile(const char *text)
{
    CHK_OUT.print("       i  ");
    CHK_OUT.println(text);
}

/* ============================================================
 *  Kleine Helfer
 * ============================================================ */

/// Antwortet unter dieser Adresse ein Geraet? 0 = ACK (TwoWire::endTransmission).
static bool i2cDa(TwoWire &bus, uint8_t addr)
{
    bus.beginTransmission(addr);
    return bus.endTransmission() == 0;
}

/// Ein Registerbyte lesen. @param ok wird false, wenn das Geraet nicht antwortet.
static uint8_t i2cReg(TwoWire &bus, uint8_t addr, uint8_t reg, bool &ok)
{
    bus.beginTransmission(addr);
    bus.write(reg);
    if (bus.endTransmission() != 0) { ok = false; return 0; }
    if (bus.requestFrom(addr, (uint8_t)1) != 1) { ok = false; return 0; }
    ok = true;
    return (uint8_t)bus.read();
}

static String hex2(uint8_t v)
{
    String s = "0x";
    if (v < 0x10) s += '0';
    s += String(v, HEX);
    return s;
}

/// Portbyte als Bitkette, Pin 7 links.
static String bitKette(uint8_t v)
{
    String s;
    for (int8_t b = 7; b >= 0; b--) s += ((v >> b) & 1) ? '1' : '0';
    return s;
}

/// Messreihe eines ADC-Kanals: Mittelwert und Spanne als Rauschmass.
struct AdcStat { int16_t mittel; int16_t min; int16_t max; };

static AdcStat adcMessen(ADC108S102 &adc, uint8_t kanal)
{
    AdcStat s;
    int32_t summe = 0;
    s.min = 32767;
    s.max = -32768;

    for (uint16_t i = 0; i < CHK_ADC_SAMPLES; i++)
    {
        const int16_t v = adc.read(kanal);   // Rohwert in mV, keine Motor-Umrechnung
        summe += v;
        if (v < s.min) s.min = v;
        if (v > s.max) s.max = v;
    }
    s.mittel = (int16_t)(summe / CHK_ADC_SAMPLES);
    return s;
}

/* ---- SwitchBoard: Adressen sind per Jumper einstellbar ---------------------
 * Vorgabe laut Plan ist 0x26/0x27. Der alte Code liest die Schalter unter
 * 0x22/0x23 — moeglich, wenn die Jumper umgesetzt sind. Gesucht wird nur auf
 * Wire, denn 0x22/0x23 auf Wire1 sind IC9 und IC8 der Main-Platine. */
static uint8_t sbSwAddr  = 0; ///< 0 = nicht gefunden
static uint8_t sbBtnAddr = 0;

static void sbAdressenSuchen(void)
{
    sbSwAddr = sbBtnAddr = 0;
    if (i2cDa(Wire, ADR_SB_SW))  sbSwAddr  = ADR_SB_SW;
    else if (i2cDa(Wire, 0x22))  sbSwAddr  = 0x22;
    if (i2cDa(Wire, ADR_SB_BTN)) sbBtnAddr = ADR_SB_BTN;
    else if (i2cDa(Wire, 0x23))  sbBtnAddr = 0x23;
}

/* ============================================================
 *  Eigene Hardware-Einrichtung
 * ============================================================
 *  Reihenfolge ist Absicht:
 *    1. Sollwerte der ESCON auf 0, bevor irgendetwas freigegeben wird.
 *    2. Ausgangsregister der Portexpander VOR der Richtungsumschaltung
 *       setzen. Nach dem Einschalten steht es auf 0xFF — ein setup(false)
 *       als erstes wuerde alle ESCON-Steuerleitungen auf High treiben.
 *    3. Freigaben: nur Sensorversorgung, VMOT und VDRIB bleiben aus.
 */

static void hardwareInit(void)
{
    /* ---- 1. Stellgroessen zuerst stillegen ---- */
    for (uint8_t i = 0; i < 4; i++)
    {
        pinMode(Pin::MO_SPEED[i], OUTPUT);
        analogWrite(Pin::MO_SPEED[i], 0);
    }
    pinMode(Pin::BLDC_PWM, OUTPUT);
    analogWrite(Pin::BLDC_PWM, 0);
    pinMode(Pin::KICKER, OUTPUT);
    digitalWrite(Pin::KICKER, LOW);

    pinMode(Pin::ROBO_ID, INPUT);
    pinMode(Pin::LDR_IN, INPUT);

    /* ---- 2. I2C ---- */
    Wire.begin();
    Wire.setClock(100000);
    Wire1.begin();
    Wire1.setClock(100000);

    /* ---- 3. Portexpander ---- */
    expMO12.write(0x00); // alle ESCON-Leitungen von MO1/MO2 auf Low
    expMO12.setup(false);

    expMO34.write(0x00);
    // P6/P7 sind SWITCH1/BUTTON1 und damit Eingaenge (true = Eingang).
    expMO34.setup(false, false, false, false, false, false, true, true);

    expEN.write(EN_SAFE);
    expEN.setup(false);
    expEN.writePin(EN_VMOT, true);  // VMOT bleibt an
    expEN.writePin(EN_VDRIB, true); // VDRIB bleibt an
    expEN.writePin(EN_INTERCON, false); // Intercon wird durch Pulldown AKTIVIERTs
    expEN.writePin(EN_LINE, true);  // Line wird durch Pullup AKTIVIERTs

    // Den externen Modulen am Intercon Zeit zum Hochlaufen geben, bevor
    // gescannt wird — sonst fehlen sie im Ergebnis.
    delay(250);

    /* ---- 4. SPI und ADCs ---- */
    pinMode(Pin::SPI_CS_LINE, OUTPUT);
    digitalWrite(Pin::SPI_CS_LINE, HIGH);
    SPI.begin();

    adcMotor.init();
    for (uint8_t i = 0; i < 6; i++) adcLine[i]->init();

    /* ---- 5. Strommessung ---- */
    inaAkku.begin(Wire1);
    inaMotor.begin(Wire1);
    inaDrib.begin(Wire1);
    inaAkku.setMaxCurrentShunt(CHK_INA_SHUNT_A);
    inaMotor.setMaxCurrentShunt(CHK_INA_SHUNT_A);
    inaDrib.setMaxCurrentShunt(CHK_INA_SHUNT_A);

    /* ---- 6. Lage und Radar ---- */
    bno.init();
    delay(1);
    radar.begin(Wire);   // blockiert bis ~2 s je Sensor

    sbAdressenSuchen();
}

/* ============================================================
 *  1  Teensy
 * ============================================================ */

#if defined(__IMXRT1062__)
extern "C" float tempmonGetTemp(void);
extern unsigned long _heap_start, _heap_end;
extern char *__brkval;
#endif

static void stufe_teensy(void)
{
    ueberschrift("1  Teensy 4.0 / Firmware");

    zeile(CHK_INFO, "Firmware", String("CheckSystems  (") + BUILD_DATE " " BUILD_TIME ")");
    zeile(CHK_INFO, "CPU-Takt", String(F_CPU_ACTUAL / 1000000UL) + " MHz");

#if defined(__IMXRT1062__)
    const float temp = tempmonGetTemp();
    zeile(temp < 70.0f ? CHK_OK : (temp < 85.0f ? CHK_WARN : CHK_FAIL),
          "CPU-Temperatur", String(temp, 1) + " C",
          "Teensy wird heiss — Luft, Takt oder Dauerlast pruefen");

    const int32_t ramFrei = (int32_t)((char *)&_heap_end - __brkval);
    zeile(ramFrei > 20000 ? CHK_OK : CHK_WARN,
          "RAM1 frei (Heap)", String(ramFrei) + " Byte",
          "wenig Luft fuer new/String — Puffergroessen pruefen");

    // SRC_SRSR sammelt die Reset-Gruende auf; nach dem Lesen zuruecksetzen,
    // damit beim naechsten Start nur der neue Grund drinsteht.
    const uint32_t srsr = SRC_SRSR;
    SRC_SRSR = srsr;
    String grund;
    if (srsr & (1u << 0)) grund += "Power-On/Reset-Pin ";
    if (srsr & (1u << 1)) grund += "Lockup/SysResetReq ";
    if (srsr & (1u << 2)) grund += "CSU ";
    if (srsr & (1u << 3)) grund += "User-Reset ";
    if (srsr & (1u << 4)) grund += "WATCHDOG ";
    if (srsr & (1u << 5)) grund += "JTAG ";
    if (srsr & (1u << 6)) grund += "JTAG-SW ";
    if (srsr & (1u << 7)) grund += "WATCHDOG3 ";
    if (srsr & (1u << 8)) grund += "Uebertemperatur ";
    if (grund.length() == 0) grund = "unbekannt ";
    grund += "(SRSR=" + String(srsr, HEX) + ")";

    zeile((srsr & ((1u << 4) | (1u << 7) | (1u << 8))) ? CHK_WARN : CHK_INFO,
          "Reset-Grund", grund,
          "letzter Neustart kam nicht vom Einschalten — Absturz/Hitze pruefen");
#endif

    // ROBO_ID unterscheidet die beiden Roboter (Main Blatt 1, Pin 21).
    zeile(CHK_INFO, "Roboter-ID (Pin 21)",
          digitalRead(Pin::ROBO_ID) ? "HIGH (Roboter 1)" : "LOW (Roboter 2)");
}

/* ============================================================
 *  2/3  I2C-Busse
 * ============================================================ */

struct I2CSoll
{
    uint8_t     addr;
    const char *name;
    uint8_t     bus;     ///< 0 = Wire ("ESP"), 1 = Wire1 ("TNS")
    bool        pflicht;
};

static const I2CSoll i2cSoll[] = {
    /* ---- Wire: BNO + alles am I2C-INTERCON ---- */
    { ADR_IRRING,   "IR-Ring",              0, true  },
    { ADR_BNO,      "BNO055",               0, true  },
    { ADR_SB_SW,    "SwitchBoard IC1",      0, true  },
    { ADR_SB_BTN,   "SwitchBoard IC2",      0, true  },
    { 0x52,         "Radar (0x52)",         0, true  },
    { 0x55,         "Radar (0x55)",         0, true  },
    { 0x56,         "Radar (0x56)",         0, true  },
    { 0x58,         "Radar (0x58)",         0, true  },
    /* ---- Wire1: Main-Platine ---- */
    { ADR_IC7,      "IC7 Expander MO3/MO4", 1, true  },
    { ADR_IC9,      "IC9 Expander MO1/MO2", 1, true  },
    { ADR_IC8,      "IC8 Expander Freigabe",1, true  },
    { ADR_INA_AKKU, "INA219 Akku",          1, true  },
    { ADR_INA_MOT,  "INA219 Motor",         1, true  },
    { ADR_INA_DRIB, "INA219 Dribbler",      1, false },
};
static constexpr uint8_t i2cSollAnzahl = sizeof(i2cSoll) / sizeof(i2cSoll[0]);

static uint8_t wireGefunden[16];
static uint8_t wire1Gefunden[16];

static uint8_t i2cScan(TwoWire &bus, uint8_t (&gefunden)[16])
{
    for (uint8_t i = 0; i < 16; i++) gefunden[i] = 0;
    uint8_t anzahl = 0;
    for (uint8_t a = 0x08; a <= 0x77; a++)
    {
        if (i2cDa(bus, a))
        {
            gefunden[a >> 3] |= (uint8_t)(1u << (a & 0x07));
            anzahl++;
        }
    }
    return anzahl;
}

static bool i2cHat(const uint8_t (&gefunden)[16], uint8_t addr)
{
    return (gefunden[addr >> 3] & (1u << (addr & 0x07))) != 0;
}

/// Name einer gefundenen Adresse, unabhaengig vom Bus.
static const I2CSoll *i2cSuchen(uint8_t addr)
{
    for (uint8_t i = 0; i < i2cSollAnzahl; i++)
        if (i2cSoll[i].addr == addr) return &i2cSoll[i];
    return nullptr;
}

static void stufe_i2c(TwoWire &bus, uint8_t busNr, uint8_t (&gefunden)[16])
{
    const bool esp = (busNr == 0);
    ueberschrift(esp ? "2  I2C \"ESP\" = Wire (Teensy 18/19, mit Intercon)"
                     : "3  I2C \"TNS\" = Wire1 (Teensy 16/17, Main-Platine)");

    const uint8_t anzahl = i2cScan(bus, gefunden);

    // Antwortet niemand, liegt es am Bus selbst — jede Folgezeile waere Rauschen.
    if (anzahl == 0)
    {
        zeile(CHK_FAIL, esp ? "Bus Wire (ESP)" : "Bus Wire1 (TNS)", "kein Geraet antwortet",
              esp ? "Jumper JP1/JP2 gesteckt? Sonst 3V3, Pull-ups, Levelshifter, Intercon-Stecker"
                  : "3V3, Pull-ups oder SDA/SCL-Kurzschluss auf der Main-Platine pruefen");
        return;
    }

    String liste;
    for (uint8_t a = 0x08; a <= 0x77; a++)
        if (i2cHat(gefunden, a)) liste += hex2(a) + " ";
    zeile(CHK_INFO, esp ? "Bus Wire (ESP)" : "Bus Wire1 (TNS)",
          String(anzahl) + " Geraete: " + liste);

    /* ---- Soll/Ist je erwartetem Geraet dieses Busses ---- */
    for (uint8_t i = 0; i < i2cSollAnzahl; i++)
    {
        const I2CSoll &s = i2cSoll[i];
        if (s.bus != busNr) continue;

        // SwitchBoard: Adressen sind per Jumper einstellbar, darum die
        // tatsaechlich gefundene Adresse bewerten, nicht die Vorgabe.
        if (s.addr == ADR_SB_SW || s.addr == ADR_SB_BTN)
        {
            const uint8_t ist  = (s.addr == ADR_SB_SW) ? sbSwAddr : sbBtnAddr;
            const uint8_t soll = s.addr;
            if (ist == soll)
                zeile(CHK_OK, s.name, hex2(soll) + " antwortet (Vorgabe)");
            else if (ist != 0)
                zeile(CHK_OK, s.name, hex2(ist) + " antwortet (Jumper abweichend, Vorgabe " + hex2(soll) + ")");
            else
                zeile(CHK_FAIL, s.name, hex2(soll) + "/" + hex2(soll - 4) + " antworten nicht",
                      "SwitchBoard haengt am I2C-INTERCON — Stecker, 5V und Adressjumper pruefen");
            continue;
        }

        if (i2cHat(gefunden, s.addr))
        {
            zeile(CHK_OK, s.name, hex2(s.addr) + " antwortet");
        }
        else
        {
            // Haengt es am anderen Bus? Dann ist nicht das Geraet kaputt,
            // sondern die Zuordnung bzw. der Stecker.
            TwoWire &anderer = esp ? Wire1 : Wire;
            const bool woanders = i2cDa(anderer, s.addr);
            zeile(s.pflicht ? CHK_FAIL : CHK_SKIP, s.name,
                  hex2(s.addr) + (woanders ? " — antwortet am ANDEREN Bus!" : " antwortet nicht"),
                  woanders ? "laut Schaltplan gehoert diese Adresse auf den anderen Bus"
                           : "Stecker, Versorgung oder Adressjumper pruefen");
        }
    }

    /* ---- Alles, was antwortet und hier nicht erwartet wird ---- */
    for (uint8_t a = 0x08; a <= 0x77; a++)
    {
        if (!i2cHat(gefunden, a)) continue;
        const I2CSoll *s = i2cSuchen(a);
        if (s && s->bus == busNr) continue;
        if (esp && (a == sbSwAddr || a == sbBtnAddr)) continue; // schon oben behandelt

        if (a == 0x21)
            zeile(CHK_INFO, "Expander 0x21", "antwortet — auf den aktuellen Plaenen nicht vorhanden");
        else if (a >= 0x70 && a <= 0x7F)
            zeile(CHK_INFO, "SRF08-Bereich", hex2(a) + " antwortet — Ultraschall ist auf keinem Plan");
        else if (s)
            zeile(CHK_WARN, s->name, hex2(a) + " an diesem Bus (erwartet am anderen)",
                  "Geraet haengt nicht am Bus, den der Schaltplan vorsieht");
        else
            zeile(CHK_INFO, "unbekannte Adresse", hex2(a) + " antwortet");
    }
}

/* ============================================================
 *  4  Versorgung
 * ============================================================ */

/// Eine INA219-Schiene bewerten. @param pflichtAn false = darf abgeschaltet sein.
static void schienePruefen(const char *name, INA219 &ina, uint8_t addr,
                           float vMin, float vMax, bool pflichtAn, const char *ausHinweis)
{
    if (!i2cDa(Wire1, addr))
    {
        zeile(pflichtAn ? CHK_FAIL : CHK_SKIP, name, hex2(addr) + " antwortet nicht",
              "INA219 fehlt — ohne ihn gibt es keine Strom-/Spannungsueberwachung");
        return;
    }

    if (!ina.isCalibrated())
        zeile(CHK_WARN, (String(name) + " Kalibrierung").c_str(), "nicht kalibriert",
              "setMaxCurrentShunt() lief nicht durch — Stromwerte bleiben 0");

    const float v  = ina.getBusVoltage();
    const float i  = ina.getCurrent_mA();
    const float sh = ina.getShuntVoltage_mV();

    if (v < CHK_VMOT_V_MIN && !pflichtAn)
    {
        zeile(CHK_SKIP, name, String(v, 2) + " V — abgeschaltet");
        hinweiszeile(ausHinweis);
        return;
    }

    ChkState st = CHK_OK;
    const char *hint = nullptr;
    if (v < vMin || v > vMax)
    {
        st = CHK_FAIL;
        hint = "ausserhalb des erwarteten Fensters — Akku, LTC4365 oder Shunt pruefen";
    }
    else if (pflichtAn && v < CHK_AKKU_V_LEER)
    {
        st = CHK_WARN;
        hint = "Akku fast leer, LTC4365 trennt bei 13,2 V — laden";
    }
    zeile(st, name, String(v, 2) + " V, " + String(i, 0) + " mA (Shunt " + String(sh, 2) + " mV)", hint);

    if (st != CHK_FAIL)
        zeile(fabsf(i) < CHK_I_RUHE_MAX_MA ? CHK_OK : CHK_WARN,
              (String(name) + " Ruhestrom").c_str(), String(i, 0) + " mA",
              "hoher Strom, obwohl nichts laeuft — Kurzschluss oder ein Antrieb zieht");
}

static void stufe_power(void)
{
    ueberschrift("4  Stromversorgung (INA219 @ Wire1)");

    schienePruefen("Akku VBAT-4S", inaAkku, ADR_INA_AKKU,
                   CHK_AKKU_V_MIN, CHK_AKKU_V_MAX, true, nullptr);
    hinweiszeile((String("4S voll = ") + String(CHK_AKKU_V_VOLL, 1) + " V, LTC4365 trennt bei 13,2 / 24,3 V").c_str());

    schienePruefen("Motor VMOT", inaMotor, ADR_INA_MOT,
                   CHK_AKKU_V_MIN, CHK_AKKU_V_MAX, false,
                   "VMOT braucht VMOT_EN (IC8 P1) UND Schalter SW3 — der Check laesst sie aus");

    schienePruefen("Dribbler VDrib", inaDrib, ADR_INA_DRIB,
                   CHK_AKKU_V_MIN, CHK_AKKU_V_MAX, false,
                   "VDrib braucht VDRIB_EN (IC8 P0) UND Schalter SW19 — der Check laesst sie aus");

    // 3V3 hat keinen eigenen Messpunkt. Dass die Expander antworten, ist der
    // indirekte Nachweis dafuer.
    const bool lebt = i2cDa(Wire1, ADR_IC7) || i2cDa(Wire1, ADR_IC9) || i2cDa(Wire1, ADR_IC8);
    zeile(lebt ? CHK_OK : CHK_WARN, "3V3 Main (indirekt)",
          lebt ? "Portexpander antworten" : "kein Portexpander antwortet",
          "ohne 3V3 schweigt der ganze TNS-Bus — TSR-Wandler und Sicherung pruefen");
}

/* ============================================================
 *  5  Portexpander
 * ============================================================
 *  TCA9534DW hat kein ID-Register. "Antwortet" kommt aus dem I2C-Scan, die
 *  Funktionsprobe aus dem Konfigurationsregister 0x03 — dort steht, was
 *  hardwareInit() geschrieben hat.
 *
 *  Gelesen wird ueber den Bus, nicht ueber TCA9534DW::read(): dessen read()
 *  schreibt den gelesenen Wert in das interne _state, aus dem writePin()
 *  spaeter das Ausgangsbyte baut. An IC7/IC9 haengen die ESCON-Freigaben.
 */

static void expanderPruefen(const char *name, TwoWire &bus, uint8_t addr,
                            uint8_t sollCfg, const char *belegung)
{
    if (!i2cDa(bus, addr))
    {
        zeile(CHK_FAIL, name, hex2(addr) + " antwortet nicht",
              "Expander fehlt — readPin() liefert dann 0xFF, also 'alle Pins high'");
        return;
    }

    bool okCfg = false, okIn = false, okOut = false;
    const uint8_t cfg = i2cReg(bus, addr, 0x03, okCfg); // Richtung, 1 = Eingang
    const uint8_t in  = i2cReg(bus, addr, 0x00, okIn);  // tatsaechliche Pegel
    const uint8_t out = i2cReg(bus, addr, 0x01, okOut); // was geschrieben wurde

    if (!okCfg || !okIn || !okOut)
    {
        zeile(CHK_WARN, name, hex2(addr) + " ACK, aber Register nicht lesbar",
              "Bus wackelt (Pull-ups, Leitungslaenge) oder Chip haengt");
        return;
    }

    const String wert = "Cfg=" + hex2(cfg) + " Pegel=" + bitKette(in) +
                        " Ausgang=" + hex2(out) + " [" + belegung + "]";

    zeile(cfg == sollCfg ? CHK_OK : CHK_WARN, name, wert,
          "Richtungsregister weicht ab — der Chip hat resettet oder etwas anderes schreibt mit");
}

static void stufe_expander(void)
{
    ueberschrift("5  Portexpander TCA9534DW");

    expanderPruefen("IC9 0x22 (MO1/MO2)", Wire1, ADR_IC9, 0x00, "ESCON 1+2 Enable/Richtung");
    // P6/P7 = SWITCH1/BUTTON1 sind Eingaenge -> Cfg 0xC0
    expanderPruefen("IC7 0x20 (MO3/MO4)", Wire1, ADR_IC7, 0xC0, "ESCON 3+4 + SWITCH1/BUTTON1");
    expanderPruefen("IC8 0x23 (Freigaben)", Wire1, ADR_IC8, 0x00, "VMOT/VDRIB/LINE/Barrier/Intercon");

    /* ---- Freigaben einzeln: ohne sie sehen ganze Stufen tot aus ---- */
    bool ok = false;
    const uint8_t en = i2cReg(Wire1, ADR_IC8, 0x01, ok);
    if (!ok)
    {
        zeile(CHK_FAIL, "Freigaben (IC8)", "nicht lesbar");
    }
    else
    {
        struct Freigabe { uint8_t bit; const char *name; bool sollAn; const char *was; };
        static const Freigabe fr[6] = {
            { EN_LINE,     "LINE_EN (P3)",      true,  "Stromversorgung der Linien-LEDs" },
            { EN_INTERCON, "I2C_INTERCON (P5)", true,  "3V3 der Intercon-Stecker" },
            { EN_BARRIER,  "LIGHT_BARRIER (P2)",true,  "LED der Lichtschranke" },
            { EN_VMOT,     "VMOT_EN (P1)",      false, "Motorspannung" },
            { EN_VDRIB,    "VDRIB_EN (P0)",     false, "Dribblerspannung" },
            { EN_UART_DIO, "UART_DIO1 (P4)",    false, "DIO zur Kicker-/Dribbler-Platine" },
        };
        for (uint8_t i = 0; i < 6; i++)
        {
            const bool an = (en >> fr[i].bit) & 1;
            if (an == fr[i].sollAn)
                zeile(CHK_OK, fr[i].name, String(an ? "ein" : "aus") + " — " + fr[i].was);
            else
                zeile(CHK_WARN, fr[i].name, String(an ? "ein" : "aus") + " — " + fr[i].was,
                      fr[i].sollAn ? "sollte fuer den Check ein sein — IC8 nimmt den Schreibzugriff nicht an"
                                   : "sollte im Ruhezustand aus sein");
        }
    }

    /* ---- SwitchBoard (am Intercon, also auf Wire) ---- */
    if (sbSwAddr)
        expanderPruefen("SwitchBoard IC1", Wire, sbSwAddr, 0xFF, "SW1-5, Start, RCJ");
    else
        zeile(CHK_FAIL, "SwitchBoard IC1", "nicht gefunden", "Intercon-Stecker und Adressjumper pruefen");

    if (sbBtnAddr)
        expanderPruefen("SwitchBoard IC2", Wire, sbBtnAddr, 0xFF, "BTN1-5, RCJ-Out");
    else
        zeile(CHK_FAIL, "SwitchBoard IC2", "nicht gefunden", "Intercon-Stecker und Adressjumper pruefen");

    hinweiszeile("IC7/IC9/IC8 werden nur gelesen; geschrieben wurde allein beim Hochfahren.");
}

/* ============================================================
 *  6  BNO055  (Main Blatt 12, am ESP-Bus = Wire)
 * ============================================================ */

/// Ein BNO-Register schreiben. @return Code von TwoWire::endTransmission (0 = ACK)
static uint8_t bnoSchreiben(uint8_t reg, uint8_t wert)
{
    Wire.beginTransmission(ADR_BNO);
    Wire.write(reg);
    Wire.write(wert);
    return Wire.endTransmission();
}

/// Zustand nach einem Versuch in einer Zeile: OPR_MODE, SYS_STATUS, SYS_ERR, PAGE_ID.
static String bnoZustand(void)
{
    bool ok;
    const uint8_t opr  = i2cReg(Wire, ADR_BNO, REG_OPR_MODE, ok);
    const uint8_t st   = i2cReg(Wire, ADR_BNO, REG_SYS_STATUS, ok);
    const uint8_t err  = i2cReg(Wire, ADR_BNO, REG_SYS_ERR, ok);
    const uint8_t page = i2cReg(Wire, ADR_BNO, REG_PAGE_ID, ok);
    return "OPR=" + hex2(opr) + " STAT=" + hex2(st) + " ERR=" + hex2(err) + " PAGE=" + hex2(page);
}

/// OPR_MODE steht nicht auf dem Sollwert: von Hand setzen, notfalls nach
/// Software-Reset, und jeweils zeigen, was der Chip daraus macht.
static void bnoModusDiagnose(void)
{
    bool ok;

    /* Versuch 1: OPR_MODE direkt schreiben */
    const uint8_t tx1 = bnoSchreiben(REG_OPR_MODE, DEFAULT_OPR_MODE);
    delay(30);
    const bool klappt1 = (i2cReg(Wire, ADR_BNO, REG_OPR_MODE, ok) & 0x0F) == DEFAULT_OPR_MODE;
    zeile(klappt1 ? CHK_OK : CHK_FAIL, "  Modus direkt setzen",
          "TX=" + String(tx1) + "  " + bnoZustand(),
          tx1 ? "Schreibzugriff ohne ACK (TX 2/3 = NACK, 4/5 = Bus/Timeout)"
              : "ACK, aber der Chip uebernimmt den Modus nicht");
    if (klappt1)
    {
        hinweiszeile("direkt gesetzt geht — bno.init() kommt zu frueh oder wird ueberschrieben");
        return;
    }

    /* Versuch 2: Software-Reset (SYS_TRIGGER.RST_SYS), Neustart abwarten, erneut setzen */
    const uint8_t txR = bnoSchreiben(REG_SYS_TRIGGER, 1 << 5);
    const uint32_t t0 = millis();
    uint8_t id = 0;
    delay(50);
    while (millis() - t0 < 1000)
    {
        id = i2cReg(Wire, ADR_BNO, REG_CHIP_ID, ok);
        if (ok && id == CHK_BNO_CHIP_ID) break;
        delay(10);
    }
    const uint32_t tBoot = millis() - t0;
    bnoSchreiben(REG_PAGE_ID, 0);
    const uint8_t tx2 = bnoSchreiben(REG_OPR_MODE, DEFAULT_OPR_MODE);
    delay(30);
    const bool klappt2 = (i2cReg(Wire, ADR_BNO, REG_OPR_MODE, ok) & 0x0F) == DEFAULT_OPR_MODE;
    zeile(klappt2 ? CHK_OK : CHK_FAIL, "  nach Software-Reset",
          "TX=" + String(txR) + "/" + String(tx2) + " Boot " + String(tBoot) + " ms  " + bnoZustand(),
          "auch nach Reset kein Fusion-Modus — Versorgung/Takt des Breakouts pruefen");
    if (klappt2)
        hinweiszeile("nach Reset geht es — Chip hing in einem Fehlerzustand vom Einschalten");
}

static void stufe_bno(void)
{
    ueberschrift("6  Lagesensor BNO055 (0x28 @ Wire)");

    if (!i2cDa(Wire, ADR_BNO))
    {
        zeile(CHK_FAIL, "BNO055", "antwortet nicht",
              "Breakout sitzt auf SKT20/21 am ESP-Bus — Jumper JP1/JP2 und Stecker pruefen");
        return;
    }

    bool ok = false;
    const uint8_t chipId = i2cReg(Wire, ADR_BNO, REG_CHIP_ID, ok);
    if (!ok || chipId != CHK_BNO_CHIP_ID)
    {
        zeile(CHK_FAIL, "BNO055 Chip-ID", hex2(chipId) + " (erwartet " + hex2(CHK_BNO_CHIP_ID) + ")",
              "an 0x28 antwortet etwas anderes als ein BNO055");
        return;
    }
    zeile(CHK_OK, "BNO055 Chip-ID", hex2(chipId));

    const uint8_t accId = i2cReg(Wire, ADR_BNO, REG_ACC_ID, ok);
    const uint8_t magId = i2cReg(Wire, ADR_BNO, REG_MAG_ID, ok);
    const uint8_t gyrId = i2cReg(Wire, ADR_BNO, REG_GYR_ID, ok);
    const bool teileOk = (accId == CHK_BNO_ACC_ID) && (magId == CHK_BNO_MAG_ID) && (gyrId == CHK_BNO_GYR_ID);
    zeile(teileOk ? CHK_OK : CHK_WARN, "Teilsensoren ACC/MAG/GYR",
          hex2(accId) + " " + hex2(magId) + " " + hex2(gyrId),
          "ein Teilsensor meldet sich nicht — Chip ist beschaedigt");

    const uint8_t st = i2cReg(Wire, ADR_BNO, REG_ST_RESULT, ok);
    zeile((ok && (st & 0x0F) == 0x0F) ? CHK_OK : CHK_WARN, "Power-On-Selbsttest",
          "ST_RESULT=" + hex2(st) + " (Bit0 ACC, 1 MAG, 2 GYR, 3 MCU)",
          "nicht alle Bits gesetzt — ein Teilsensor hat den Selbsttest nicht bestanden");

    const e_bnoSysStatus sysSt = bno.systemStatus();
    const uint8_t sysErr = bno.systemError();
    const char *stTxt = "unbekannt";
    switch (sysSt)
    {
    case systemIdle:                          stTxt = "Idle"; break;
    case systemError:                         stTxt = "FEHLER"; break;
    case initializingPeripherals:             stTxt = "Peripherie startet"; break;
    case systemInitialization:                stTxt = "Initialisierung"; break;
    case executingSelftest:                   stTxt = "Selbsttest laeuft"; break;
    case sensorFusionAlgorithmRunning:        stTxt = "Fusion laeuft"; break;
    case systemRunningWithoutFusionAlgorithm: stTxt = "laeuft ohne Fusion"; break;
    }
    zeile(sysSt == sensorFusionAlgorithmRunning ? CHK_OK : CHK_WARN,
          "Systemstatus", String(stTxt) + " (SYS_ERR=" + hex2(sysErr) + ")",
          "nur 'Fusion laeuft' liefert ein brauchbares Heading — OPR_MODE pruefen");

    const uint8_t opr = i2cReg(Wire, ADR_BNO, REG_OPR_MODE, ok) & 0x0F;
    zeile((ok && opr == DEFAULT_OPR_MODE) ? CHK_OK : CHK_FAIL, "Betriebsmodus",
          "OPR_MODE=" + hex2(opr) + " (erwartet " + hex2(DEFAULT_OPR_MODE) + ")",
          opr == OPR_MODE_CONFIGMODE ? "haengt im CONFIGMODE — bno.init() hat nicht gegriffen"
                                     : "falscher Modus — bno.init() pruefen");

    if (opr != DEFAULT_OPR_MODE)
        bnoModusDiagnose();

    const uint8_t cal = bno.calibStatus();
    zeile(((cal >> 4) & 0x03) == 3 ? CHK_OK : CHK_WARN, "Kalibrierung",
          "Sys " + String((cal >> 6) & 0x03) + " / Gyro " + String((cal >> 4) & 0x03) +
          " / Acc " + String((cal >> 2) & 0x03) + " / Mag " + String(cal & 0x03) + "  (3 = fertig)",
          "Gyro nicht kalibriert: Roboter 5 s voellig still stehen lassen");

    const float h0 = bno.eulHeading();
    float hMin = h0, hMax = h0;
    for (uint8_t i = 0; i < 20; i++)
    {
        delay(20);
        const float h = bno.eulHeading();
        if (h < hMin) hMin = h;
        if (h > hMax) hMax = h;
    }
    const float drift = hMax - hMin;

    if (h0 == 0.0f && drift == 0.0f)
        zeile(CHK_FAIL, "Heading", "konstant 0.0 Grad",
              "Euler-Register leer — Fusion laeuft nicht (Betriebsmodus/Systemstatus oben)");
    else if (drift > CHK_BNO_DRIFT_MAX)
        zeile(CHK_WARN, "Heading", String(h0, 1) + " Grad, wandert " + String(drift, 1) + " Grad",
              "unruhig: steht der Roboter still? Sonst Gyro kalibrieren / Magnetfeld pruefen");
    else
        zeile(CHK_OK, "Heading", String(h0, 1) + " Grad, Rauschen " + String(drift, 2) + " Grad");

    hinweiszeile("Gegentest: Roboter 90 Grad drehen, Heading muss ~90 Grad wandern.");
}

/* ============================================================
 *  7  Radar  (XM125 am I2C-INTERCON, also am ESP-Bus = Wire)
 * ============================================================
 *  Alle vier messen gleichzeitig; update() holt nur ab. Darum erst einige
 *  Runden pumpen, bevor bewertet wird. Dist ist in mm:
 *      cm = Dist / 10 + POS_RADAR_OFFSET_CM
 */

/// Mindest-Signalstaerke in mdB, gleiche Kennlinie wie in systemlib.
static int32_t chkMinStrengthMdb(float distCm)
{
    const float minDb = -11000.0f - ((distCm / 100.0f) * 2500.0f);
    return (int32_t)max(minDb, -17000.0f);
}

static float radarCm(uint16_t distMm) { return (distMm / 10.0f) + POS_RADAR_OFFSET_CM; }

static void radarWarmlaufen(uint32_t dauerMs)
{
    elapsedMillis t;
    while (t < dauerMs)
    {
        radar.update();
        delay(5);
    }
}

static void stufe_radar(void)
{
    ueberschrift("7  Radar XM125 (4x, am I2C-INTERCON)");

    CHK_OUT.print("       i  Messrunden sammeln (");
    CHK_OUT.print(CHK_RADAR_WARMUP_MS / 1000);
    CHK_OUT.println(" s) ...");
    radarWarmlaufen(CHK_RADAR_WARMUP_MS);

    static const uint8_t pos[4]     = { RadarVo, RadarRe, RadarHi, RadarLi };
    static const char   *posName[4] = { "Radar vorne", "Radar rechts", "Radar hinten", "Radar links" };

    uint8_t lebendig = 0;

    for (uint8_t k = 0; k < 4; k++)
    {
        const uint8_t p = pos[k];
        const RadarData d = radar[p];
        const uint32_t alter = radar.giveAge(p);

        if (!d.isCon)
        {
            zeile(CHK_FAIL, posName[k], "antwortet nicht (Adresse " + hex2(RADAR_SENSOR_ADDRS[p]) + ")",
                  "Sensor fehlt oder Kalibrierung schlug fehl — RadarArray startet ihn alle 500 ms neu");
            continue;
        }
        lebendig++;

        const float cm = radarCm(d.Dist);
        const String wert = String(cm, 1) + " cm  Peaks=" + String(d.Peaks) +
                            "  Staerke=" + String(d.Strenght) + " mdB  Alter=" + String(alter) + " ms";

        if (alter > CHK_RADAR_AGE_WARN)
            zeile(CHK_WARN, posName[k], wert,
                  "Messwert zu alt — Certainty faellt auf 0, Sensor liefert nicht mehr");
        else if (d.Peaks == 0)
            zeile(CHK_WARN, posName[k], wert,
                  "keine Wand erkannt: Sichtfeld frei? Abstand im Fenster 15..270 cm?");
        else if (d.Strenght < chkMinStrengthMdb(cm))
            zeile(CHK_WARN, posName[k], wert,
                  "Echo zu schwach fuer diese Entfernung — Ausrichtung/Empfindlichkeit pruefen");
        else if (cm < (RADAR_START_MM / 10.0f) + POS_RADAR_OFFSET_CM)
            zeile(CHK_WARN, posName[k], wert, "Ziel im Blindbereich (< RADAR_START_MM)");
        else
            zeile(CHK_OK, posName[k], wert);

        XM125Distance &s = radar.giveSensor(p);
        if (s.getCalibrationNeeded())
            zeile(CHK_WARN, "  Kalibrierung", "Sensor fordert Neukalibrierung an",
                  "radar.calib() aufrufen (blockiert ~1 s je Sensor)");
        if (s.getMeasureError())
            zeile(CHK_WARN, "  Messfehler", "Sensor meldet measureError",
                  "Detector-Status pruefen, ggf. Modul neu kalibrieren");
        zeile(CHK_INFO, "  Temperatur", String(s.getTemperature()) + " C");
    }

    zeile(lebendig == RADAR_NUM_SENSORS ? CHK_OK : (lebendig >= 2 ? CHK_WARN : CHK_FAIL),
          "Radar gesamt", String(lebendig) + " von " + String(RADAR_NUM_SENSORS) + " Sensoren",
          "mit weniger als 4 Sensoren faellt mindestens eine Achse auf Odometrie zurueck");

    /* ---- Gegenprobe: gegenueberliegende Sensoren messen dieselbe Achse ---- */
    struct Achse { uint8_t a, b; const char *name; float feld; };
    static const Achse achsen[2] = {
        { RadarLi, RadarRe, "Achse X (links+rechts)", POS_FIELD_X_CM },
        { RadarVo, RadarHi, "Achse Y (vorne+hinten)", POS_FIELD_Y_CM },
    };

    for (uint8_t i = 0; i < 2; i++)
    {
        const Achse &ac = achsen[i];
        const RadarData da = radar[ac.a];
        const RadarData db = radar[ac.b];

        if (!da.isCon || !db.isCon || da.Peaks == 0 || db.Peaks == 0)
        {
            zeile(CHK_SKIP, ac.name, "nicht pruefbar (ein Sensor liefert nichts)");
            continue;
        }

        const float summe = radarCm(da.Dist) + radarCm(db.Dist);
        const float abw   = fabsf(summe - ac.feld);

        zeile(abw <= POS_PAIR_TOL_CM ? CHK_OK : CHK_WARN, ac.name,
              "Summe " + String(summe, 1) + " cm  (Feld " + String(ac.feld, 0) +
              " cm, Abweichung " + String(abw, 1) + " cm)",
              "Paar widerspricht sich: Hindernis, Tornische oder Roboter steht schraeg");
    }
}

/* ============================================================
 *  8  IR-Ring  (STM32-Platine am I2C-INTERCON, also am ESP-Bus)
 * ============================================================ */

static void stufe_irring(void)
{
    ueberschrift("8  IR-Ring (Ballerkennung, am I2C-INTERCON)");

    if (!i2cDa(Wire, ADR_IRRING))
    {
        zeile(CHK_FAIL, "IR-Ring 0x0A", "antwortet nicht",
              "Intercon-Stecker, 5V und die STM32-Firmware der IR-Platine pruefen");
        return;
    }

    const uint8_t id = irRing.read_sensor_id();
    zeile(id != 0x00 && id != 0xFF ? CHK_OK : CHK_WARN, "Sensor-ID", hex2(id),
          "0x00/0xFF heisst meist: Chip antwortet, liefert aber keine Daten");

    const IR_ring::Fw_version fw = irRing.read_fw_version();
    zeile(CHK_INFO, "Firmware", String(fw.major) + "." + String(fw.minor) + "." + String(fw.patch));

    const IR_ring::Hw_version hw = irRing.read_hw_version();
    const char *hwTxt = "unbekannt";
    if (hw == IR_ring::Hw_version::v1_0) hwTxt = "v1.0";
    else if (hw == IR_ring::Hw_version::v1_1) hwTxt = "v1.1";
    zeile(hw == IR_ring::Hw_version::unknown ? CHK_WARN : CHK_OK, "Hardware-Revision", hwTxt,
          "Platine meldet keine Revision");

    const uint16_t err = irRing.read_error_code();
    zeile(err == 0 ? CHK_OK : CHK_FAIL, "Fehlercode", "0x" + String(err, HEX),
          "die IR-Ring-Platine meldet selbst einen Fehler");

    /* ---- 16 IR-Rohwerte ---- */
    uint16_t roh[16];
    irRing.read_raw_values(roh);

    uint16_t rMin = 0xFFFF, rMax = 0;
    uint8_t  nullen = 0;
    for (uint8_t i = 0; i < 16; i++)
    {
        if (roh[i] < rMin) rMin = roh[i];
        if (roh[i] > rMax) rMax = roh[i];
        if (roh[i] == 0) nullen++;
    }

    if (nullen == 16)
        zeile(CHK_FAIL, "IR-Sensoren (16x)", "alle Rohwerte 0",
              "IR-Versorgung aus (write_sensor_power_supply) oder Sensoren defekt");
    else if (rMin == rMax)
        zeile(CHK_WARN, "IR-Sensoren (16x)", "alle Werte identisch (" + String(rMin) + ")",
              "unplausibel — Leseweg der IR-Platine pruefen");
    else
        zeile(nullen == 0 ? CHK_OK : CHK_WARN, "IR-Sensoren (16x)",
              String(rMin) + ".." + String(rMax) + ", " + String(nullen) + " auf 0",
              "einzelne Sensoren liefern 0 — Bestueckung/Maske (ir_sensor_mask) pruefen");

    String zeileRoh;
    for (uint8_t i = 0; i < 16; i++) { zeileRoh += String(roh[i]); zeileRoh += ' '; }
    hinweiszeile(zeileRoh.c_str());

    /* ---- TSSP4038, 8 Stueck ---- */
    bool tssp[8];
    irRing.read_tssp(tssp);
    String tsppTxt;
    uint8_t aktiv = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        tsppTxt += tssp[i] ? '1' : '0';
        if (tssp[i]) aktiv++;
    }
    zeile(CHK_INFO, "TSSP4038 (8x)", tsppTxt + "  (" + String(aktiv) + " aktiv)");

    if (irRing.read_ball_visible())
        zeile(CHK_OK, "Ball", "erkannt bei " + String(irRing.read_ball_angle(), 1) +
                              " Grad, " + String(irRing.read_ball_distance()) + " (roh)");
    else
        zeile(CHK_SKIP, "Ball", "kein Ball im Sichtfeld");

    hinweiszeile("Gegentest: Ball vor den Roboter legen, Winkel muss ~0 Grad werden.");
}

/* ============================================================
 *  9  Linien-ADCs  (Bottom-Platine, 6 x ADC108S102 ueber 74HC138)
 * ============================================================
 *  Zwei verschiedene Fehlerbilder:
 *    - Chip/CS tot     -> alle 8 Kanaele gleich und am Anschlag (0 oder 3300 mV)
 *    - Decoder falsch  -> zwei Ports liefern Bit fuer Bit dieselben Werte
 *  Darum wird beides geprueft: je Chip die Kanaele, danach die Ports gegeneinander.
 */

static void stufe_lineADC(void)
{
    ueberschrift("9  Liniensensoren (6x ADC108S102 A..F, 48 Kanaele)");

    /* ---- Vorbedingung: ohne LINE_EN sind die LEDs dunkel ---- */
    bool okEn = false;
    const uint8_t en = i2cReg(Wire1, ADR_IC8, 0x01, okEn);
    if (!okEn)
        zeile(CHK_WARN, "LINE_EN (IC8 P3)", "IC8 nicht lesbar",
              "ohne LINE_EN haben die Linien-LEDs keine Stromversorgung");
    else
        zeile(((en >> EN_LINE) & 1) ? CHK_OK : CHK_FAIL, "LINE_EN (IC8 P3)",
              ((en >> EN_LINE) & 1) ? "ein" : "AUS",
              "Linien-LEDs ohne Strom — alle Kanaele messen Fremdlicht (Bottom Blatt 1)");

    static const char *portName[6] = { "Line-ADC A", "Line-ADC B", "Line-ADC C",
                                       "Line-ADC D", "Line-ADC E", "Line-ADC F" };
    int16_t mittel[6][8];
    uint8_t ueberSchwelle = 0;

    for (uint8_t p = 0; p < 6; p++)
    {
        int16_t pMin = 32767, pMax = -32768;
        uint16_t rauschMax = 0;
        uint8_t  gleich = 0, tot = 0;

        for (uint8_t k = 0; k < 8; k++)
        {
            const AdcStat s = adcMessen(*adcLine[p], k);
            mittel[p][k] = s.mittel;

            if (s.mittel < pMin) pMin = s.mittel;
            if (s.mittel > pMax) pMax = s.mittel;

            const uint16_t spanne = (uint16_t)(s.max - s.min);
            if (spanne > rauschMax) rauschMax = spanne;

            // Am Anschlag festgenagelt und voellig rauschfrei = kein lebender Kanal
            if (spanne == 0 && (s.mittel <= CHK_ADC_LOW_DEAD || s.mittel >= CHK_ADC_HIGH_DEAD)) tot++;
            if (k > 0 && s.mittel == mittel[p][0]) gleich++;

            // s.mittel kann negativ werden, wenn MISO Muell liefert (read()
            // rechnet mit int16_t) — ohne diese Pruefung wuerde der Cast
            // daraus einen riesigen Wert machen.
            if (s.mittel > 0 && (uint16_t)s.mittel > CHK_LINE_SCHWELLE) ueberSchwelle++;
        }

        const String wert = "Kanal 0-7: " + String(pMin) + ".." + String(pMax) +
                            " mV, Rauschen max " + String(rauschMax) + " mV";

        if (tot == 8)
            zeile(CHK_FAIL, portName[p], wert,
                  "Chip antwortet nicht: SPI (11/12/13), Decoder 31/32/33 oder 3V3-A pruefen");
        else if (gleich == 7)
            zeile(CHK_WARN, portName[p], wert,
                  "alle 8 Kanaele identisch — Chip wird nicht richtig adressiert");
        else if (tot > 0)
            zeile(CHK_WARN, portName[p], wert + ", " + String(tot) + " Kanal/Kanaele am Anschlag",
                  "einzelne Sensoren offen oder kurzgeschlossen");
        else
            zeile(CHK_OK, portName[p], wert);
    }

    /* ---- Decoder-Gegenprobe ---- */
    // Zwei verschiedene Chips liefern nie Bit fuer Bit dieselben 8 Werte.
    // Passiert es doch, zeigt der 74HC138 bei beiden Adressen auf denselben Chip.
    uint8_t dubletten = 0;
    for (uint8_t a = 0; a < 6; a++)
    {
        for (uint8_t b = (uint8_t)(a + 1); b < 6; b++)
        {
            bool identisch = true;
            for (uint8_t k = 0; k < 8; k++)
                if (mittel[a][k] != mittel[b][k]) { identisch = false; break; }

            if (identisch)
            {
                dubletten++;
                zeile(CHK_WARN, "Decoder-Adressierung",
                      String("Adresse ") + char('A' + a) + " und " + char('A' + b) + " liefern dieselben Werte",
                      "74HC138 waehlt beide Male denselben Chip — Pins 31/32/33 pruefen");
            }
        }
    }
    if (dubletten == 0)
        zeile(CHK_OK, "Decoder-Adressierung", "alle 6 Adressen liefern eigene Werte");

    zeile(CHK_INFO, "Linie erkannt", String(ueberSchwelle) + " von 48 Kanaelen ueber " +
                                     String(CHK_LINE_SCHWELLE) + " mV");
    if (ueberSchwelle > 24)
        zeile(CHK_WARN, "Linienschwelle", String(ueberSchwelle) + " Kanaele aktiv",
              "mehr als die Haelfte sieht 'Linie' — Schwelle passt nicht zum Untergrund");

    zeile(CHK_SKIP, "Maussensor (Adresse 6)", "Steckplatz am Decoder, hier nicht geprueft");
    hinweiszeile("Gegentest: Roboter ueber die weisse Linie schieben, die Zahl muss steigen.");
}

/* ============================================================
 * 10  Motor-ADC  (Main Blatt 5, IC10 an SPI, CS = Pin 30)
 * ============================================================
 *  IN0..IN7 = MO1_AO1, MO1_AO2, MO2_AO1, ... MO4_AO2.
 *  Beide Analogausgaenge sind bipolar und liegen im Stand auf der Mitte
 *  (1650 mV). Anschlag unten/oben heisst: ESCON ohne Versorgung oder
 *  Leitung offen. Welcher Ausgang Drehzahl und welcher Strom ist, ermittelt
 *  erst Stufe 12.
 */

/// Ein ESCON-Modul, wie es auf den Plaenen heisst (MO1..MO4).
struct MotorHW
{
    const char *name;
    uint8_t     pwmPin;    ///< Main Blatt 1
    TCA9534DW  *exp;       ///< IC9 fuer MO1/MO2, IC7 fuer MO3/MO4
    uint8_t     expAddr;
    uint8_t     pBase;     ///< erster der drei Expanderpins (DO1, DIO3, DIO4)
    uint8_t     chAO1;     ///< ADC-Kanal
    uint8_t     chAO2;
};

static const MotorHW motoren[4] = {
    { "MO1", Pin::MO_SPEED[0], &expMO12, ADR_IC9, 0, 0, 1 },
    { "MO2", Pin::MO_SPEED[1], &expMO12, ADR_IC9, 3, 2, 3 },
    { "MO3", Pin::MO_SPEED[2], &expMO34, ADR_IC7, 0, 4, 5 },
    { "MO4", Pin::MO_SPEED[3], &expMO34, ADR_IC7, 3, 6, 7 },
};

static void stufe_motorADC(void)
{
    ueberschrift("10  Motor-ADC / ESCON-Rueckmeldung (Ruhezustand)");

    uint8_t plausibel = 0;

    for (uint8_t i = 0; i < 4; i++)
    {
        const MotorHW &m = motoren[i];
        const AdcStat a1 = adcMessen(adcMotor, m.chAO1);
        const AdcStat a2 = adcMessen(adcMotor, m.chAO2);

        const String wert = "AO1 " + String(a1.mittel) + " mV (IN" + String(m.chAO1) +
                            "), AO2 " + String(a2.mittel) + " mV (IN" + String(m.chAO2) + ")";

        const int16_t d1 = (int16_t)abs(a1.mittel - CHK_MOT_MID_MV);
        const int16_t d2 = (int16_t)abs(a2.mittel - CHK_MOT_MID_MV);

        if (a1.mittel <= CHK_ADC_LOW_DEAD && a2.mittel <= CHK_ADC_LOW_DEAD)
            zeile(CHK_FAIL, m.name, wert,
                  "beide Ausgaenge auf 0 V: ESCON ohne Versorgung oder Stecker ab");
        else if (a1.mittel >= CHK_ADC_HIGH_DEAD || a2.mittel >= CHK_ADC_HIGH_DEAD)
            zeile(CHK_WARN, m.name, wert,
                  "Ausgang am oberen Anschlag — Leitung offen oder ESCON meldet Fehler");
        else if (d1 > CHK_MOT_MID_TOL && d2 > CHK_MOT_MID_TOL)
            zeile(CHK_WARN, m.name, wert,
                  "beide Ausgaenge abseits der Mitte (1650 mV): dreht das Rad? Sonst ESCON-Konfiguration");
        else
        {
            zeile(CHK_OK, m.name, wert);
            plausibel++;
        }

        zeile(CHK_INFO, "  Rauschen", "AO1 " + String(a1.max - a1.min) +
                                      " mV, AO2 " + String(a2.max - a2.min) + " mV");

        // Die Steuerleitungen liegen auf dem Portexpander — hier nur anzeigen.
        bool ok = false;
        const uint8_t out = i2cReg(Wire1, m.expAddr, 0x01, ok);
        if (ok)
            zeile(CHK_INFO, "  Steuerleitungen",
                  String("Enable=") + String((out >> (m.pBase + ESCON_P_ENABLE)) & 1) +
                  " Richtung=" + String((out >> (m.pBase + ESCON_P_DIR)) & 1) +
                  " DIO4=" + String((out >> (m.pBase + ESCON_P_DIO4)) & 1) +
                  "  (" + hex2(m.expAddr) + " P" + String(m.pBase) + "..)");
    }

    zeile(plausibel == 4 ? CHK_OK : (plausibel > 0 ? CHK_WARN : CHK_FAIL),
          "Motor-ADC gesamt", String(plausibel) + " von 4 Modulen plausibel",
          "ohne Drehzahlrueckmeldung gibt es keine Odometrie");

    zeile(CHK_SKIP, "Dribbler (BLDC)", "AO1/AO2 liegen auf Teensy 27/26, nicht am Motor-ADC");
    hinweiszeile("Welcher ESCON-Ausgang Drehzahl und welcher Strom ist, klaert Stufe 12 ('m').");
}

/* ============================================================
 * 11  Schalter und Taster  (SwitchBoard am Intercon)
 * ============================================================ */

static const char *sbSwName[8]  = { "SW5 (P0)", "SW4 (P1)", "SW3 (P2)", "SW2 (P3)",
                                    "SW1 (P4)", "Startschalter (P5)", "RCJ-Disable (P6)", "RCJ-erkannt (P7)" };
static const char *sbBtnName[8] = { "BTN1 (P0)", "BTN2 (P1)", "BTN3 (P2)", "BTN4 (P3)",
                                    "BTN5 (P4)", "RCJ-Out1 (P5)", "RCJ-Out2 (P6)", "PX2_DIO (P7)" };

static void stufe_schalter(void)
{
    ueberschrift("11  Schalter, Taster, RCJ-Modul (SwitchBoard)");

    if (!sbSwAddr)
    {
        zeile(CHK_FAIL, "SwitchBoard IC1", "nicht gefunden",
              "ohne SwitchBoard melden alle Schalter 'high' — der Roboter startet nicht");
        return;
    }

    bool okSw = false, okBt = false;
    const uint8_t sw = i2cReg(Wire, sbSwAddr, 0x00, okSw);
    const uint8_t bt = sbBtnAddr ? i2cReg(Wire, sbBtnAddr, 0x00, okBt) : 0;

    if (!okSw)
    {
        zeile(CHK_FAIL, "SwitchBoard IC1", hex2(sbSwAddr) + " nicht lesbar");
        return;
    }

    zeile(CHK_INFO, "IC1 Portbyte", bitKette(sw) + " (" + hex2(sw) + " @ " + hex2(sbSwAddr) + ")");
    for (uint8_t b = 0; b < 8; b++)
        zeile(CHK_INFO, sbSwName[b], ((sw >> b) & 1) ? "HIGH" : "LOW");

    if (!sbBtnAddr || !okBt)
    {
        zeile(CHK_FAIL, "SwitchBoard IC2", "nicht lesbar", "Taster und RCJ-Signal fehlen");
    }
    else
    {
        zeile(CHK_INFO, "IC2 Portbyte", bitKette(bt) + " (" + hex2(bt) + " @ " + hex2(sbBtnAddr) + ")");
        for (uint8_t b = 0; b < 8; b++)
            zeile(CHK_INFO, sbBtnName[b], ((bt >> b) & 1) ? "HIGH" : "LOW");
    }

    /* ---- RCJ-Modul: Startentscheidung ---- */
    const bool rcjDet     = (sw >> 7) & 1;
    const bool rcjDisable = (sw >> 6) & 1;
    const bool startSw    = (sw >> 5) & 1;
    const bool rcjOut1    = okBt ? (bool)((bt >> 5) & 1) : false;

    if (rcjDet)
        zeile(CHK_OK, "RCJ-Modul", String("gesteckt, Out1 ") + (rcjOut1 ? "START" : "Stop") +
                                   ", Disable " + (rcjDisable ? "ein" : "aus"));
    else
        zeile(CHK_SKIP, "RCJ-Modul", "nicht gesteckt — es gilt der Startschalter");

    zeile(CHK_INFO, "Startentscheidung", rcjDet ? ((rcjOut1 && !rcjDisable) ? "START" : "Stop")
                                                : (startSw ? "START" : "Stop"));

    /* ---- Lichtschranke (LDR auf Teensy 23, LED ueber IC8 P2) ---- */
    bool okEn = false;
    const uint8_t en = i2cReg(Wire1, ADR_IC8, 0x01, okEn);
    const bool barriereAn = okEn && ((en >> EN_BARRIER) & 1);
    zeile(barriereAn ? CHK_INFO : CHK_SKIP, "Lichtschranke (Pin 23)",
          String(digitalRead(Pin::LDR_IN) ? "HIGH" : "LOW") +
          (barriereAn ? " (LED ein)" : " (LED aus — Wert ohne Bedeutung)"));

    hinweiszeile("Nur eine Momentaufnahme. Jeden Kontakt einzeln pruefen: Taste 't'.");
}

/* ------------------------------------------------------------
 *  11b  Kontaktdurchlauf
 * ------------------------------------------------------------
 *  Eine Momentaufnahme sagt nur, wie die Schalter gerade stehen — nicht, ob
 *  jeder Kontakt funktioniert. Darum hier mitschreiben, welche Bits sowohl 0
 *  als auch 1 waren. Was nie gewechselt hat, ist nicht bewegt worden oder tot.
 */

static void stufe_kontakte(void)
{
    ueberschrift("11b  Kontaktdurchlauf (jeden Schalter/Taster einmal bewegen)");

    if (!sbSwAddr)
    {
        zeile(CHK_FAIL, "Kontaktdurchlauf", "SwitchBoard nicht gefunden");
        return;
    }

    CHK_OUT.println("       !  Jetzt jeden Schalter umlegen und jeden Taster druecken.");
    CHK_OUT.println("       !  Beliebige Taste beendet den Durchlauf.");
    while (CHK_OUT.available()) CHK_OUT.read();

    uint8_t swNull = 0, swEins = 0, btNull = 0, btEins = 0;
    uint8_t sw = 0, bt = 0;
    elapsedMillis anzeige;

    while (!CHK_OUT.available())
    {
        bool ok = false;
        sw = i2cReg(Wire, sbSwAddr, 0x00, ok);
        if (ok) { swEins |= sw; swNull |= (uint8_t)~sw; }

        if (sbBtnAddr)
        {
            ok = false;
            bt = i2cReg(Wire, sbBtnAddr, 0x00, ok);
            if (ok) { btEins |= bt; btNull |= (uint8_t)~bt; }
        }

        if (anzeige >= 200)
        {
            anzeige = 0;
            CHK_OUT.print("\r  IC1 ");
            CHK_OUT.print(bitKette(sw));
            CHK_OUT.print("   IC2 ");
            CHK_OUT.print(bitKette(bt));
            CHK_OUT.print("   beide Zustaende: IC1 ");
            CHK_OUT.print(bitKette((uint8_t)(swNull & swEins)));
            CHK_OUT.print(" IC2 ");
            CHK_OUT.print(bitKette((uint8_t)(btNull & btEins)));
            CHK_OUT.print("   ");
        }
        delay(5);
    }
    CHK_OUT.read();
    CHK_OUT.println();

    const uint8_t swBeide = (uint8_t)(swNull & swEins);
    const uint8_t btBeide = (uint8_t)(btNull & btEins);

    for (uint8_t b = 0; b < 8; b++)
        zeile(((swBeide >> b) & 1) ? CHK_OK : CHK_SKIP, sbSwName[b],
              ((swBeide >> b) & 1) ? "schaltet" : "nicht bewegt oder tot");

    for (uint8_t b = 0; b < 8; b++)
        zeile(((btBeide >> b) & 1) ? CHK_OK : CHK_SKIP, sbBtnName[b],
              ((btBeide >> b) & 1) ? "schaltet" : "nicht bewegt oder tot");

    hinweiszeile("[ -- ] heisst nur: dieses Bit hat nicht gewechselt. RCJ-erkannt wechselt nur beim Stecken.");
}

/* ============================================================
 * 12  Aktortest (opt-in)
 * ============================================================
 *  Der einzige Teil, der etwas bewegt und der Einzige, der VMOT freigibt.
 *  Je Modul: Ruhewerte merken, vorwaerts anfahren, rueckwaerts anfahren,
 *  stoppen. Daraus folgt zweierlei:
 *    - reagiert die Kette PWM -> ESCON -> Motor -> AO -> ADC ueberhaupt?
 *    - welcher der beiden Analogausgaenge ist Drehzahl (wechselt mit der
 *      Richtung das Vorzeichen) und welcher Strom (steigt in beide Richtungen)?
 */

static void motorStill(const MotorHW &m)
{
    analogWrite(m.pwmPin, 0);
    m.exp->writePin(m.pBase + ESCON_P_ENABLE, false);
    m.exp->writePin(m.pBase + ESCON_P_DIR, false);
}

static void alleMotorenStill(void)
{
    for (uint8_t i = 0; i < 4; i++) motorStill(motoren[i]);
}

static void stufe_aktor(void)
{
    ueberschrift("12  Aktortest — die Raeder drehen sich!");

    alleMotorenStill();

    CHK_OUT.println("       !  Roboter AUFBOCKEN. Der Check gibt VMOT frei.");
    CHK_OUT.println("       !  Start mit 'j', alles andere bricht ab.");
    while (!CHK_OUT.available()) { delay(10); }
    if (CHK_OUT.read() != 'j')
    {
        zeile(CHK_SKIP, "Aktortest", "abgebrochen");
        return;
    }

    /* ---- VMOT freigeben (zusaetzlich haengt Schalter SW3 in Reihe) ---- */
    expEN.writePin(EN_VMOT, true);
    delay(300);

    const float vmot = inaMotor.getBusVoltage();
    if (vmot < CHK_VMOT_V_MIN)
    {
        zeile(CHK_FAIL, "Aktortest", "VMOT bleibt bei " + String(vmot, 2) + " V",
              "Schalter SW3 (MOT) steht auf aus — ohne Motorspannung dreht nichts");
        expEN.writePin(EN_VMOT, false);
        return;
    }
    zeile(CHK_OK, "VMOT freigegeben", String(vmot, 2) + " V");

    for (uint8_t i = 0; i < 4; i++)
    {
        const MotorHW &m = motoren[i];

        const AdcStat ruhe1 = adcMessen(adcMotor, m.chAO1);
        const AdcStat ruhe2 = adcMessen(adcMotor, m.chAO2);

        /* ---- vorwaerts ---- */
        m.exp->writePin(m.pBase + ESCON_P_DIR, false);
        m.exp->writePin(m.pBase + ESCON_P_ENABLE, true);
        analogWrite(m.pwmPin, CHK_AKTOR_PWM);
        delay(CHK_AKTOR_RAMP_MS);
        const AdcStat vor1 = adcMessen(adcMotor, m.chAO1);
        const AdcStat vor2 = adcMessen(adcMotor, m.chAO2);

        /* ---- rueckwaerts ---- */
        analogWrite(m.pwmPin, 0);
        delay(CHK_AKTOR_STOP_MS);
        m.exp->writePin(m.pBase + ESCON_P_DIR, true);
        analogWrite(m.pwmPin, CHK_AKTOR_PWM);
        delay(CHK_AKTOR_RAMP_MS);
        const AdcStat rueck1 = adcMessen(adcMotor, m.chAO1);
        const AdcStat rueck2 = adcMessen(adcMotor, m.chAO2);

        motorStill(m);
        delay(CHK_AKTOR_STOP_MS);

        const int16_t dVor1   = (int16_t)(vor1.mittel   - ruhe1.mittel);
        const int16_t dVor2   = (int16_t)(vor2.mittel   - ruhe2.mittel);
        const int16_t dRueck1 = (int16_t)(rueck1.mittel - ruhe1.mittel);
        const int16_t dRueck2 = (int16_t)(rueck2.mittel - ruhe2.mittel);

        const String wert = "AO1 " + String(ruhe1.mittel) + "/" + String(vor1.mittel) + "/" + String(rueck1.mittel) +
                            "  AO2 " + String(ruhe2.mittel) + "/" + String(vor2.mittel) + "/" + String(rueck2.mittel) +
                            " mV (Ruhe/vor/rueck)";

        const bool reagiert1 = abs(dVor1) >= CHK_MOT_RUN_DIFF || abs(dRueck1) >= CHK_MOT_RUN_DIFF;
        const bool reagiert2 = abs(dVor2) >= CHK_MOT_RUN_DIFF || abs(dRueck2) >= CHK_MOT_RUN_DIFF;

        if (!reagiert1 && !reagiert2)
        {
            zeile(CHK_FAIL, m.name, wert,
                  "kein Ausgang reagiert: PWM-Pin, Enable auf dem Expander, ESCON oder Motor");
            continue;
        }

        zeile(CHK_OK, m.name, wert);

        /* ---- Welcher Ausgang ist Drehzahl, welcher Strom? ---- */
        // Drehzahl: wechselt mit der Richtung das Vorzeichen.
        // Strom: steigt in beide Richtungen in dieselbe Richtung.
        const bool dreh1 = reagiert1 && ((dVor1 > 0) != (dRueck1 > 0));
        const bool dreh2 = reagiert2 && ((dVor2 > 0) != (dRueck2 > 0));

        if (dreh1 && !dreh2)
            zeile(CHK_OK, "  Belegung", "AO1 = Drehzahl (IN" + String(m.chAO1) +
                                        "), AO2 = Strom (IN" + String(m.chAO2) + ")");
        else if (dreh2 && !dreh1)
            zeile(CHK_OK, "  Belegung", "AO2 = Drehzahl (IN" + String(m.chAO2) +
                                        "), AO1 = Strom (IN" + String(m.chAO1) + ")");
        else if (dreh1 && dreh2)
            zeile(CHK_WARN, "  Belegung", "beide Ausgaenge drehrichtungsabhaengig",
                  "ESCON gibt zweimal die Drehzahl aus — Konfiguration pruefen");
        else
            zeile(CHK_WARN, "  Belegung", "kein Ausgang wechselt mit der Richtung",
                  "Richtungsleitung DI2 wirkt nicht, oder beide Ausgaenge zeigen den Strom");
    }

    alleMotorenStill();
    expEN.writePin(EN_VMOT, false);
    zeile(CHK_OK, "Aktortest beendet", "alle Module aus, VMOT wieder gesperrt");
}

/* ============================================================
 *  Dauerueberwachung
 * ============================================================ */

static void liveMonitor(void)
{
    ueberschrift("Dauerueberwachung — beliebige Taste beendet");
    while (CHK_OUT.available()) CHK_OUT.read();

    elapsedMillis ausgabe;
    while (!CHK_OUT.available())
    {
        radar.update();

        if (ausgabe >= 250)
        {
            ausgabe = 0;

            bool okSw = false, okBt = false;
            const uint8_t sw = sbSwAddr  ? i2cReg(Wire, sbSwAddr,  0x00, okSw) : 0;
            const uint8_t bt = sbBtnAddr ? i2cReg(Wire, sbBtnAddr, 0x00, okBt) : 0;

            CHK_OUT.print("Akku ");        CHK_OUT.print(inaAkku.getBusVoltage(), 2);
            CHK_OUT.print(" V | VMOT ");   CHK_OUT.print(inaMotor.getBusVoltage(), 2);
            CHK_OUT.print(" V | Hd ");     CHK_OUT.print(bno.eulHeading(), 0);
            CHK_OUT.print(" | Radar ");
            CHK_OUT.print(radarCm(radar[RadarVo].Dist), 0); CHK_OUT.print('/');
            CHK_OUT.print(radarCm(radar[RadarRe].Dist), 0); CHK_OUT.print('/');
            CHK_OUT.print(radarCm(radar[RadarHi].Dist), 0); CHK_OUT.print('/');
            CHK_OUT.print(radarCm(radar[RadarLi].Dist), 0);
            CHK_OUT.print(" cm | IC1 ");   CHK_OUT.print(okSw ? bitKette(sw) : String("--------"));
            CHK_OUT.print(" IC2 ");        CHK_OUT.print(okBt ? bitKette(bt) : String("--------"));
            CHK_OUT.print(" | Ball ");     CHK_OUT.print(irRing.read_ball_visible() ? "ja " : "-- ");
            CHK_OUT.print(irRing.read_ball_angle(), 0);
            CHK_OUT.println(" Grad");
        }
        delay(5);
    }
    CHK_OUT.read();
}

/* ============================================================
 *  Bericht
 * ============================================================ */

static void zaehlerNull(void)
{
    cntOk = cntWarn = cntFail = cntSkip = 0;
    merkAnzahl = 0;
}

static void kopf(void)
{
    CHK_OUT.println();
    CHK_OUT.println("================================================================");
    CHK_OUT.println("  KOALAS 2027  —  SYSTEM-CHECK   (Sollwerte aus pcbs/)");
    CHK_OUT.println("================================================================");
}

static void zusammenfassung(void)
{
    ueberschrift("Zusammenfassung");

    CHK_OUT.print("  OK ");    CHK_OUT.print(cntOk);
    CHK_OUT.print("   WARN "); CHK_OUT.print(cntWarn);
    CHK_OUT.print("   FAIL "); CHK_OUT.print(cntFail);
    CHK_OUT.print("   nicht geprueft "); CHK_OUT.println(cntSkip);

    if (merkAnzahl == 0)
    {
        CHK_OUT.println();
        CHK_OUT.println("  Keine Auffaelligkeiten.");
    }
    else
    {
        CHK_OUT.println();
        CHK_OUT.println("  Nachsehen bei:");
        for (uint8_t i = 0; i < merkAnzahl; i++)
        {
            CHK_OUT.print("   - ");
            CHK_OUT.println(merkListe[i]);
        }
        if (merkAnzahl >= CHK_MAX_MERK)
            CHK_OUT.println("   ... (Liste voll, weitere Befunde stehen oben im Bericht)");
    }

    CHK_OUT.println();
    CHK_OUT.println("  Taste 'h' zeigt das Menue.");
    CHK_OUT.println("================================================================");
}

static void allesPruefen(void)
{
    zaehlerNull();
    kopf();

    stufe_teensy();
    sbAdressenSuchen();            // vor dem Scan, damit die Jumperadressen stimmen
    stufe_i2c(Wire,  0, wireGefunden);
    stufe_i2c(Wire1, 1, wire1Gefunden);
    stufe_power();
    stufe_expander();
    stufe_bno();
    stufe_radar();
    stufe_irring();
    stufe_lineADC();
    stufe_motorADC();
    stufe_schalter();

    zusammenfassung();
}

static void hilfe(void)
{
    CHK_OUT.println();
    CHK_OUT.println("  a  alles pruefen (Stufe 1-11, passiv)");
    CHK_OUT.println("  1  Teensy / Firmware");
    CHK_OUT.println("  2  I2C \"ESP\" = Wire  (BNO, IR-Ring, SwitchBoard, Radar)");
    CHK_OUT.println("  3  I2C \"TNS\" = Wire1 (Portexpander, INA219)");
    CHK_OUT.println("  4  Stromversorgung");
    CHK_OUT.println("  5  Portexpander und Freigaben");
    CHK_OUT.println("  6  BNO055");
    CHK_OUT.println("  7  Radar");
    CHK_OUT.println("  8  IR-Ring");
    CHK_OUT.println("  9  Liniensensoren");
    CHK_OUT.println("  0  Motor-ADC (Ruhezustand)");
    CHK_OUT.println("  s  Schalter und Taster (Momentaufnahme)");
    CHK_OUT.println("  t  Kontaktdurchlauf — jeden Schalter/Taster einzeln");
    CHK_OUT.println("  m  Aktortest — die Raeder drehen sich, Roboter aufbocken!");
    CHK_OUT.println("  l  Dauerueberwachung der Live-Werte");
    CHK_OUT.println("  h  diese Hilfe");
    CHK_OUT.println();
}

/// Eine einzelne Stufe mit eigenem Zaehlerstand ausfuehren.
static void einzeln(void (*stufe)(void))
{
    zaehlerNull();
    stufe();
    zusammenfassung();
}

/* ============================================================
 *  setup / loop
 * ============================================================ */

void setup()
{
    CHK_OUT.begin(115200);
    delay(500);   // dem USB-Serial Zeit geben, sonst fehlen die ersten Zeilen

    hardwareInit();

    allesPruefen();
    hilfe();
}

void loop()
{
    if (CHK_OUT.available())
    {
        switch (CHK_OUT.read())
        {
        case 'a': allesPruefen();          break;
        case '1': einzeln(stufe_teensy);   break;
        case '2': zaehlerNull(); sbAdressenSuchen(); stufe_i2c(Wire,  0, wireGefunden);  zusammenfassung(); break;
        case '3': zaehlerNull();                     stufe_i2c(Wire1, 1, wire1Gefunden); zusammenfassung(); break;
        case '4': einzeln(stufe_power);    break;
        case '5': einzeln(stufe_expander); break;
        case '6': einzeln(stufe_bno);      break;
        case '7': einzeln(stufe_radar);    break;
        case '8': einzeln(stufe_irring);   break;
        case '9': einzeln(stufe_lineADC);  break;
        case '0': einzeln(stufe_motorADC); break;
        case 's': einzeln(stufe_schalter); break;
        case 't': einzeln(stufe_kontakte); break;
        case 'm': einzeln(stufe_aktor);    break;
        case 'l': liveMonitor();           break;
        case 'h': hilfe();                 break;
        default:  break;
        }
    }

    // Radar im Hintergrund weiterlaufen lassen, damit Stufe 7 nicht bei jedem
    // Aufruf von vorn warmlaufen muss.
    radar.update();
    delay(5);
}
