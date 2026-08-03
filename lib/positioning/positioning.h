#pragma once
#include <Arduino.h>

// ╔══════════════════════════════════════════════════════════════════════════╗
// ║  positioning  —  Positionsbestimmung (x,y) aus Radar + Kamera            ║
// ║  RoboCup Junior Soccer 2v2 Lightweight                                   ║
// ╠══════════════════════════════════════════════════════════════════════════╣
// ║  Ablauf pro Zyklus:                                                      ║
// ║    1. Radar zuerst: pro Achse wird geprueft, ob die beiden gegenueber-   ║
// ║       liegenden Sensoren ein Peak-Paar liefern, dessen Summe exakt der   ║
// ║       Feldbreite/-laenge entspricht. Ist das Paar eindeutig, ist die     ║
// ║       Achse fertig -- die Kamera wird gar nicht erst ausgewertet.        ║
// ║    2. Nur wenn eine Achse in sich nicht plausibel ist (kein Paar oder    ║
// ║       mehrere moegliche Paare), wird die Kamera-Triangulation berechnet  ║
// ║       und als Referenz benutzt, um zu entscheiden welcher Peak die Wand  ║
// ║       ist und welcher ein Roboter.                                       ║
// ║    3. Liefert ein Sensor einer Achse nichts Plausibles (z.B. durch einen ║
// ║       Gegner verdeckt), wird der Wert des anderen Sensors genommen --    ║
// ║       sofern er grob (camTol_cm) zur Kamera passt.                       ║
// ║    4. Erst wenn gar kein Radar-Wert brauchbar ist, wird die Kamera-      ║
// ║       position direkt uebernommen (bzw. die letzte Position gehalten).   ║
// ║    => Prioritaet liegt immer bei den Radarsensoren, die Kamera ist nur   ║
// ║       Schiedsrichter. Sie darf ruhig ~30 cm daneben liegen.              ║
// ╠══════════════════════════════════════════════════════════════════════════╣
// ║  Koordinatensystem (Coding_Rules.md):                                    ║
// ║    Ursprung  : linke untere Ecke am EIGENEN Tor                          ║
// ║    x [cm]    : 0 = linke Wand      .. fieldWidth_cm  (182)               ║
// ║    y [cm]    : 0 = eigene Torlinie .. fieldLength_cm (243)               ║
// ║    heading   : Kompasswinkel in Grad, +-180, im Uhrzeigersinn,           ║
// ║                0 deg = Roboterfront zeigt Richtung +y (gegnerisches Tor) ║
// ║                also z.B. Util.AMap(bno.eulHeading())                     ║
// ║    x_coord/y_coord : Index im 2 cm-Raster (x_coord = round(x / 2))       ║
// ╚══════════════════════════════════════════════════════════════════════════╝

namespace Positioning
{

    // ── Feldgeometrie [cm] ────────────────────────────────────────────────
    static constexpr float FIELD_WIDTH_CM  = 182.0f; // x: Wand zu Wand
    static constexpr float FIELD_LENGTH_CM = 243.0f; // y: Tor zu Tor

    static constexpr float POS_GRID_CM = 2.0f; // Rasterweite fuer x_coord/y_coord

    static constexpr uint8_t RADAR_MAX_PEAKS = 10; // wie XM125D_MAX_PEAKS

    // Montagereihenfolge auf dem Radar-Ring (90 deg-Schritte, CW)
    enum RadarIndex : uint8_t
    {
        RADAR_FRONT = 0,
        RADAR_RIGHT = 1,
        RADAR_BACK  = 2,
        RADAR_LEFT  = 3,
        RADAR_COUNT = 4
    };

    enum AxisIndex : uint8_t
    {
        AXIS_X = 0,
        AXIS_Y = 1,
        AXIS_COUNT = 2
    };

    // Bewertung eines einzelnen Radar-Peaks nach der Positionsbestimmung
    enum class PeakType : uint8_t
    {
        Unknown = 0, // kein Peak / nicht bewertbar
        Wall,        // passt zur Feldwand
        Robot,       // naeher als die Wand -> Hindernis (Roboter, Ball-Halter, ...)
        Behind       // weiter als die Wand -> Mehrfachreflexion / Ausreisser
    };

    // Woher die Koordinate einer Achse letztlich stammt
    enum class AxisSource : uint8_t
    {
        None = 0,
        RadarPair,   // beide Sensoren, Summe == Feldmass (bester Fall)
        RadarSingle, // nur ein Sensor, gegen Referenz geprueft
        Camera,      // kein Radarwert brauchbar -> Kamera
        Hold         // gar nichts brauchbar -> letzte Position gehalten
    };

    // ── Rohwert-Container (vom Aufrufer pro Zyklus befuellt) ──────────────
    struct RadarPeak
    {
        float   distance_cm = -1.0f; // Distanz Sensor -> Reflektor
        int32_t strength    = 0;     // Signalstaerke (XM125: getPeakStrength)
    };

    struct RadarReading
    {
        bool     valid     = false; // measure() war erfolgreich
        uint8_t  peakCount = 0;
        RadarPeak peaks[RADAR_MAX_PEAKS];
    };

    struct CamReading
    {
        bool  valid           = false; // cam.isValid()
        float angle_field_deg = 0.0f;  // Peilung Roboter -> Tor im Feld-KS (cam.give_Angle())
        int   blobHeight_px   = 0;     // cam.give_BlobH()
    };

    // ── Ergebnis ──────────────────────────────────────────────────────────
    struct position
    {
        int   x_coord = 0, y_coord = 0; // Koordinaten im Feld (2cm Raster)
        float x = 0.0f, y = 0.0f;       // genaue Position im Feld [cm]
    };

    struct AxisResult
    {
        bool       valid    = false;
        float      coord_cm = 0.0f;
        AxisSource source   = AxisSource::None;
        bool       camUsed  = false; // Kamera musste zum Validieren herangezogen werden
        bool       ambiguous = false; // mehrere Paare/Peaks kamen in Frage
        uint8_t    pairCount = 0;     // gefundene Peak-Paare mit Summe == Feldmass
        int8_t     wallPeak[2] = {-1, -1}; // [0] Sensor in +Achse, [1] Sensor in -Achse
    };

    struct Status
    {
        AxisResult axis[AXIS_COUNT];

        bool  camValid        = false; // Kamerareferenz vorhanden
        bool  camTriangulated = false; // echte Triangulation (beide Tore gesehen)
        bool  camEvaluated    = false; // Kamera wurde ueberhaupt ausgewertet
        float camX = 0.0f, camY = 0.0f;
        float camQuality = 0.0f; // 0..1, Schnittgueete der beiden Peilungen

        float confidence = 0.0f; // 0..1

        // Peak-Klassifizierung: peak[Sensor][Index]
        PeakType peak[RADAR_COUNT][RADAR_MAX_PEAKS] = {};

        float  sensorFacingDeg[RADAR_COUNT] = {}; // Blickrichtung im Feld-KS
        int8_t sensorAxis[RADAR_COUNT] = {};      // AXIS_X / AXIS_Y, -1 = zu schraeg
        int8_t sensorSign[RADAR_COUNT] = {};      // +1 = schaut Richtung +Achse
        float  sensorTiltDeg[RADAR_COUNT] = {};   // Winkel zwischen Blickrichtung und Achse
    };

    // ── Konfiguration ─────────────────────────────────────────────────────
    struct Config
    {
        float fieldWidth_cm  = FIELD_WIDTH_CM;
        float fieldLength_cm = FIELD_LENGTH_CM;

        // Montage
        float radarRingRadius_cm = 9.0f; // Sensorstirn -> Roboterzentrum
        float mountAngleDeg[RADAR_COUNT] = {0.0f, 90.0f, 180.0f, 270.0f};

        bool  hasRadarServo  = false; // Ring wird gegen die Roboterdrehung gekontert
        float servoNeutralDeg = 90.0f;
        float servoRangeDeg   = 45.0f;
        bool  servoInverted   = false;

        // Ein Sensor zaehlt nur fuer eine Achse, wenn er hoechstens so schraeg
        // zur Wand steht (Projektion ueber cos wird sonst zu ungenau).
        float maxTiltDeg = 20.0f;
        // true : Sensor misst bei Schraegstellung den Lotabstand zur Wand
        // false: Sensor misst die Strahllaenge -> Projektion mit cos(tilt)
        bool  perpendicularModel = false;

        // Toleranzen
        float pairTol_cm   = 12.0f; // |d+ + d- - Feldmass| fuer ein gueltiges Wandpaar
        float pairAgree_cm = 8.0f;  // Paare, die sich um weniger unterscheiden, gelten als dasselbe
        float camTol_cm    = 40.0f; // Kamera darf gut 30 cm daneben liegen
        float holdTol_cm   = 35.0f; // Toleranz gegen die letzte Position (ohne Kamera)
        float wallTol_cm   = 10.0f; // Fenster fuer die Peak-Klassifizierung Wand/Roboter
        float minWallDist_cm = 4.0f;   // naeher kann eine Wand nicht sein
        float outOfField_cm  = 15.0f;  // erlaubtes Ueberschiessen ueber die Feldgrenze

        int32_t minPeakStrength = 0; // Peaks darunter werden ignoriert (0 = aus)

        // Kamera
        int   camMinBlobHeight_px = 4;    // kleinere Blobs sind zu unsicher
        float camMinQuality       = 0.30f; // |sin| zwischen den beiden Peilungen
        float camMinAxisSin       = 0.30f; // Mindest-Projektion fuer Einzelpeilung

        // Ausgangsfilter: 1.0 = ungefiltert (volle Radar-Prioritaet)
        float outputAlpha = 1.0f;

        // Rohwert-Umrechnung des XM125 (siehe getPeakDistanceRaw()-Warnung!)
        // distanz_cm = rawOffset_cm + raw * rawScale_cm
        float rawOffset_cm = 20.0f;  // == config.startMm
        float rawScale_cm  = 0.902f; // == (endMm - startMm) / 255 / 10
    };

    // ── API ───────────────────────────────────────────────────────────────

    /** Einmal in setup() aufrufen. */
    void begin(const Config &cfg = Config());

    /** Position hart setzen (z.B. bekannte Kickoff-Position). */
    void reset(float x_cm, float y_cm);

    /**
     * Haupt-Update, einmal pro loop() aufrufen.
     * @param headingDeg   BNO-Heading im Feld-KS (+-180 CW, 0 = Front Richtung +y)
     * @param camOpponent  Kamera, die das GEGNERISCHE Tor sieht (y = fieldLength)
     * @param camOwn       Kamera, die das EIGENE Tor sieht (y = 0)
     * @param radar        4 Radarwerte in Ringreihenfolge (vorne, rechts, hinten, links)
     */
    position update(float headingDeg,
                    const CamReading &camOpponent,
                    const CamReading &camOwn,
                    const RadarReading radar[RADAR_COUNT]);

    /** Letztes Ergebnis von update(). */
    position getPosition();

    /** Diagnose: Peak-Klassifizierung, Achsenquellen, Kamerareferenz. */
    const Status &getStatus();

    const Config &getConfig();

    /** Klassifizierung eines Peaks nach dem letzten update(). */
    PeakType getPeakType(uint8_t sensor, uint8_t peakIdx);

    /**
     * Zielwinkel fuer den Ring-Servo, damit der Radar-Ring feldparallel
     * bleibt. Der Ring ist 90 deg-symmetrisch, also muss nur der Rest
     * modulo 90 deg ausgeglichen werden (minimale Servobewegung).
     */
    float radarServoAngle(float headingDeg);

    /** Rohwert (0-255 Counts) des XM125 in cm, siehe Config::rawScale_cm. */
    float rawCountToCm(uint8_t raw);

} // namespace Positioning
