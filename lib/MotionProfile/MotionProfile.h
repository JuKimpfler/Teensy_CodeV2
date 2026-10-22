#pragma once
#include <Arduino.h>

#include "params.h"

/**
 * MotionProfile.h  --  Speed-Pattern, Zielanfahrt und Punkt-Liste
 *
 * Die Klasse rechnet nur. Sie kennt weder systemlib noch Sensoren, Motoren
 * oder Regler: sie bekommt je Zyklus Position, Winkel und Zeit (MotionInput)
 * und liefert zurueck, wohin und wie schnell gefahren werden soll
 * (MotionCommand). Umgesetzt wird das vom Aufrufer.
 *
 *   MotionInput  in = {millis(), micros(), Position.X, Position.Y, valid, heading};
 *   MotionCommand c = Profile.moveToPoint(in, 90, 150, SPattern_Linear, 80, 25, 0);
 *   if (c.Active) setOmniWheelDir(c.DirectionDeg, c.SpeedPct, omega);
 *
 * Einheiten (docs/Coding_Rules.md): Winkel +-180 Grad CW, Koordinaten in cm ab
 * der linken unteren Ecke am eigenen Tor, Speed 0..100 %.
 * Parameter: lib/include/params/params_motion.h. Doku: MotionProfile.md
 */

/// Zustand der Punkt-Liste, siehe MotionProfile::updateArray().
enum ArrayState : uint8_t
{
    ArrayIdle = 0, ///< nicht gestartet / leer
    ArrayRunning,  ///< faehrt die Punkte ab
    ArrayPaused,   ///< angehalten, Fortschritt bleibt
    ArrayFinished  ///< letzter Punkt erreicht
};

/// Ein Zielpunkt der Punkt-Liste, siehe MotionProfile::setPointArray().
struct ArrayPoint
{
    uint16_t X;           ///< Ziel X [cm]
    uint16_t Y;           ///< Ziel Y [cm]
    int16_t Andrehwinkel; ///< Soll-Heading [Grad]
    uint8_t SpeedPattern; ///< SPattern_* aus params_motion.h
    uint8_t MaxSpeed;     ///< Speed [%] bei Pattern-Wert 1
    uint8_t MinSpeed;     ///< Speed [%] bei Pattern-Wert 0
    uint16_t HoldMs;      ///< Wartezeit nach Erreichen des Punkts [ms]
    uint8_t PassCm;       ///< >0: Durchfahrpunkt, gilt ab diesem Abstand [cm] als erreicht (kein Halt); 0 = anhalten
};

/// Was der Aufrufer je Zyklus an MotionProfile uebergibt.
struct MotionInput
{
    uint32_t NowMs;     ///< millis()
    uint32_t NowUs;     ///< micros()
    float X;            ///< Feldposition [cm]
    float Y;            ///< Feldposition [cm]
    bool PosValid;      ///< false = Position unbekannt, X/Y werden dann nicht benutzt
    float HeadingDeg;   ///< Istwinkel [Grad, +-180 CW]
};

/// Fahrbefehl eines Zyklus, Ergebnis aller update-/move-Funktionen.
struct MotionCommand
{
    bool Active;         ///< false = kein Fahrbefehl, Motoren nicht anfassen
    float DirectionDeg;  ///< Fahrtrichtung im Feld [Grad, +-180 CW]
    uint8_t SpeedPct;    ///< 0 ... 100 %
    int16_t Andrehwinkel; ///< Soll-Ausrichtung [Grad] fuer den Kursregler
};

/// Ein Wegpunkt einer geplanten Route -- reine Geometrie, ohne Tempo.
struct RoutePoint
{
    uint16_t X;       ///< [cm]
    uint16_t Y;       ///< [cm]
    float SegmentCm;  ///< Laenge des Teilstuecks, das an diesem Punkt endet
};

/// Ergebnis der Pfadplanung (RoutePlanner). Das Tempo je Teilstueck rechnet MotionProfile.
struct Route
{
    RoutePoint Points[ARRAY_MAX_POINTS];
    uint8_t Count = 0;
    float TotalCm = 0.0f; ///< Summe aller Teilstuecke
};

class MotionProfile
{
public:
    void begin();

    /* -------------- Einzelziel -------------- */

    /// Zielanfahrt mit Speed-Pattern. Zyklisch aufrufen.
    MotionCommand moveToPoint(const MotionInput &in, uint16_t X, uint16_t Y,
                              uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed,
                              int16_t Andrehwinkel);

    /// Einfache Zielanfahrt ohne Pattern: lineare Bremsrampe ab MOVE_SLOWDOWN_CM.
    MotionCommand moveToPointSimple(const MotionInput &in, uint16_t X, uint16_t Y,
                                    uint8_t Speed, int16_t Andrehwinkel);

    void resetProfile();              ///< Pattern von vorn; rollt der Roboter weiter, laeuft der Speed stetig weiter
    void stopProfile();               ///< wie resetProfile(), aber der weiche Anlauf beginnt wieder bei 0
    bool giveArrived() const;         ///< true = im Zielkreis MOVE_ARRIVE_CM
    float giveProgress() const;       ///< Fortschritt 0..1 des aktuellen Patterns

    /// Verlauf f(p) eines Speed-Patterns; p = Fortschritt 0..1, Ergebnis 0..1.
    static float giveSpeedPattern(uint8_t pattern, float progress);

    /// true, wenn die Position naeher als MOVE_WALL_MARGIN_CM an einer Feldwand liegt.
    static bool isNearWall(float X, float Y);

    /* -------------- Punkt-Liste -------------- */

    bool setPointArray(const ArrayPoint *points, uint8_t count, bool loop = false);
    template <size_t N>
    bool setPointArray(const ArrayPoint (&points)[N], bool loop = false) { return setPointArray(points, (uint8_t)N, loop); }
    bool addPointToArray(uint16_t X, uint16_t Y, int16_t Andrehwinkel, uint8_t SpeedPattern,
                         uint8_t MaxSpeed, uint8_t MinSpeed, uint16_t HoldMs = 0);
    bool addPointToArray(const ArrayPoint &point);

    void resetArray();                  ///< Liste leeren, Zustand Idle
    void startArray(bool restart = false);
    void pauseArray();
    void resumeArray();
    MotionCommand updateArray(const MotionInput &in); ///< jeden Zyklus aufrufen

    ArrayState giveArrayState() const;
    uint8_t giveArrayIndex() const;
    uint8_t giveArrayCount() const;

    /* -------------- Route (von RoutePlanner) -------------- */

    /// Route uebernehmen und je Teilstueck das Tempo aus dem Pattern ableiten.
    /// Die Zwischenpunkte werden ohne Halt durchfahren, das letzte Stueck faehrt das Pattern.
    bool setRoute(const Route &route, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed,
                  int16_t Andrehwinkel);

    /// Andrehwinkel aller Punkte aendern, ohne die Liste neu zu setzen.
    void setArrayHeading(int16_t Andrehwinkel);

private:
    /* -------------- Fahrprofil -------------- */
    bool _active = false;
    bool _arrived = false;
    bool _correcting = false; ///< nach dem Wegschieben: Feinkorrektur mit MinSpeed
    float _tx = 0.0f;
    float _ty = 0.0f;
    uint8_t _pattern = 0;
    uint8_t _maxSpeed = 0;
    uint8_t _minSpeed = 0;
    float _dist0 = 0.0f; ///< Abstand beim Start des Patterns
    float _progress = 0.0f;
    float _lastSpeed = 0.0f;
    uint32_t _lastCallMs = 0;
    uint32_t _lastUs = 0;

    /* -------------- Punkt-Liste -------------- */
    ArrayPoint _points[ARRAY_MAX_POINTS];
    uint8_t _count = 0;
    uint8_t _index = 0;
    bool _loop = false;
    ArrayState _state = ArrayIdle;
    bool _holding = false; ///< Punkt erreicht, Wartezeit laeuft
    uint32_t _holdStartMs = 0;

    /* -------------- Route -------------- */
    float _routeTotal0 = 1.0f; ///< Streckenlaenge [cm], auf die sich der Pattern-Fortschritt bezieht
    bool _routeValid = false;
    uint16_t _routeTx = 0;
    uint16_t _routeTy = 0;
    uint8_t _routePattern = 0;
    uint8_t _routeMaxS = 0;
    uint8_t _routeMinS = 0;

    static MotionCommand stopCommand(int16_t Andrehwinkel);
};
