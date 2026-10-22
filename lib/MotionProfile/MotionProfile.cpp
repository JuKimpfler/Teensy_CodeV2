#include <MotionProfile.h>
#include <Utils.h>

namespace
{
/// Weiche 0-1-Stufe ohne Knick an den Enden (3x^2 - 2x^3).
float smoothstep01(float x)
{
    x = constrain(x, 0.0f, 1.0f);
    return x * x * (3.0f - 2.0f * x);
}

bool samePoint(const ArrayPoint &a, const ArrayPoint &b)
{
    return a.X == b.X && a.Y == b.Y && a.Andrehwinkel == b.Andrehwinkel && a.SpeedPattern == b.SpeedPattern &&
           a.MaxSpeed == b.MaxSpeed && a.MinSpeed == b.MinSpeed && a.HoldMs == b.HoldMs && a.PassCm == b.PassCm;
}

/// Fahrtrichtung vom Roboter zum Ziel [Grad, +-180 CW]
float directionTo(float dx, float dy)
{
    return Util.wrapAngle(atan2f(-dx, dy) * RAD_TO_DEG);
}
} // namespace

void MotionProfile::begin()
{
    resetArray();
    _lastSpeed = 0.0f;
    _lastUs = 0;
}

MotionCommand MotionProfile::stopCommand(int16_t Andrehwinkel)
{
    MotionCommand cmd;
    cmd.Active = true;
    cmd.DirectionDeg = 0.0f;
    cmd.SpeedPct = 0;
    cmd.Andrehwinkel = Andrehwinkel;
    return cmd;
}

/* ========================================================================
 * Speed-Pattern
 *
 * Ein Pattern beschreibt nur den VERLAUF der Geschwindigkeit ueber den
 * Fahrweg: Fortschritt p = 0 (Start) ... 1 (Ziel), Ergebnis f(p) = 0 ... 1.
 * Gefahren wird damit:  Speed = MinSpeed + (MaxSpeed - MinSpeed) * f(p)
 * Die Liste der Pattern steht in params_motion.h (SPattern_*).
 * ===================================================================== */

float MotionProfile::giveSpeedPattern(uint8_t pattern, float progress)
{
    const float p = constrain(progress, 0.0f, 1.0f);
    float f;

    switch (pattern)
    {
    case SPattern_Linear:
        f = 1.0f - p;
        break;
    case SPattern_LinearUp:
        f = p;
        break;
    case SPattern_Constant:
        f = 1.0f;
        break;
    case SPattern_Trapezoid:
        f = min(min(p / 0.2f, 1.0f), (1.0f - p) / 0.4f);
        break;
    case SPattern_Triangle:
        f = 1.0f - fabsf(1.0f - 2.0f * p);
        break;
    case SPattern_QuadDecel:
        f = (1.0f - p) * (1.0f - p);
        break;
    case SPattern_ConstDecel:
        f = sqrtf(1.0f - p);
        break;
    case SPattern_SmoothStep:
        f = 1.0f - smoothstep01(p);
        break;
    case SPattern_Cosine:
        f = 0.5f * (1.0f + cosf(PI * p));
        break;
    case SPattern_Sine:
        f = sinf(PI * p);
        break;
    case SPattern_SCurve:
        f = min(smoothstep01(p / 0.25f), 1.0f - smoothstep01((p - 0.6f) / 0.4f));
        break;
    case SPattern_LateBrake:
        f = (p < 0.7f) ? 1.0f : 0.5f * (1.0f + cosf(PI * (p - 0.7f) / 0.3f));
        break;
    case SPattern_FastThenCrawl:
        f = (p < 0.5f) ? 1.0f : ((p < 0.7f) ? 1.0f - (p - 0.5f) / 0.2f : 0.0f);
        break;
    case SPattern_QuadUp:
        f = p * p;
        break;
    case SPattern_CubicUp:
        f = p * p * p;
        break;
    case SPattern_Expo:
        f = (expf(-3.0f * p) - expf(-3.0f)) / (1.0f - expf(-3.0f));
        break;
    case SPattern_Stairs:
        f = 1.0f - min(floorf(p * 4.0f), 3.0f) / 3.0f;
        break;
    case SPattern_Wave:
        f = (1.0f - p) * (0.6f + 0.4f * sinf(2.0f * PI * 3.0f * p));
        break;
    case SPattern_Pulse:
        f = (fmodf(p * 5.0f, 1.0f) < 0.6f) ? 1.0f : 0.0f;
        break;
    default:
        f = 1.0f;
        break; // unbekanntes Pattern: konstant
    }

    return constrain(f, 0.0f, 1.0f);
}

bool MotionProfile::isNearWall(float X, float Y)
{
    if (X < 0.0f || Y < 0.0f)
    {
        return false; // Position unbekannt: der Wandabstand ist nicht bekannt
    }

    return X < MOVE_WALL_MARGIN_CM || X > (FIELD_X_CM - MOVE_WALL_MARGIN_CM) ||
           Y < MOVE_WALL_MARGIN_CM || Y > (FIELD_Y_CM - MOVE_WALL_MARGIN_CM);
}

/* ========================================================================
 * Zielanfahrt
 * ===================================================================== */

void MotionProfile::resetProfile()
{
    _active = false; // _lastSpeed bleibt: rollt der Roboter weiter, steigt der Speed stetig statt von 0
}

void MotionProfile::stopProfile()
{
    _active = false;
    _lastSpeed = 0.0f; // Roboter steht -> der Anlauf ist beim naechsten Mal wieder begrenzt
}

bool MotionProfile::giveArrived() const { return _active && _arrived; }

float MotionProfile::giveProgress() const { return _progress; }

/**
 * Einfache Zielanfahrt ohne Speed-Pattern.
 *
 * Das Tempo faellt auf den letzten MOVE_SLOWDOWN_CM linear ab, naeher als
 * MOVE_SIMPLE_STOP_CM bleibt der Roboter stehen. Zielkreis, Fortschritt und
 * giveArrived() gibt es nur bei moveToPoint().
 *
 * Zyklisch aufrufen. Position unbekannt -> Speed 0, der Winkel wird weiter geregelt.
 */
MotionCommand MotionProfile::moveToPointSimple(const MotionInput &in, uint16_t X, uint16_t Y,
                                               uint8_t Speed, int16_t Andrehwinkel)
{
    if (!in.PosValid)
    {
        return stopCommand(Andrehwinkel);
    }

    const float dx = X - in.X;
    const float dy = Y - in.Y;
    const float dist = sqrtf(dx * dx + dy * dy);

    float speed = Speed * constrain(dist / MOVE_SLOWDOWN_CM, 0.0f, 1.0f);
    if (dist < MOVE_SIMPLE_STOP_CM)
    {
        speed = 0.0f;
    }

    MotionCommand cmd;
    cmd.Active = true;
    cmd.DirectionDeg = directionTo(dx, dy);
    cmd.SpeedPct = (uint8_t)speed;
    cmd.Andrehwinkel = Andrehwinkel;
    return cmd;
}

/**
 * Faehrt zum Punkt (X, Y) [cm, Feldkoordinaten] und dreht dabei auf Andrehwinkel [Grad].
 *
 * @param SpeedPattern SPattern_* aus params_motion.h (Verlauf ueber den Fahrweg)
 * @param MaxSpeed     Speed [%] bei f = 1
 * @param MinSpeed     Speed [%] bei f = 0 (Mindestfahrt, damit der Roboter nicht stehen bleibt)
 * @param Andrehwinkel Soll-Heading [Grad], wird waehrend der ganzen Fahrt geregelt
 *
 * Zyklisch aufrufen. Position ungueltig -> Speed 0 und Profil verworfen, der Winkel
 * wird weiter geregelt. Zielkreis MOVE_ARRIVE_CM: darin Speed 0; wird der Roboter
 * weggeschoben, faehrt er nach MOVE_ARRIVE_HYST_CM mit MinSpeed zurueck.
 * Status: giveArrived(). Der Speed darf hoechstens mit MOVE_MAX_ACCEL_PCT_S
 * ansteigen (weicher Anlauf, Bremsen nie begrenzt).
 */
MotionCommand MotionProfile::moveToPoint(const MotionInput &in, uint16_t X, uint16_t Y,
                                         uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed,
                                         int16_t Andrehwinkel)
{
    if (MaxSpeed > 100)
        MaxSpeed = 100;
    if (MinSpeed > MaxSpeed)
        MinSpeed = MaxSpeed;

    float dt = (_lastUs != 0) ? (in.NowUs - _lastUs) * 1e-6f : 0.0f;
    if (dt > 0.1f)
        dt = 0.0f; // lange Aufrufpause: Anlaufbegrenzung beginnt von vorn statt sofort 60 % freizugeben
    _lastUs = in.NowUs;

    // Position verloren: anhalten, Profil verwerfen
    if (!in.PosValid)
    {
        resetProfile();
        _lastSpeed = 0.0f;
        _lastCallMs = in.NowMs;
        return stopCommand(Andrehwinkel);
    }

    const float tx = constrain((float)X, 0.0f, FIELD_X_CM);
    const float ty = constrain((float)Y, 0.0f, FIELD_Y_CM);
    const float dx = tx - in.X;
    const float dy = ty - in.Y;
    const float dist = sqrtf(dx * dx + dy * dy);

    // ---- Neustart des Patterns? -----------------------------------------
    const bool sameTarget = (fabsf(tx - _tx) <= MOVE_TARGET_TOL_CM) && (fabsf(ty - _ty) <= MOVE_TARGET_TOL_CM);
    const bool restart = !_active || !sameTarget || (SpeedPattern != _pattern) || (MaxSpeed != _maxSpeed) ||
                         (MinSpeed != _minSpeed) || ((in.NowMs - _lastCallMs) > MOVE_RESTART_GAP_MS);
    _lastCallMs = in.NowMs;

    if (restart)
    {
        _active = true;
        _tx = tx;
        _ty = ty;
        _pattern = SpeedPattern;
        _maxSpeed = MaxSpeed;
        _minSpeed = MinSpeed;
        _dist0 = dist;
        _progress = 0.0f;
        _arrived = (dist <= MOVE_ARRIVE_CM);
        _correcting = false;
        // _lastSpeed bleibt: rollt der Roboter noch, wird der Speed nicht ruckartig umgesetzt
    }

    // ---- Zielkreis mit Hysterese -----------------------------------------
    if (_arrived)
    {
        if (dist > MOVE_ARRIVE_CM + MOVE_ARRIVE_HYST_CM) // weggeschoben -> Feinkorrektur
        {
            _arrived = false;
            _correcting = true;
            _dist0 = dist;
            _progress = 0.0f;
        }
    }
    else if (dist <= MOVE_ARRIVE_CM)
    {
        _arrived = true;
    }

    // ---- Geschwindigkeit -------------------------------------------------
    float speed = 0.0f;
    if (!_arrived)
    {
        _progress = (_dist0 > MOVE_ARRIVE_CM) ? constrain(1.0f - dist / _dist0, 0.0f, 1.0f) : 1.0f;

        const float f = _correcting ? 0.0f : giveSpeedPattern(_pattern, _progress);
        speed = MinSpeed + (MaxSpeed - MinSpeed) * f;

        // weicher Anlauf: Zunahme begrenzen, Abnahme (Bremsen) nie
        if (MOVE_MAX_ACCEL_PCT_S > 0.0f && speed > _lastSpeed + MOVE_MAX_ACCEL_PCT_S * dt)
            speed = _lastSpeed + MOVE_MAX_ACCEL_PCT_S * dt;
    }
    else
    {
        _progress = 1.0f;
    }
    _lastSpeed = speed;

    MotionCommand cmd;
    cmd.Active = true;
    cmd.DirectionDeg = directionTo(dx, dy);
    cmd.SpeedPct = (uint8_t)constrain(speed + 0.5f, 0.0f, 100.0f);
    cmd.Andrehwinkel = Andrehwinkel;
    return cmd;
}

/* ========================================================================
 * Punkt-Liste: Zielpunkte, die nacheinander angefahren werden
 *
 * Jeder Punkt hat Ziel (X, Y), Andrehwinkel, Speed-Pattern, Max-/MinSpeed und
 * optional eine Wartezeit. Gefahren wird mit moveToPoint(), der Verlauf des
 * Speed-Patterns startet also an jedem Punkt neu.
 *
 * Ein Punkt gilt als erreicht, wenn der Zielkreis (MOVE_ARRIVE_CM) erreicht ist,
 * der Roboter auf Andrehwinkel +-ARRAY_HEADING_TOL_DEG steht und die Wartezeit
 * abgelaufen ist.
 * ===================================================================== */

/**
 * Ersetzt die komplette Liste. Zyklisch aufrufbar: sind Punkte und loop unveraendert,
 * passiert nichts (Fortschritt bleibt). Bei jeder Aenderung beginnt die Liste wieder
 * beim ersten Punkt (ein laufender oder pausierter Zustand bleibt erhalten,
 * ein beendeter/ruhender wird zu Idle -> startArray()).
 *
 * @param loop true = nach dem letzten Punkt wieder beim ersten weitermachen
 * @return false, wenn die Liste laenger als ARRAY_MAX_POINTS ist (dann unveraendert)
 */
bool MotionProfile::setPointArray(const ArrayPoint *points, uint8_t count, bool loop)
{
    if (count > ARRAY_MAX_POINTS || (points == nullptr && count > 0))
    {
        return false;
    }

    bool same = (count == _count) && (loop == _loop);
    for (uint8_t i = 0; same && i < count; i++)
    {
        same = samePoint(points[i], _points[i]);
    }
    if (same)
    {
        return true;
    }

    for (uint8_t i = 0; i < count; i++)
    {
        _points[i] = points[i];
    }
    _count = count;
    _loop = loop;
    _index = 0;
    _holding = false;
    resetProfile();

    if (count == 0 || _state == ArrayFinished)
    {
        _state = ArrayIdle;
    }
    return true;
}

/**
 * Haengt einen Punkt hinten an. Eine laufende Fahrt wird dadurch nicht unterbrochen;
 * war die Liste schon beendet, faehrt der Roboter zum neuen Punkt weiter.
 *
 * @param HoldMs Wartezeit nach Erreichen des Punkts [ms], bevor der naechste folgt
 * @return false, wenn die Liste voll ist (ARRAY_MAX_POINTS)
 */
bool MotionProfile::addPointToArray(uint16_t X, uint16_t Y, int16_t Andrehwinkel, uint8_t SpeedPattern,
                                    uint8_t MaxSpeed, uint8_t MinSpeed, uint16_t HoldMs)
{
    ArrayPoint p;
    p.X = X;
    p.Y = Y;
    p.Andrehwinkel = Andrehwinkel;
    p.SpeedPattern = SpeedPattern;
    p.MaxSpeed = MaxSpeed;
    p.MinSpeed = MinSpeed;
    p.HoldMs = HoldMs;
    p.PassCm = 0;
    return addPointToArray(p);
}

bool MotionProfile::addPointToArray(const ArrayPoint &point)
{
    if (_count >= ARRAY_MAX_POINTS)
    {
        return false;
    }

    _points[_count] = point;
    _count++;

    if (_state == ArrayFinished) // Liste war fertig -> zum neuen Punkt weiterfahren
    {
        _index = _count - 1;
        _state = ArrayRunning;
        _holding = false;
        resetProfile();
    }
    return true;
}

/** Loescht alle Punkte und setzt den Zustand auf Idle. updateArray() liefert dann keinen Fahrbefehl mehr. */
void MotionProfile::resetArray()
{
    _count = 0;
    _index = 0;
    _loop = false;
    _state = ArrayIdle;
    _holding = false;
    _routeValid = false;
    _routeTotal0 = 1.0f;
    resetProfile();
}

/**
 * Startet die Liste beim ersten Punkt. Zyklisch aufrufbar: laeuft sie schon, ist sie
 * pausiert (-> resumeArray()) oder beendet, passiert nichts.
 *
 * @param restart true = in jedem Zustand von vorn beginnen
 */
void MotionProfile::startArray(bool restart)
{
    if (_count == 0)
    {
        return;
    }
    if (_state != ArrayIdle && !restart)
    {
        return;
    }

    _index = 0;
    _holding = false;
    _state = ArrayRunning;
    resetProfile();
}

/** Haelt den Roboter an (der Winkel wird weiter geregelt). Der Fortschritt bleibt erhalten. */
void MotionProfile::pauseArray()
{
    if (_state == ArrayRunning)
    {
        _state = ArrayPaused;
        _holding = false;
        stopProfile(); // Roboter steht -> Anlauf nach resumeArray() wieder begrenzt
    }
}

/** Setzt eine pausierte Liste am aktuellen Punkt fort (das Speed-Pattern startet fuer diesen Punkt neu). */
void MotionProfile::resumeArray()
{
    if (_state == ArrayPaused)
    {
        _state = ArrayRunning;
        _holding = false;
        resetProfile();
    }
}

void MotionProfile::setArrayHeading(int16_t Andrehwinkel)
{
    for (uint8_t i = 0; i < _count; i++)
    {
        _points[i].Andrehwinkel = Andrehwinkel;
    }
}

/**
 * Jeden Zyklus aufrufen. Faehrt den aktuellen Punkt an und schaltet weiter,
 * sobald er erreicht ist.
 *
 *  Idle      -> Command.Active = false, der Aufrufer laesst die Motoren in Ruhe
 *  Running   -> faehrt / wartet am Punkt
 *  Paused    -> steht, haelt Andrehwinkel des aktuellen Punkts
 *  Finished  -> haelt Position und Winkel des letzten Punkts
 */
MotionCommand MotionProfile::updateArray(const MotionInput &in)
{
    MotionCommand idle;
    idle.Active = false;
    idle.DirectionDeg = 0.0f;
    idle.SpeedPct = 0;
    idle.Andrehwinkel = 0;

    if (_count == 0)
    {
        return idle;
    }

    const ArrayPoint &p = _points[_index];

    switch (_state)
    {
    case ArrayRunning:
        break;

    case ArrayFinished:
        return moveToPoint(in, p.X, p.Y, p.SpeedPattern, p.MaxSpeed, p.MinSpeed, p.Andrehwinkel);

    case ArrayPaused:
        return stopCommand(p.Andrehwinkel);

    default: // ArrayIdle
        return idle;
    }

    const MotionCommand cmd = moveToPoint(in, p.X, p.Y, p.SpeedPattern, p.MaxSpeed, p.MinSpeed, p.Andrehwinkel);

    const bool lastPoint = (_index + 1 >= _count) && !_loop;

    // ---- Durchfahrpunkt: weiterschalten, ohne anzuhalten -----------------
    // Zielkreis, Winkel und Wartezeit gelten nur fuer Haltepunkte (PassCm = 0) und den letzten Punkt.
    if (p.PassCm > 0 && !lastPoint)
    {
        _holding = false;

        if (in.PosValid)
        {
            const float dx = p.X - in.X;
            const float dy = p.Y - in.Y;
            if (sqrtf(dx * dx + dy * dy) <= p.PassCm)
            {
                _index = (_index + 1 < _count) ? (_index + 1) : 0;
                resetProfile(); // Pattern fuer den naechsten Punkt von vorn, Speed laeuft stetig weiter
            }
        }
        return cmd;
    }

    // ---- Punkt erreicht? -------------------------------------------------
    const float headingErr = fabsf(Util.wrapAngle(p.Andrehwinkel - in.HeadingDeg));
    const bool reached = giveArrived() && (ARRAY_HEADING_TOL_DEG <= 0.0f || headingErr <= ARRAY_HEADING_TOL_DEG);

    if (!reached)
    {
        _holding = false;
        return cmd;
    }

    if (!_holding)
    {
        _holding = true;
        _holdStartMs = in.NowMs;
    }
    if ((uint32_t)(in.NowMs - _holdStartMs) < p.HoldMs)
    {
        return cmd; // Wartezeit laeuft
    }

    // ---- weiterschalten --------------------------------------------------
    _holding = false;
    if (_index + 1 < _count)
    {
        _index++;
        resetProfile(); // Pattern fuer den neuen Punkt von vorn, auch bei gleichem Ziel
    }
    else if (_loop)
    {
        _index = 0;
        resetProfile();
    }
    else
    {
        _state = ArrayFinished;
    }
    return cmd;
}

ArrayState MotionProfile::giveArrayState() const { return _state; }
uint8_t MotionProfile::giveArrayIndex() const { return _index; }
uint8_t MotionProfile::giveArrayCount() const { return _count; }

/* ========================================================================
 * Route aus der Pfadplanung
 *
 * RoutePlanner liefert nur Geometrie. Das Speed-Pattern gilt fuer die GANZE
 * Strecke: Zwischenpunkte bekommen je ein konstantes Tempo, das aus dem
 * Pattern an der Stelle des Teilstuecks berechnet wird, und werden ohne Halt
 * durchfahren (PATH_PASS_CM). Das letzte Teilstueck faehrt das Pattern selbst,
 * inklusive Zielkreis und Andrehwinkel.
 * ===================================================================== */

bool MotionProfile::setRoute(const Route &route, uint8_t SpeedPattern, uint8_t MaxSpeed, uint8_t MinSpeed,
                             int16_t Andrehwinkel)
{
    if (route.Count == 0 || route.Count > ARRAY_MAX_POINTS)
    {
        return false;
    }

    const uint16_t tx = route.Points[route.Count - 1].X;
    const uint16_t ty = route.Points[route.Count - 1].Y;

    // Der Pattern-Fortschritt bezieht sich auf die Strecke bei Fahrtbeginn (neues Ziel /
    // Pattern / Speed). Periodische Neuplanungen aendern das nicht, sonst wuerde das Tempo
    // bei jeder Neuplanung neu anlaufen.
    const bool restart = !_routeValid || tx != _routeTx || ty != _routeTy || SpeedPattern != _routePattern ||
                         MaxSpeed != _routeMaxS || MinSpeed != _routeMinS;

    const float total = (route.TotalCm < 1.0f) ? 1.0f : route.TotalCm;
    if (restart || total > _routeTotal0)
    {
        _routeTotal0 = total;
    }

    _routeValid = true;
    _routeTx = tx;
    _routeTy = ty;
    _routePattern = SpeedPattern;
    _routeMaxS = MaxSpeed;
    _routeMinS = MinSpeed;

    ArrayPoint points[ARRAY_MAX_POINTS];
    float cum = 0.0f;

    for (uint8_t i = 0; i < route.Count; i++)
    {
        const float pStart = constrain(1.0f - (total - cum) / _routeTotal0, 0.0f, 1.0f);
        const float pMid = constrain(1.0f - (total - cum - 0.5f * route.Points[i].SegmentCm) / _routeTotal0, 0.0f, 1.0f);
        cum += route.Points[i].SegmentCm;

        ArrayPoint &a = points[i];
        a.X = route.Points[i].X;
        a.Y = route.Points[i].Y;
        a.Andrehwinkel = Andrehwinkel;
        a.HoldMs = 0;

        if (i + 1 < route.Count) // Zwischenpunkt: konstantes Tempo, ohne Halt durchfahren
        {
            const float v = MinSpeed + (MaxSpeed - MinSpeed) * giveSpeedPattern(SpeedPattern, pMid);
            a.SpeedPattern = SPattern_Constant;
            a.MaxSpeed = (uint8_t)(v + 0.5f);
            a.MinSpeed = a.MaxSpeed;
            a.PassCm = PATH_PASS_CM;
        }
        else // letztes Teilstueck: Pattern bis zum Ziel
        {
            const float v0 = MinSpeed + (MaxSpeed - MinSpeed) * giveSpeedPattern(SpeedPattern, pStart);
            a.SpeedPattern = SpeedPattern;
            a.MaxSpeed = (uint8_t)(v0 + 0.5f);
            a.MinSpeed = MinSpeed;
            a.PassCm = 0;
        }
    }

    // Zeiger-Ueberladung erzwingen: points ist voll dimensioniert, es zaehlt aber
    // nur route.Count. Ohne den Cast ist der Aufruf mehrdeutig.
    return setPointArray((const ArrayPoint *)points, route.Count); // unveraendert: kein Neustart
}
