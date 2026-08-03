#include "positioning.h"
#include <math.h>

namespace Positioning
{

    namespace
    {

        // ── Zustand ───────────────────────────────────────────────────────
        Config   s_cfg;
        position s_pos;
        Status   s_status;
        bool     s_initialized = false;

        // Ein Wand-Kandidat: "wenn dieser Peak die Wand waere, staende der
        // Roboter auf dieser Achse bei coord".
        struct Candidate
        {
            int8_t  peakIdx  = -1;
            float   dCenter  = 0.0f; // Roboterzentrum -> Wand, entlang der Achse
            float   coord    = 0.0f; // daraus folgende Koordinate
            int32_t strength = 0;
        };

        // Ein Peak-Paar zweier gegenueberliegender Sensoren, dessen Summe dem
        // Feldmass entspricht.
        struct Match
        {
            float   coord    = 0.0f;
            int8_t  idxPlus  = -1;
            int8_t  idxMinus = -1;
            int32_t strength = 0;
        };

        static constexpr uint8_t MAX_MATCHES = 12;

        // Arbeitsdaten je Achse, einmal pro update() gefuellt
        struct AxisWork
        {
            int8_t    sensor[2] = {-1, -1};              // [0] = +Achse, [1] = -Achse
            uint8_t   nCand[2]  = {0, 0};
            Candidate cand[2][RADAR_MAX_PEAKS];
            uint8_t   nMatch = 0;
            Match     match[MAX_MATCHES];
        };

        AxisWork s_work[AXIS_COUNT];

        // Kamerareferenz, je Achse getrennt (eine Peilung kann auch nur eine
        // Achse stuetzen, wenn die andere schon vom Radar bekannt ist).
        struct CamRef
        {
            bool  has[AXIS_COUNT] = {false, false};
            float coord[AXIS_COUNT] = {0.0f, 0.0f};
            float quality = 0.0f;
            bool  triangulated = false;
        };

        // ── kleine Helfer ─────────────────────────────────────────────────
        float wrap180(float a)
        {
            while (a > 180.0f) a -= 360.0f;
            while (a < -180.0f) a += 360.0f;
            return a;
        }

        // Kompasswinkel -> Richtungsvektor im Feld-KS (0 deg = +y, CW)
        void dirFromCompass(float deg, float &outX, float &outY)
        {
            float rad = deg * DEG_TO_RAD;
            outX = sinf(rad);
            outY = cosf(rad);
        }

        float axisExtent(uint8_t axis)
        {
            return (axis == AXIS_X) ? s_cfg.fieldWidth_cm : s_cfg.fieldLength_cm;
        }

        int16_t snapToGrid(float v)
        {
            return (int16_t)lroundf(v / POS_GRID_CM);
        }

        // Rest der Roboterdrehung, den der Ring-Servo ausgleicht (Ring ist
        // 90 deg-symmetrisch -> nur der Anteil modulo 90 deg zaehlt).
        float ringCorrectionDeg(float headingDeg)
        {
            if (!s_cfg.hasRadarServo) return 0.0f;
            float m = fmodf(headingDeg, 90.0f);
            if (m < 0.0f) m += 90.0f;
            if (m > 45.0f) m -= 90.0f;
            if (s_cfg.servoInverted) m = -m;
            return m;
        }

        // Distanz von (px,py) zur naechsten Feldwand entlang der Kompass-
        // richtung dirDeg (Ray/AABB-Schnitt, beruecksichtigt auch Ecken).
        float rayExitDistance(float px, float py, float dirDeg)
        {
            float dx, dy;
            dirFromCompass(dirDeg, dx, dy);

            float tx = INFINITY;
            if (dx > 1e-5f)       tx = (s_cfg.fieldWidth_cm - px) / dx;
            else if (dx < -1e-5f) tx = (0.0f - px) / dx;

            float ty = INFINITY;
            if (dy > 1e-5f)       ty = (s_cfg.fieldLength_cm - py) / dy;
            else if (dy < -1e-5f) ty = (0.0f - py) / dy;

            float t = (tx < ty) ? tx : ty;
            return (t > 0.0f) ? t : 0.0f;
        }

        // ── Sensorgeometrie ───────────────────────────────────────────────
        // Ordnet jeden Radarsensor der Achse zu, auf die er (annaehernd)
        // schaut. Zu schraeg stehende Sensoren zaehlen fuer keine Achse.
        void updateSensorGeometry(float headingDeg)
        {
            const float ringCorr = ringCorrectionDeg(headingDeg);

            // Blickrichtungen der 4 Feldachsen im Feld-KS
            const float axisDirDeg[4] = {0.0f, 90.0f, 180.0f, -90.0f};
            const int8_t axisOf[4]    = {AXIS_Y, AXIS_X, AXIS_Y, AXIS_X};
            const int8_t signOf[4]    = {+1, +1, -1, -1};

            for (uint8_t i = 0; i < RADAR_COUNT; i++)
            {
                float facing = wrap180(headingDeg + s_cfg.mountAngleDeg[i] - ringCorr);
                s_status.sensorFacingDeg[i] = facing;

                uint8_t best = 0;
                float bestDiff = 999.0f;
                for (uint8_t a = 0; a < 4; a++)
                {
                    float d = fabsf(wrap180(facing - axisDirDeg[a]));
                    if (d < bestDiff) { bestDiff = d; best = a; }
                }

                if (bestDiff <= s_cfg.maxTiltDeg)
                {
                    s_status.sensorAxis[i] = axisOf[best];
                    s_status.sensorSign[i] = signOf[best];
                }
                else
                {
                    s_status.sensorAxis[i] = -1;
                    s_status.sensorSign[i] = 0;
                }
                s_status.sensorTiltDeg[i] = wrap180(facing - axisDirDeg[best]);
            }
        }

        // Sensor, der auf einer Achse in die gewuenschte Richtung schaut
        // (bei mehreren der am wenigsten schraege).
        int8_t sensorFor(uint8_t axis, int8_t sign)
        {
            int8_t best = -1;
            float bestTilt = 999.0f;
            for (uint8_t i = 0; i < RADAR_COUNT; i++)
            {
                if (s_status.sensorAxis[i] != (int8_t)axis) continue;
                if (s_status.sensorSign[i] != sign) continue;
                float t = fabsf(s_status.sensorTiltDeg[i]);
                if (t < bestTilt) { bestTilt = t; best = i; }
            }
            return best;
        }

        // ── Kandidaten & Paare ────────────────────────────────────────────
        uint8_t collectCandidates(int8_t sensor, uint8_t axis, int8_t sign,
                                  const RadarReading radar[RADAR_COUNT],
                                  Candidate out[RADAR_MAX_PEAKS])
        {
            if (sensor < 0) return 0;

            const RadarReading &r = radar[sensor];
            if (!r.valid) return 0;

            const float extent = axisExtent(axis);
            const float cosTilt = cosf(s_status.sensorTiltDeg[sensor] * DEG_TO_RAD);

            uint8_t n = 0;
            uint8_t count = r.peakCount;
            if (count > RADAR_MAX_PEAKS) count = RADAR_MAX_PEAKS;

            for (uint8_t p = 0; p < count; p++)
            {
                const RadarPeak &pk = r.peaks[p];
                if (pk.distance_cm < s_cfg.minWallDist_cm) continue;
                if (s_cfg.minPeakStrength > 0 && pk.strength < s_cfg.minPeakStrength) continue;

                float dCenter = pk.distance_cm + s_cfg.radarRingRadius_cm;
                if (!s_cfg.perpendicularModel) dCenter *= cosTilt;

                float coord = (sign > 0) ? (extent - dCenter) : dCenter;
                // Ein Peak, der eine Position ausserhalb des Feldes bedeuten
                // wuerde, kann nicht die Wand dieser Achse sein.
                if (coord < -s_cfg.outOfField_cm) continue;
                if (coord > extent + s_cfg.outOfField_cm) continue;

                out[n].peakIdx  = (int8_t)p;
                out[n].dCenter  = dCenter;
                out[n].coord    = coord;
                out[n].strength = pk.strength;
                n++;
            }
            return n;
        }

        // Fuellt Kandidaten beider Sensoren einer Achse und sucht alle
        // Peak-Paare, deren Summe dem Feldmass entspricht.
        void buildAxisWork(uint8_t axis, const RadarReading radar[RADAR_COUNT])
        {
            AxisWork &w = s_work[axis];
            w = AxisWork();

            w.sensor[0] = sensorFor(axis, +1);
            w.sensor[1] = sensorFor(axis, -1);

            w.nCand[0] = collectCandidates(w.sensor[0], axis, +1, radar, w.cand[0]);
            w.nCand[1] = collectCandidates(w.sensor[1], axis, -1, radar, w.cand[1]);

            const float extent = axisExtent(axis);

            for (uint8_t a = 0; a < w.nCand[0] && w.nMatch < MAX_MATCHES; a++)
            {
                for (uint8_t b = 0; b < w.nCand[1] && w.nMatch < MAX_MATCHES; b++)
                {
                    float sum = w.cand[0][a].dCenter + w.cand[1][b].dCenter;
                    if (fabsf(sum - extent) > s_cfg.pairTol_cm) continue;

                    Match &m = w.match[w.nMatch++];
                    m.coord    = 0.5f * (w.cand[0][a].coord + w.cand[1][b].coord);
                    m.idxPlus  = w.cand[0][a].peakIdx;
                    m.idxMinus = w.cand[1][b].peakIdx;
                    m.strength = w.cand[0][a].strength + w.cand[1][b].strength;
                }
            }
        }

        // Schritt 1: Achse allein aus dem Radar loesen. Erfolgreich nur, wenn
        // es genau ein (bzw. ein eindeutiges) Wandpaar gibt -- dann wird die
        // Kamera fuer diese Achse gar nicht gebraucht.
        bool resolveAxisRadarOnly(uint8_t axis, AxisResult &res)
        {
            AxisWork &w = s_work[axis];
            res.pairCount = w.nMatch;
            if (w.nMatch == 0) return false;

            float lo = w.match[0].coord, hi = w.match[0].coord;
            float sum = 0.0f;
            int32_t bestStrength = -2147483647;
            uint8_t bestIdx = 0;

            for (uint8_t i = 0; i < w.nMatch; i++)
            {
                float c = w.match[i].coord;
                if (c < lo) lo = c;
                if (c > hi) hi = c;
                sum += c;
                if (w.match[i].strength > bestStrength)
                {
                    bestStrength = w.match[i].strength;
                    bestIdx = i;
                }
            }

            // Mehrere Paare, die auf (fast) dieselbe Position zeigen, sind
            // kein Widerspruch -> weiterhin eindeutig.
            if ((hi - lo) > s_cfg.pairAgree_cm)
            {
                res.ambiguous = true;
                return false;
            }

            res.valid     = true;
            res.coord_cm  = sum / w.nMatch;
            res.source    = AxisSource::RadarPair;
            res.camUsed   = false;
            res.ambiguous = false;
            res.wallPeak[0] = w.match[bestIdx].idxPlus;
            res.wallPeak[1] = w.match[bestIdx].idxMinus;
            return true;
        }

        // Schritt 2: Achse mit Referenz (Kamera oder letzte Position) loesen.
        // Erst die Paare, dann -- falls kein Paar passt, z.B. weil ein Sensor
        // verdeckt ist -- der beste Einzelwert.
        bool resolveAxisWithRef(uint8_t axis, AxisResult &res, float ref, float tol, bool refIsCam)
        {
            AxisWork &w = s_work[axis];

            // 2a) mehrdeutige Paare: das nehmen, das zur Referenz passt
            int8_t best = -1;
            float bestErr = tol;
            for (uint8_t i = 0; i < w.nMatch; i++)
            {
                float err = fabsf(w.match[i].coord - ref);
                if (err <= bestErr) { bestErr = err; best = (int8_t)i; }
            }
            if (best >= 0)
            {
                res.valid     = true;
                res.coord_cm  = w.match[best].coord;
                res.source    = AxisSource::RadarPair;
                res.camUsed   = refIsCam;
                res.ambiguous = (w.nMatch > 1);
                res.wallPeak[0] = w.match[best].idxPlus;
                res.wallPeak[1] = w.match[best].idxMinus;
                return true;
            }

            // 2b) kein Paar -> Einzelsensor. Genau der Fall "ein Sensor ist
            // verdeckt": der andere Wert wird genommen, wenn er grob zur
            // Referenz passt.
            int8_t bestSide = -1, bestCand = -1;
            bestErr = tol;
            for (uint8_t s = 0; s < 2; s++)
            {
                for (uint8_t c = 0; c < w.nCand[s]; c++)
                {
                    float err = fabsf(w.cand[s][c].coord - ref);
                    if (err <= bestErr)
                    {
                        bestErr  = err;
                        bestSide = (int8_t)s;
                        bestCand = (int8_t)c;
                    }
                }
            }
            if (bestSide >= 0)
            {
                res.valid     = true;
                res.coord_cm  = w.cand[bestSide][bestCand].coord;
                res.source    = AxisSource::RadarSingle;
                res.camUsed   = refIsCam;
                res.ambiguous = (w.nCand[0] + w.nCand[1]) > 1;
                res.wallPeak[0] = (bestSide == 0) ? w.cand[0][bestCand].peakIdx : -1;
                res.wallPeak[1] = (bestSide == 1) ? w.cand[1][bestCand].peakIdx : -1;
                return true;
            }

            return false;
        }

        // Notnagel: weder Kamera noch letzte Position vorhanden (erster Zyklus
        // nach dem Einschalten). Dann gilt der staerkste plausible Peak als
        // Wand -- eine Wand reflektiert deutlich staerker als ein Roboter.
        bool resolveAxisNoRef(uint8_t axis, AxisResult &res)
        {
            AxisWork &w = s_work[axis];

            if (w.nMatch > 0)
            {
                uint8_t bestIdx = 0;
                for (uint8_t i = 1; i < w.nMatch; i++)
                    if (w.match[i].strength > w.match[bestIdx].strength) bestIdx = i;

                res.valid     = true;
                res.coord_cm  = w.match[bestIdx].coord;
                res.source    = AxisSource::RadarPair;
                res.ambiguous = (w.nMatch > 1);
                res.wallPeak[0] = w.match[bestIdx].idxPlus;
                res.wallPeak[1] = w.match[bestIdx].idxMinus;
                return true;
            }

            int8_t bestSide = -1, bestCand = -1;
            int32_t bestStrength = -2147483647;
            for (uint8_t s = 0; s < 2; s++)
            {
                for (uint8_t c = 0; c < w.nCand[s]; c++)
                {
                    if (w.cand[s][c].strength > bestStrength)
                    {
                        bestStrength = w.cand[s][c].strength;
                        bestSide = (int8_t)s;
                        bestCand = (int8_t)c;
                    }
                }
            }
            if (bestSide < 0) return false;

            res.valid     = true;
            res.coord_cm  = w.cand[bestSide][bestCand].coord;
            res.source    = AxisSource::RadarSingle;
            res.ambiguous = true;
            res.wallPeak[0] = (bestSide == 0) ? w.cand[0][bestCand].peakIdx : -1;
            res.wallPeak[1] = (bestSide == 1) ? w.cand[1][bestCand].peakIdx : -1;
            return true;
        }

        // ── Kamera ────────────────────────────────────────────────────────
        bool camUsable(const CamReading &cam)
        {
            return cam.valid && cam.blobHeight_px >= s_cfg.camMinBlobHeight_px;
        }

        // Schnittpunkt der beiden Torpeilungen.
        bool triangulate(const CamReading &camOpp, const CamReading &camOwn,
                         float &outX, float &outY, float &outQuality)
        {
            if (!camUsable(camOpp) || !camUsable(camOwn)) return false;

            const float goalX = s_cfg.fieldWidth_cm * 0.5f;
            const float A[2] = {goalX, s_cfg.fieldLength_cm}; // gegnerisches Tor
            const float B[2] = {goalX, 0.0f};                 // eigenes Tor

            float d1x, d1y, d2x, d2y;
            dirFromCompass(camOpp.angle_field_deg, d1x, d1y); // Roboter -> Tor A
            dirFromCompass(camOwn.angle_field_deg, d2x, d2y); // Roboter -> Tor B

            // Roboter = A - t1*d1 = B - t2*d2
            float denom = d1x * d2y - d1y * d2x; // == sin(Winkel zwischen den Peilungen)
            outQuality = fabsf(denom);
            if (outQuality < s_cfg.camMinQuality) return false; // Peilungen zu parallel

            float t1 = ((A[0] - B[0]) * d2y - (A[1] - B[1]) * d2x) / denom;
            if (t1 <= 0.0f) return false; // Tor laege hinter der Peilung

            outX = A[0] - t1 * d1x;
            outY = A[1] - t1 * d1y;
            return true;
        }

        // Aus einer einzelnen Peilung und einer bereits bekannten Achse die
        // andere Achse bestimmen (z.B. x vom Radar -> y aus der Torpeilung).
        bool axisFromBearing(const CamReading &cam, float goalX, float goalY,
                             uint8_t solveAxis, float knownCoord, float &out)
        {
            if (!camUsable(cam)) return false;

            float dx, dy;
            dirFromCompass(cam.angle_field_deg, dx, dy);

            if (solveAxis == AXIS_X)
            {
                if (fabsf(dy) < s_cfg.camMinAxisSin) return false;
                float t = (goalY - knownCoord) / dy;
                if (t <= 0.0f) return false;
                out = goalX - t * dx;
            }
            else
            {
                if (fabsf(dx) < s_cfg.camMinAxisSin) return false;
                float t = (goalX - knownCoord) / dx;
                if (t <= 0.0f) return false;
                out = goalY - t * dy;
            }
            return true;
        }

        // Kamerareferenz aufbauen -- wird nur aufgerufen, wenn mindestens eine
        // Achse aus dem Radar heraus nicht eindeutig war.
        CamRef buildCamRef(const CamReading &camOpp, const CamReading &camOwn,
                           const AxisResult axis[AXIS_COUNT])
        {
            CamRef ref;
            s_status.camEvaluated = true;

            float cx = 0.0f, cy = 0.0f, q = 0.0f;
            if (triangulate(camOpp, camOwn, cx, cy, q))
            {
                ref.has[AXIS_X] = ref.has[AXIS_Y] = true;
                ref.coord[AXIS_X] = cx;
                ref.coord[AXIS_Y] = cy;
                ref.quality = q;
                ref.triangulated = true;
                return ref;
            }
            ref.quality = q;

            // Keine saubere Triangulation (z.B. Roboter auf der Mittellinie,
            // beide Tore in einer Flucht). Ist eine Achse schon vom Radar
            // bekannt, reicht eine einzelne Torpeilung fuer die andere.
            const float goalX = s_cfg.fieldWidth_cm * 0.5f;
            for (uint8_t a = 0; a < AXIS_COUNT; a++)
            {
                uint8_t other = (a == AXIS_X) ? AXIS_Y : AXIS_X;
                if (axis[a].valid) continue;   // Achse braucht keine Referenz
                if (!axis[other].valid) continue; // ohne bekannte Achse nicht loesbar

                float sum = 0.0f;
                uint8_t n = 0;
                float v;
                if (axisFromBearing(camOpp, goalX, s_cfg.fieldLength_cm, a, axis[other].coord_cm, v))
                {
                    sum += v; n++;
                }
                if (axisFromBearing(camOwn, goalX, 0.0f, a, axis[other].coord_cm, v))
                {
                    sum += v; n++;
                }
                if (n > 0)
                {
                    ref.has[a]   = true;
                    ref.coord[a] = sum / n;
                }
            }
            return ref;
        }

        // ── Peak-Klassifizierung ──────────────────────────────────────────
        // Nach der Positionsbestimmung: was jeder Sensor eigentlich sieht.
        void classifyPeaks(const RadarReading radar[RADAR_COUNT], float x, float y)
        {
            for (uint8_t i = 0; i < RADAR_COUNT; i++)
            {
                for (uint8_t p = 0; p < RADAR_MAX_PEAKS; p++)
                    s_status.peak[i][p] = PeakType::Unknown;

                const RadarReading &r = radar[i];
                if (!r.valid) continue;

                float dx, dy;
                dirFromCompass(s_status.sensorFacingDeg[i], dx, dy);
                float sx = x + s_cfg.radarRingRadius_cm * dx;
                float sy = y + s_cfg.radarRingRadius_cm * dy;
                float expected = rayExitDistance(sx, sy, s_status.sensorFacingDeg[i]);

                uint8_t count = r.peakCount;
                if (count > RADAR_MAX_PEAKS) count = RADAR_MAX_PEAKS;

                for (uint8_t p = 0; p < count; p++)
                {
                    float d = r.peaks[p].distance_cm;
                    if (d < 0.0f) continue;
                    if (fabsf(d - expected) <= s_cfg.wallTol_cm) s_status.peak[i][p] = PeakType::Wall;
                    else if (d < expected)                       s_status.peak[i][p] = PeakType::Robot;
                    else                                         s_status.peak[i][p] = PeakType::Behind;
                }
            }
        }

        float axisConfidence(const AxisResult &res)
        {
            if (!res.valid) return 0.0f;
            switch (res.source)
            {
            case AxisSource::RadarPair:   return res.ambiguous ? 0.40f : 0.50f;
            case AxisSource::RadarSingle: return 0.30f;
            case AxisSource::Camera:      return 0.15f;
            case AxisSource::Hold:        return 0.05f;
            default:                      return 0.0f;
            }
        }

    } // namespace

    // ══════════════════════════════════════════════════════════════════════
    //  API
    // ══════════════════════════════════════════════════════════════════════

    void begin(const Config &cfg)
    {
        s_cfg = cfg;
        s_pos = position();
        s_status = Status();
        s_initialized = false;
    }

    void reset(float x_cm, float y_cm)
    {
        s_pos.x = x_cm;
        s_pos.y = y_cm;
        s_pos.x_coord = snapToGrid(x_cm);
        s_pos.y_coord = snapToGrid(y_cm);
        s_initialized = true;
    }

    position getPosition() { return s_pos; }

    const Status &getStatus() { return s_status; }

    const Config &getConfig() { return s_cfg; }

    PeakType getPeakType(uint8_t sensor, uint8_t peakIdx)
    {
        if (sensor >= RADAR_COUNT || peakIdx >= RADAR_MAX_PEAKS) return PeakType::Unknown;
        return s_status.peak[sensor][peakIdx];
    }

    float radarServoAngle(float headingDeg)
    {
        if (!s_cfg.hasRadarServo) return s_cfg.servoNeutralDeg;
        float m = ringCorrectionDeg(headingDeg);
        float out = s_cfg.servoNeutralDeg - m;
        return constrain(out,
                         s_cfg.servoNeutralDeg - s_cfg.servoRangeDeg,
                         s_cfg.servoNeutralDeg + s_cfg.servoRangeDeg);
    }

    float rawCountToCm(uint8_t raw)
    {
        return s_cfg.rawOffset_cm + (float)raw * s_cfg.rawScale_cm;
    }

    position update(float headingDeg,
                    const CamReading &camOpponent,
                    const CamReading &camOwn,
                    const RadarReading radar[RADAR_COUNT])
    {
        // Diagnose zuruecksetzen, Zustand (s_pos, s_initialized) bleibt
        s_status.camEvaluated    = false;
        s_status.camValid        = false;
        s_status.camTriangulated = false;
        s_status.camQuality      = 0.0f;
        s_status.camX            = s_pos.x; // bleibt stehen, solange die Kamera
        s_status.camY            = s_pos.y; // gar nicht gebraucht wird

        for (uint8_t a = 0; a < AXIS_COUNT; a++) s_status.axis[a] = AxisResult();

        // ── 1. Welcher Sensor schaut auf welche Wand ──────────────────────
        updateSensorGeometry(headingDeg);

        // ── 2. Radar zuerst: eindeutiges Wandpaar je Achse? ───────────────
        for (uint8_t a = 0; a < AXIS_COUNT; a++)
        {
            buildAxisWork(a, radar);
            resolveAxisRadarOnly(a, s_status.axis[a]);
        }

        // ── 3. Kamera nur, wenn das Radar nicht eindeutig war ─────────────
        CamRef cam;
        if (!s_status.axis[AXIS_X].valid || !s_status.axis[AXIS_Y].valid)
        {
            cam = buildCamRef(camOpponent, camOwn, s_status.axis);

            s_status.camQuality      = cam.quality;
            s_status.camTriangulated = cam.triangulated;
            s_status.camValid        = cam.has[AXIS_X] || cam.has[AXIS_Y];
            s_status.camX = cam.has[AXIS_X] ? cam.coord[AXIS_X] : s_pos.x;
            s_status.camY = cam.has[AXIS_Y] ? cam.coord[AXIS_Y] : s_pos.y;

            // ── 4. Offene Achsen mit Referenz aufloesen ───────────────────
            for (uint8_t a = 0; a < AXIS_COUNT; a++)
            {
                AxisResult &res = s_status.axis[a];
                if (res.valid) continue;

                bool done = false;
                if (cam.has[a])
                    done = resolveAxisWithRef(a, res, cam.coord[a], s_cfg.camTol_cm, true);

                // Ohne Kamera: die letzte bekannte Position als Referenz.
                if (!done && s_initialized)
                {
                    float last = (a == AXIS_X) ? s_pos.x : s_pos.y;
                    done = resolveAxisWithRef(a, res, last, s_cfg.holdTol_cm, false);
                }

                // Weder Kamera noch Vorgeschichte -> staerksten Peak nehmen.
                if (!done && !cam.has[a] && !s_initialized)
                    done = resolveAxisNoRef(a, res);

                // Immer noch nichts vom Radar -> Kamera bzw. halten.
                if (!done)
                {
                    if (cam.has[a])
                    {
                        res.valid    = true;
                        res.coord_cm = cam.coord[a];
                        res.source   = AxisSource::Camera;
                        res.camUsed  = true;
                    }
                    else if (s_initialized)
                    {
                        res.valid    = true;
                        res.coord_cm = (a == AXIS_X) ? s_pos.x : s_pos.y;
                        res.source   = AxisSource::Hold;
                    }
                    else
                    {
                        res.valid    = true;
                        res.coord_cm = 0.5f * axisExtent(a); // Feldmitte als Startwert
                        res.source   = AxisSource::None;
                    }
                }
            }
        }

        // ── 5. Ergebnis zusammensetzen ────────────────────────────────────
        float newX = s_status.axis[AXIS_X].coord_cm;
        float newY = s_status.axis[AXIS_Y].coord_cm;

        if (s_initialized && s_cfg.outputAlpha < 1.0f)
        {
            float alpha = constrain(s_cfg.outputAlpha, 0.0f, 1.0f);
            newX = s_pos.x + alpha * (newX - s_pos.x);
            newY = s_pos.y + alpha * (newY - s_pos.y);
        }

        newX = constrain(newX, 0.0f, s_cfg.fieldWidth_cm);
        newY = constrain(newY, 0.0f, s_cfg.fieldLength_cm);

        s_pos.x = newX;
        s_pos.y = newY;
        s_pos.x_coord = snapToGrid(newX);
        s_pos.y_coord = snapToGrid(newY);
        s_initialized = true;

        // ── 6. Peaks klassifizieren (Wand / Roboter) ──────────────────────
        classifyPeaks(radar, newX, newY);

        s_status.confidence = constrain(axisConfidence(s_status.axis[AXIS_X]) +
                                            axisConfidence(s_status.axis[AXIS_Y]),
                                        0.0f, 1.0f);

        return s_pos;
    }

} // namespace Positioning
