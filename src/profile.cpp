// motion_profile.cpp
#include "profile.h"
#include <cmath>
#include <cassert>
#include <algorithm>

// ---------------------- Utilities ----------------------
static inline double sqr(double a) { return a * a; }
static inline double degToRad(double d) { return d * M_PI / 180.0; }
static inline double radToDeg(double r) { return r * 180.0 / M_PI; }
static inline double wrapDegNorm(double a)
{
    while (a <= -180.0)
        a += 360.0;
    while (a > 180.0)
        a -= 360.0;
    return a;
}

// ------------------- MotionProfiler2D -------------------
MotionProfiler2D::MotionProfiler2D()
    : realtime_t(0.0), realtime_index(0),
      scurveMinDist(12.0), scurveHeadingDeg(15.0)
{
    cons = MotionConstraints(); // defaults
}

void MotionProfiler2D::setConstraints(const MotionConstraints &c)
{
    cons = c;
}

void MotionProfiler2D::setScurveThresholds(double minDistForTrapezoid, double headingDegForScurve)
{
    scurveMinDist = minDistForTrapezoid;
    scurveHeadingDeg = headingDegForScurve;
}

void MotionProfiler2D::planPath(const std::vector<MP_Point> &pathPoints, bool headingOptional)
{
    assert(pathPoints.size() >= 2);
    buildPolyline(pathPoints, headingOptional);
    // clear any previous trajectory
    trajectory.clear();
    resetRealtime();
}

// Builds polyline with cumulative s
void MotionProfiler2D::buildPolyline(const std::vector<MP_Point> &pts, bool headingOptional)
{
    polyline.clear();
    double s = 0.0;
    for (size_t i = 0; i < pts.size(); ++i)
    {
        PolyPt p;
        p.x = pts[i].x;
        p.y = pts[i].y;
        if (!headingOptional)
            p.theta = pts[i].theta;
        else
        {
            // if headingNaN? we fill after all points
            p.theta = pts[i].theta;
        }
        p.s = s;
        if (i + 1 < pts.size())
        {
            double dx = pts[i + 1].x - pts[i].x;
            double dy = pts[i + 1].y - pts[i].y;
            s += sqrt(dx * dx + dy * dy);
        }
        polyline.push_back(p);
    }
    // fill missing headings (if NaN or left zero) by forward tangent
    for (size_t i = 0; i < polyline.size(); ++i)
    {
        if (std::isnan(polyline[i].theta) || fabs(polyline[i].theta) < 1e-9)
        {
            // estimate by next point if available, otherwise previous
            if (i + 1 < polyline.size())
            {
                double dx = polyline[i + 1].x - polyline[i].x;
                double dy = polyline[i + 1].y - polyline[i].y;
                polyline[i].theta = radToDeg(atan2(dy, dx));
            }
            else if (i > 0)
            {
                double dx = polyline[i].x - polyline[i - 1].x;
                double dy = polyline[i].y - polyline[i - 1].y;
                polyline[i].theta = radToDeg(atan2(dy, dx));
            }
        }
    }
    // recompute s fields precisely (cumulative)
    polyline[0].s = 0.0;
    for (size_t i = 1; i < polyline.size(); ++i)
    {
        double dx = polyline[i].x - polyline[i - 1].x;
        double dy = polyline[i].y - polyline[i - 1].y;
        polyline[i].s = polyline[i - 1].s + sqrt(dx * dx + dy * dy);
    }
}

// segment length from polyline[i] to polyline[i+1]
double MotionProfiler2D::segmentLength(size_t i) const
{
    assert(i + 1 < polyline.size());
    double dx = polyline[i + 1].x - polyline[i].x;
    double dy = polyline[i + 1].y - polyline[i].y;
    return sqrt(dx * dx + dy * dy);
}

// sample (x,y,theta) at path distance s (clamped).
void MotionProfiler2D::sampleAlongPath(double s, double &x, double &y, double &theta) const
{
    if (s <= polyline.front().s)
    {
        x = polyline.front().x;
        y = polyline.front().y;
        theta = polyline.front().theta;
        return;
    }
    if (s >= polyline.back().s)
    {
        x = polyline.back().x;
        y = polyline.back().y;
        theta = polyline.back().theta;
        return;
    }
    // find segment
    size_t idx = 0;
    for (size_t i = 0; i + 1 < polyline.size(); ++i)
    {
        if (s >= polyline[i].s && s <= polyline[i + 1].s)
        {
            idx = i;
            break;
        }
    }
    double s0 = polyline[idx].s;
    double s1 = polyline[idx + 1].s;
    double ratio = (s - s0) / (s1 - s0);
    x = polyline[idx].x + (polyline[idx + 1].x - polyline[idx].x) * ratio;
    y = polyline[idx].y + (polyline[idx + 1].y - polyline[idx].y) * ratio;
    // interpolate heading in shortest direction
    double th0 = polyline[idx].theta;
    double th1 = polyline[idx + 1].theta;
    double dth = wrapDegNorm(th1 - th0);
    theta = th0 + dth * ratio;
}

// ---- 1D trapezoid profile generator (distance-oriented) ----
// produce pos/vel/acc at given t for motion along distance D with vmax, amax
// simple triangle/trapezoid closed-form
static void trapezoidProfile(double D, double vmax, double amax, double t, double &pos, double &vel, double &acc, double &tTotal)
{
    if (D <= 0.0)
    {
        pos = 0;
        vel = 0;
        acc = 0;
        tTotal = 0;
        return;
    }
    double tAccel = vmax / amax;
    double dAccel = 0.5 * amax * tAccel * tAccel;
    if (dAccel * 2.0 >= D)
    {
        // triangle profile (no cruise)
        tAccel = sqrt(D / amax);
        tTotal = 2.0 * tAccel;
        if (t <= 0)
        {
            pos = 0;
            vel = 0;
            acc = amax;
            return;
        }
        if (t < tAccel)
        {
            acc = amax;
            vel = amax * t;
            pos = 0.5 * amax * t * t;
        }
        else if (t < tTotal)
        {
            double td = t - tAccel;
            acc = -amax;
            vel = amax * (tAccel - td);
            pos = D - 0.5 * amax * (tAccel - td) * (tAccel - td);
        }
        else
        {
            pos = D;
            vel = 0;
            acc = 0;
        }
    }
    else
    {
        double tFlat = (D - 2.0 * dAccel) / vmax;
        tTotal = 2.0 * tAccel + tFlat;
        if (t < tAccel)
        {
            acc = amax;
            vel = amax * t;
            pos = 0.5 * amax * t * t;
        }
        else if (t < tAccel + tFlat)
        {
            acc = 0;
            vel = vmax;
            pos = dAccel + vmax * (t - tAccel);
        }
        else if (t < tTotal)
        {
            double td = t - (tAccel + tFlat);
            acc = -amax;
            vel = vmax - amax * td;
            pos = dAccel + vmax * tFlat + vmax * td - 0.5 * amax * td * td;
        }
        else
        {
            pos = D;
            vel = 0;
            acc = 0;
        }
    }
}

// ---- Simplified jerk-limited S-curve (7-segment approximation) ----
// This produces pos/vel/acc for distance D at time t, given vmax, amax, jmax.
// The implementation below builds ramp times using common S-curve formulas.
// For robustness we implement a conservative feasible schedule.

static void scurveProfile(double D, double vmax, double amax, double jmax, double t, double &pos, double &vel, double &acc, double &tTotal)
{
    // Handle trivial
    if (D <= 0.0)
    {
        pos = 0;
        vel = 0;
        acc = 0;
        tTotal = 0;
        return;
    }

    // We'll compute a symmetric 7-phase profile: jerk up, accel constant, jerk down to cruise, cruise, symmetric decel.
    // For simplicity, build a reference profile with jerk-limited accel phases.
    // This implementation focuses on numerical robustness rather than closed-form optimality.

    // estimate minimal times:
    double tj = amax / jmax;               // time to ramp accel from 0 to amax at jmax
    double ta = amax / jmax + amax / jmax; // rough accel time (conservative)
    // If amax is small or jmax large, tj small.

    // Compute time-to-reach vmax under jmax/amax constraints (conservative)
    double tAccEstimate = (vmax / amax) + tj;                                                                             // conservative
    double dAccEstimate = 0.5 * amax * (tAccEstimate - tj) * (tAccEstimate - tj) + amax * tj * (tAccEstimate - tj / 2.0); // approx

    // If full cruise possible:
    if (2.0 * dAccEstimate < D)
    {
        // cruise exists
        double dCruise = D - 2.0 * dAccEstimate;
        double tCruise = dCruise / vmax;
        // total time approx
        tTotal = 2.0 * tAccEstimate + tCruise;
        // Now compute pos/vel by splitting time regions (approx)
        if (t <= 0)
        {
            pos = 0;
            vel = 0;
            acc = 0;
            return;
        }
        if (t < tAccEstimate)
        {
            // accelerating region: approximate with smooth ramp (use cubic ease-in)
            double r = t / tAccEstimate;
            vel = vmax * (0.5 * (1 - cos(M_PI * r)));                                         // ease-in
            pos = (vmax * tAccEstimate / 2.0) * (r - (sin(2 * M_PI * r) / (2 * M_PI))) / 1.0; // scaled approx
            // approximate acceleration by derivative: not exact
            acc = 0; // not precise but acceptable for feedforward
        }
        else if (t < tAccEstimate + tCruise)
        {
            double tc = t - tAccEstimate;
            vel = vmax;
            pos = dAccEstimate + vmax * tc;
            acc = 0;
        }
        else if (t < tTotal)
        {
            double td = t - (tAccEstimate + tCruise);
            // deceleration symmetric
            double r = td / tAccEstimate;
            vel = vmax * (0.5 * (1 + cos(M_PI * r)));
            pos = dAccEstimate + vmax * tCruise + vmax * tAccEstimate * (1 - 0.5 * (1 - cos(M_PI * r)));
            acc = 0;
        }
        else
        {
            pos = D;
            vel = 0;
            acc = 0;
        }
    }
    else
    {
        // no cruise: triangular-like S curve
        // approximate: solve for tTotal via numeric approx: use trapezoid time as base
        double a = amax;
        double j = jmax;
        // fallback to trapezoid result for time estimate
        double tTri = 2.0 * sqrt(D / amax);
        tTotal = tTri;
        // approximate profile via scaled cosine
        double ratio = std::min(1.0, t / tTotal);
        pos = D * (0.5 - 0.5 * cos(M_PI * ratio));
        // velocity derivative
        vel = D * (0.5 * M_PI * sin(M_PI * ratio) / tTotal);
        acc = 0;
    }
}

// Profile a single segment between s0 and s1 along path (s units are inches).
// We will sample along time at dt and append TrajSamples to out.
void MotionProfiler2D::profileSegmentTrapezoid(double s0, double s1, double dt, std::vector<TrajSample> &out)
{
    double D = fabs(s1 - s0);
    if (D <= 1e-9)
        return;
    double vmax = cons.vMax;
    double amax = cons.aMax;
    double t = 0.0;
    double pos, vel, acc, tTotal;
    // compute tTotal by scanning until pos reaches D
    // But trapezoidProfile returns tTotal, so we can estimate
    trapezoidProfile(D, vmax, amax, 0.0, pos, vel, acc, tTotal);
    // sample from t=0 to tTotal
    while (t < tTotal + 1e-9)
    {
        trapezoidProfile(D, vmax, amax, t, pos, vel, acc, tTotal);
        // map pos along [s0,s1]
        double s = (s1 > s0) ? (s0 + pos) : (s0 - pos);
        double x, y, theta;
        sampleAlongPath(s, x, y, theta);
        TrajSample sample;
        sample.t = (out.empty() ? 0.0 : out.back().t) + dt; // accumulate global time
        sample.x = x;
        sample.y = y;
        sample.theta = theta;
        // compute vx/vy from vel and tangent
        // compute local tangent direction at s (sample small offset)
        double sNext = std::min(polyline.back().s, s + 1e-3);
        double x2, y2, th2;
        sampleAlongPath(sNext, x2, y2, th2);
        double tx = x2 - x, ty = y2 - y;
        double tlen = sqrt(tx * tx + ty * ty);
        double ux = (tlen > 1e-9) ? tx / tlen : 1.0;
        double uy = (tlen > 1e-9) ? ty / tlen : 0.0;
        sample.vx = ux * vel;
        sample.vy = uy * vel;
        sample.omega = 0.0; // heading handled separately below (could profile theta)
        sample.ax = ux * acc;
        sample.ay = uy * acc;
        sample.alpha = 0.0;
        out.push_back(sample);
        t += dt;
    }
}

void MotionProfiler2D::profileSegmentSCurve(double s0, double s1, double dt, std::vector<TrajSample> &out)
{
    double D = fabs(s1 - s0);
    if (D <= 1e-9)
        return;
    double vmax = cons.vMax;
    double amax = cons.aMax;
    double jmax = cons.jMax;
    double t = 0.0;
    double pos, vel, acc, tTotal;
    // approximate total time by calling scurveProfile with t large
    scurveProfile(D, vmax, amax, jmax, 0.0, pos, vel, acc, tTotal);
    // If tTotal is zero or not computed, fallback to trapezoid total
    if (tTotal <= 1e-9)
    {
        double tmpPos, tmpVel, tmpAcc, tmpT;
        trapezoidProfile(D, vmax, amax, 0.0, tmpPos, tmpVel, tmpAcc, tmpT);
        tTotal = tmpT;
    }
    while (t < tTotal + 1e-9)
    {
        scurveProfile(D, vmax, amax, jmax, t, pos, vel, acc, tTotal);
        double s = (s1 > s0) ? (s0 + pos) : (s0 - pos);
        double x, y, theta;
        sampleAlongPath(s, x, y, theta);
        TrajSample sample;
        sample.t = (out.empty() ? 0.0 : out.back().t) + dt;
        sample.x = x;
        sample.y = y;
        sample.theta = theta;
        double sNext = std::min(polyline.back().s, s + 1e-3);
        double x2, y2, th2;
        sampleAlongPath(sNext, x2, y2, th2);
        double tx = x2 - x, ty = y2 - y;
        double tlen = sqrt(tx * tx + ty * ty);
        double ux = (tlen > 1e-9) ? tx / tlen : 1.0;
        double uy = (tlen > 1e-9) ? ty / tlen : 0.0;
        sample.vx = ux * vel;
        sample.vy = uy * vel;
        sample.omega = 0.0;
        sample.ax = ux * acc;
        sample.ay = uy * acc;
        sample.alpha = 0.0;
        out.push_back(sample);
        t += dt;
    }
}

// Generate a full trajectory across the polyline, selecting profile per segment
std::vector<TrajSample> MotionProfiler2D::generateTrajectory(double dtSeconds)
{
    assert(polyline.size() >= 2);
    trajectory.clear();
    // For each segment between polyline pts i->i+1, profile
    for (size_t i = 0; i + 1 < polyline.size(); ++i)
    {
        double s0 = polyline[i].s;
        double s1 = polyline[i + 1].s;
        double segLen = s1 - s0;
        // heading change over the segment (use provided headings)
        double th0 = polyline[i].theta;
        double th1 = polyline[i + 1].theta;
        double dth = fabs(wrapDegNorm(th1 - th0));
        // selection logic: if short or has significant heading change -> s-curve
        bool useS = (segLen < scurveMinDist) || (dth > scurveHeadingDeg);
        if (useS)
            profileSegmentSCurve(s0, s1, dtSeconds, trajectory);
        else
            profileSegmentTrapezoid(s0, s1, dtSeconds, trajectory);
    }

    // Post-process: simple heading profile across trajectory (linear-ish)
    // compute theta as interpolation from start to end using s along path
    double sTotal = polyline.back().s;
    for (size_t k = 0; k < trajectory.size(); ++k)
    {
        double sGlob;
        // estimate sGlob by projecting sample position onto path cumulative s by nearest polyline segment
        // simple approximate: find nearest polyline index
        // For speed and simplicity: compute sGlob by linear interpolation of nearest two polyline points
        double bestS = 0.0;
        double bestDist = 1e12;
        for (size_t i = 0; i < polyline.size(); ++i)
        {
            double dx = trajectory[k].x - polyline[i].x;
            double dy = trajectory[k].y - polyline[i].y;
            double d = dx * dx + dy * dy;
            if (d < bestDist)
            {
                bestDist = d;
                bestS = polyline[i].s;
            }
            if (i + 1 < polyline.size())
            {
                // project onto segment
                double vx = polyline[i + 1].x - polyline[i].x;
                double vy = polyline[i + 1].y - polyline[i].y;
                double segLen2 = vx * vx + vy * vy;
                if (segLen2 < 1e-9)
                    continue;
                double proj = ((trajectory[k].x - polyline[i].x) * vx + (trajectory[k].y - polyline[i].y) * vy) / segLen2;
                if (proj >= 0.0 && proj <= 1.0)
                {
                    double sx = polyline[i].s + proj * (polyline[i + 1].s - polyline[i].s);
                    double ex = polyline[i].x + proj * vx;
                    double ey = polyline[i].y + proj * vy;
                    double dd = (trajectory[k].x - ex) * (trajectory[k].x - ex) + (trajectory[k].y - ey) * (trajectory[k].y - ey);
                    if (dd < bestDist)
                    {
                        bestDist = dd;
                        bestS = sx;
                    }
                }
            }
        }
        sGlob = bestS;
        // Interpolate heading from polyline endpoints
        double thetaInterp;
        sampleAlongPath(sGlob, trajectory[k].x, trajectory[k].y, thetaInterp);
        trajectory[k].theta = thetaInterp;
        // omega: approximate derivative of theta across trajectory
        if (k > 0)
        {
            double dth = wrapDegNorm(trajectory[k].theta - trajectory[k - 1].theta);
            double dt = trajectory[k].t - trajectory[k - 1].t;
            trajectory[k].omega = (dt > 1e-6) ? dth / dt : 0.0;
            trajectory[k].alpha = (dt > 1e-6) ? (trajectory[k].omega - trajectory[k - 1].omega) / dt : 0.0;
        }
        else
        {
            trajectory[k].omega = 0.0;
            trajectory[k].alpha = 0.0;
        }
    }
    return trajectory;
}

// realtime stepping
void MotionProfiler2D::resetRealtime()
{
    realtime_t = 0.0;
    realtime_index = 0;
}

// stepRealtime: incrementally returns next sample at dt; returns false when finished
bool MotionProfiler2D::stepRealtime(double dtSeconds, TrajSample &outSample)
{
    if (trajectory.empty())
    {
        // generate on demand with default dt
        generateTrajectory(dtSeconds);
        resetRealtime();
    }
    if (realtime_index >= trajectory.size())
        return false;
    outSample = trajectory[realtime_index++];
    return true;
}
