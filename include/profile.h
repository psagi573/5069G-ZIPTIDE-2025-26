#pragma once

#include <vector>

struct MP_Point
{
    double x;     // inches
    double y;     // inches
    double theta; // degrees (optional, can be NAN to let profiler infer)
    MP_Point() : x(0), y(0), theta(0) {}
    MP_Point(double X, double Y, double T = 0) : x(X), y(Y), theta(T) {}
};

struct TrajSample
{
    double t;     // seconds since start
    double x;     // inches
    double y;     // inches
    double theta; // degrees
    double vx;    // in/s (world-frame)
    double vy;    // in/s (world-frame)
    double omega; // deg/s
    double ax;    // in/s^2
    double ay;    // in/s^2
    double alpha; // deg/s^2
};

struct MotionConstraints
{
    double vMax;  // in/s
    double aMax;  // in/s^2
    double jMax;  // in/s^3
    double wMax;  // deg/s
    double aWMax; // deg/s^2
    double jWMax; // deg/s^3

    MotionConstraints()
        : vMax(24.0), aMax(48.0), jMax(400.0), wMax(360.0), aWMax(720.0), jWMax(3000.0) {}
};

class MotionProfiler2D
{
public:
    // ctor: default constraints can be tuned after
    MotionProfiler2D();

    // Configure constraints
    void setConstraints(const MotionConstraints &c);

    // Plan a path. Path must have at least 2 points.
    // If you want smooth curves, pass in a densely-sampled spline (Hermite) here.
    // headingOptional: if true, the waypoint.theta fields are treated as suggestions (used if finite)
    void planPath(const std::vector<MP_Point> &pathPoints, bool headingOptional = true);

    // Generate a time-parameterized trajectory with given dt.
    // This consumes the path planned earlier and produces a vector<TrajSample>.
    // It performs per-segment profile selection (trapezoid vs S-curve) automatically.
    std::vector<TrajSample> generateTrajectory(double dtSeconds);

    // Real-time stepping API: call after planPath, repeatedly with dt to get live setpoints.
    // Returns false when finished.
    bool stepRealtime(double dtSeconds, TrajSample &outSample);

    // Query final planned trajectory (after generateTrajectory)
    const std::vector<TrajSample> &getTrajectory() const { return trajectory; }

    // Reset internal realtime state (call before stepping)
    void resetRealtime();

    // Tuning helpers
    void setScurveThresholds(double minDistForTrapezoid, double headingDegForScurve);

private:
    MotionConstraints cons;

    // internal representation of the sampled polyline (cumulative s)
    struct PolyPt
    {
        double x, y, theta;
        double s;
    }; // s = distance along path from start
    std::vector<PolyPt> polyline;

    // planned trajectory
    std::vector<TrajSample> trajectory;

    // realtime iteration state
    double realtime_t;
    size_t realtime_index;

    // selection thresholds
    double scurveMinDist;    // if segment shorter than this -> prefer s-curve
    double scurveHeadingDeg; // if heading change > this -> prefer s-curve

    // Helpers (private)
    void buildPolyline(const std::vector<MP_Point> &pts, bool headingOptional);
    double segmentLength(size_t i) const;
    void profileSegmentTrapezoid(double s0, double s1, double dt, std::vector<TrajSample> &out);
    void profileSegmentSCurve(double s0, double s1, double dt, std::vector<TrajSample> &out);
    void sampleAlongPath(double s, double &x, double &y, double &theta) const;
    double clampAngleDeg(double a) const;
};
