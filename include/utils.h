#pragma once
#include <cmath> // for fmod, M_PI

// ----------------- Angle Conversions -----------------
inline double deg2rad(double d) { return d * M_PI / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / M_PI; }
static inline double sqr(double a) { return a * a; }

// ----------------- Angle Wrapping -----------------
inline double wrapRad(double a)
{
    while (a > M_PI)
        a -= 2.0 * M_PI;
    while (a <= -M_PI)
        a += 2.0 * M_PI;
    return a;
}

inline double wrapDeg360(double a)
{
    double v = fmod(a, 360.0);
    if (v < 0)
        v += 360.0;
    return v;
}

static inline double wrapDegNorm(double a)
{
    while (a <= -180.0)
        a += 360.0;
    while (a > 180.0)
        a -= 360.0;
    return a;
}
// ----------------- Angle Difference -----------------
inline double smallestAngleDiffDeg(double to, double from)
{
    double d = to - from;
    while (d > 180.0)
        d -= 360.0;
    while (d <= -180.0)
        d += 360.0;
    return d;
}

// ----------------- Clamp -----------------
inline double clamp(double v, double lo, double hi) { return (v < lo) ? lo : (v > hi) ? hi
                                                                                      : v; }

// ----------------- Template Utilities -----------------
template <typename T>
inline T clampT(T v, T lo, T hi) { return (v < lo) ? lo : (v > hi) ? hi
                                                                   : v; }

template <typename T>
inline int signT(T x) { return (x > 0) - (x < 0); }

template <typename T>
inline void swapT(T &a, T &b)
{
    T temp = a;
    a = b;
    b = temp;
}

// ----------------- Min / Max -----------------
inline double maxDouble(double a, double b) { return (a > b) ? a : b; }
inline double minDouble(double a, double b) { return (a < b) ? a : b; }
inline int maxInt(int a, int b) { return (a > b) ? a : b; }
inline int minInt(int a, int b) { return (a < b) ? a : b; }

// ----------------- Linear Interpolation -----------------
inline double lerp(double a, double b, double t) { return a + t * (b - a); }
