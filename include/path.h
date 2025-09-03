#pragma once
#include <vector>
#include <cmath>

// Simple point structure for path generation
struct SplinePoint
{
    double x;
    double y;
    double theta; // heading in degrees
};

// Class to generate smooth cubic spline paths
class SplinePath
{
public:
    SplinePath() = default;

    // Add control points (waypoints)
    void addPoint(double x, double y, double theta = NAN);

    // Clear all points
    void clearPoints();

    // Generate path with given number of samples per segment
    std::vector<SplinePoint> generatePath(int samplesPerSegment = 20);

private:
    std::vector<SplinePoint> controlPoints;

    // Compute natural cubic spline coefficients
    void computeNaturalCubic(const std::vector<double> &positions,
                             std::vector<double> &a,
                             std::vector<double> &b,
                             std::vector<double> &c,
                             std::vector<double> &d);
};
