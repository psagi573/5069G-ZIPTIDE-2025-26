/*#pragma once
#include "path.h"
#include "profile.h"
#include <vector>

class TrajectoryBuilder
{
public:
    TrajectoryBuilder() = default;

    // Add a waypoint
    void addWaypoint(double x, double y, double theta = NAN);

    // Clear all waypoints
    void clearWaypoints();

    // Build and return a full MotionProfiler2D trajectory
    std::vector<TrajSample> buildTrajectory(double dtSeconds = 0.01, int samplesPerSegment = 20);

private:
    SplinePath splineGen;
    MotionProfiler2D profiler;
};*/
