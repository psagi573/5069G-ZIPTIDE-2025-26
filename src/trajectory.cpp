#include "trajectory.h"
#include "utils.h"

void TrajectoryBuilder::addWaypoint(double x, double y, double theta)
{
    splineGen.addPoint(x, y, theta);
}

void TrajectoryBuilder::clearWaypoints()
{
    splineGen.clearPoints();
}

std::vector<TrajSample> TrajectoryBuilder::buildTrajectory(double dtSeconds, int samplesPerSegment)
{
    // 1. Generate smooth spline path points
    std::vector<SplinePoint> pathPoints = splineGen.generatePath(samplesPerSegment);

    // 2. Convert SplinePoint to MP_Point for MotionProfiler2D
    std::vector<MP_Point> mpPoints;
    for (const auto &p : pathPoints)
    {
        mpPoints.push_back({p.x, p.y, p.theta});
    }

    // 3. Feed points into the motion profiler
    profiler.planPath(mpPoints);

    // 4. Generate full trajectory
    return profiler.generateTrajectory(dtSeconds);
}
