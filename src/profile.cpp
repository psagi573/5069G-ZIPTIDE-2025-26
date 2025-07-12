#include "profile.h"
#include <cmath>

Profile::Profile(double maxVelocity, double acceleration)
{
    maxVel = maxVelocity;
    accel = acceleration;
}

double Profile::getTargetVelocity(double distRemaining, double distTraveled)
{
    // Compute speed based on acceleration limit
    double accelLimit = sqrt(2 * accel * distTraveled);  // speed ramp up
    double decelLimit = sqrt(2 * accel * distRemaining); // speed ramp down

    // Target velocity is the lowest of the three
    double target = std::min({accelLimit, decelLimit, maxVel});
    return target;
}