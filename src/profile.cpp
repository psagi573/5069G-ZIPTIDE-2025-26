#include "profile.h"
#include <cmath>

Profile::Profile(double maxVelocity, double acceleration)
{
    maxVel = maxVelocity;
    accel = acceleration;
}

double Profile::getTargetVelocity(double distRemaining, double distTraveled, double direction = 1.0)
{
    // Avoid instability when very small
    if (distTraveled < 0.5)
        distTraveled = 1;
    if (distRemaining < 0.001)
        distRemaining = 0;

    // Compute speed based on acceleration limit
    double accelLimit = sqrt(2 * accel * distTraveled);  // speed ramp up
    double decelLimit = sqrt(2 * accel * distRemaining); // speed ramp down

    // Target velocity is the lowest of the three
    double target = std::min({accelLimit, decelLimit, maxVel});

    return target * direction;
}

void Profile::setMaxVelocity(double velocity)
{
    maxVel = velocity;
}

void Profile::setAcceleration(double acceleration)
{
    accel = acceleration;
}

void Profile::reset(double velocity, double acceleration)
{
    maxVel = velocity;
    accel = acceleration;
}

/*
// S-Curve Motion Profile
class SCurveProfile {
private:
    double maxVel;
    double accel;
    double jerk;
    double prevVel;

public:
    SCurveProfile(double maxVelocity, double acceleration, double jerkRate)
        : maxVel(maxVelocity), accel(acceleration), jerk(jerkRate), prevVel(0.0) {}

    void reset() {
        prevVel = 0.0;
    }

    double getTargetVelocity(double distRemaining, double dt, double direction = 1.0) {
        // Approximate jerk limited ramp-up
        double targetAccel = accel;
        double nextVel = prevVel + targetAccel * dt;

        // Compute stop velocity (as in trap profile)
        double stopVel = sqrt(2 * accel * distRemaining);
        if (stopVel < nextVel) {
            // Decelerate
            nextVel = std::max(nextVel - jerk * dt, stopVel);
        } else {
            // Accelerate
            nextVel = std::min(nextVel + jerk * dt, maxVel);
        }

        nextVel = std::clamp(nextVel, 0.0, maxVel);
        prevVel = nextVel;
        return nextVel * direction;
    }
};

/* -------------------------
 Comparison Summary:
 -------------------------
 Trapezoidal Profile:
   - Fast and simple
   - Good for short, sharp moves
   - Can cause jerky transitions (wheel slip if accel too high)

 S-Curve Profile:
   - Slower to reach top speed, smoother transitions
   - Avoids sudden jerk (acceleration changes gradually)
   - Better for heavy robots or sensitive control
*/
