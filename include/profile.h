#include "vex.h"

class Profile
{
private:
    double maxVel; // inches per second
    double accel;  // inches per second squared

public:
    Profile(double maxVelocity, double acceleration);
    double getTargetVelocity(double distanceRemaining, double distanceTraveled);
};
