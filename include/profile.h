#include "vex.h"

class Profile
{
private:
    double maxVel; // inches per second
    double accel;  // inches per second squared

public:
    Profile(double maxVelocity, double acceleration);                                      // Constructor to initialize profile parameters
    double getTargetVelocity(double distRemaining, double distTraveled, double direction); // Get target velocity based on remaining and traveled distance
    void setMaxVelocity(double velocity);                                                  // Set maximum velocity for the profile
    void setAcceleration(double acceleration);                                             // Set acceleration limit for the profile
    void reset(double velocity, double acceleration);                                      // Reset profile parameters
};
