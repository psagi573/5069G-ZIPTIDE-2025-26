#pragma once
#include "vex.h"
#include <vector>

namespace Odom {

struct Pose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0; // degrees, 0 = +Y
};

// ------------------------
// Public Odometry Functions
// ------------------------

void start(const std::vector<vex::motor*>& leftMotors,
           const std::vector<vex::motor*>& rightMotors,
           vex::inertial* imu);

void stop();

Pose getPose();
void setPose(double x, double y, double theta);
void setPose(Pose pose);

// ------------------------
// Internal Functions
// ------------------------

void setActiveMotors(const std::vector<bool>& leftActive,
                     const std::vector<bool>& rightActive);

} // namespace Odom
