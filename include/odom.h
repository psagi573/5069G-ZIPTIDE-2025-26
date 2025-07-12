#include "vex.h"

struct Pose
{
    double x;     // horazontal position in inches
    double y;     // vertical position in inches
    double theta; // rotational position in degrees
};

void startOdom(vex::rotation &xSensor, vex::rotation &ySensor, vex::inertial &imu);
Pose getPose();