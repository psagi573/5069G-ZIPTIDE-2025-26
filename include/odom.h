#pragma once

#include "vex.h"

// Pose struct to represent the robot position and heading
struct Pose
{
    double x;
    double y;
    double theta;   // degrees [0,360)
    double ySensor; // Y-axis tracker position in inches
    double xSensor; // X-axis tracker position in inches
};

// Initialize and start odometry tracking
void startOdom(vex::rotation &xSensor, vex::rotation &ySensor, vex::inertial &imu);
void startOdomAt(vex::rotation &xSensor, vex::rotation &ySensor, vex::inertial &imu, double startX, double startY, double startTheta);
// Stop odometry tracking cleanly
void stopOdom();

// Get the current estimated robot pose (position + heading)
Pose getPose();