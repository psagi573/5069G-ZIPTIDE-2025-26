#pragma once
#include "vex.h"
#include <cmath>
#include <vector>

namespace Odom {

/**
 * @brief Structure to hold the robot's state.
 */
struct Pose {
    double x = 0.0;     // X position (inches)
    double y = 0.0;     // Y position (inches)
    double theta = 0.0; // Heading (degrees, 0 = forward/+Y)
};

// ------------------------
// Public Odometry Functions
// ------------------------

/**
 * @brief Initializes and starts the Odometry thread.
 * @param leftMotors - The motor group controlling the left side.
 * @param rightMotors - The motor group controlling the right side.
 * @param imu - The Inertial Sensor.
 */
void start(vex::motor_group* leftMotors,
           vex::motor_group* rightMotors,
           vex::inertial* imu);

/**
 * @brief Stops the Odometry thread.
 */
void stop();

/**
 * @brief Gets the current robot pose. Thread-safe access.
 * @return The current Pose (x, y, theta).
 */
Pose getPose();

/**
 * @brief Sets the robot pose to a new value (used for start of autonomous).
 * @param newPose The pose to set (x, y, theta).
 */
void setPose(Pose newPose);

} // namespace Odom