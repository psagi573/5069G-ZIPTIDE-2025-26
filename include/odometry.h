#pragma once
#include "vex.h"

/**
 * @brief Represents the robot's position and orientation (pose).
 * x: Global X position (inches)
 * y: Global Y position (inches)
 * theta: Global heading (degrees, 0-360)
 */
struct Pose {
    double x;
    double y;
    double theta;
};

/**
 * @brief Odometry namespace for tracking the robot's pose.
 * Uses 6-motor differential drive encoders and TWO Inertial Sensors with fusion.
 */
namespace Odom {

    /**
     * @brief Starts the odometry tracking thread.
     * Calibrates BOTH IMUs and resets all positions.
     * @param left The motor_group for the left side of the drivetrain.
     * @param right The motor_group for the right side of the drivetrain.
     * @param imu1 The first inertial sensor.
     */
    void start(vex::motor_group& left, vex::motor_group& right, vex::inertial& imu1);

    /**
     * @brief Stops the odometry tracking thread.
     */
    void stop();

    /**
     * @brief Gets the current thread-safe pose of the robot.
     * @return The current global pose (x, y, theta).
     */
    Pose getPose();

    /**
     * @brief Sets the current global pose of the robot and resets sensors.
     * @param x The new global X position.
     * @param y The new global Y position.
     * @param theta The new global heading (degrees).
     */
    void setPose(double x, double y, double theta);

    /**
     * @brief Sets the current global pose of the robot using a Pose struct.
     * @param pose The new global pose.
     */
    void setPose(Pose pose);

} 














// // odom.h
// #pragma once
// #include "vex.h"

// struct Pose {
//     double x, y, theta;
//     Pose(double x = 0, double y = 0, double theta = 0) : x(x), y(y), theta(theta) {}
// };

// // External declarations for global functions
// extern Pose currentPose;

// // Core odometry functions
// bool startOdom(rotation &xSensor, rotation &ySensor, inertial &imu);
// void stopOdom();
// void resetOdom();
// Pose getPose();
// void setCurrentPose(double x, double y, double theta);

// // Utility functions
// void printPose();
// void getRawSensorReadings(double &xInches, double &yInches, double &headingDeg);
// void tuneOffsets(double newXOffset, double newYOffset);
// void setWheelDiameter(double newDiameter);

// // Advanced functions for specific starting positions
// bool startOdomAt(rotation &xSensor, rotation &ySensor, inertial &imu, 
//                  double startX, doubl