#include "vex.h"
#include "odom.h"

using namespace vex;

// Internal storage of the robot's position
Pose currentPose = {0.0, 0.0, 0.0};

// Sensor pointers
rotation *xRot;
rotation *yRot;
inertial *imuSensor;

double prevX = 0;
double prevY = 0;

// Background task to update position
int odomTask()
{
    while (true)
    {
        // Get change in position from X and Y rotation sensors (in degrees)
        double xDeg = xRot->position(degrees);
        double yDeg = yRot->position(degrees);

        // Convert to inches (assuming 3.25" wheels)
        double wheelCircumference = 3.25 * M_PI;
        double xInches = (xDeg - prevX) / 360.0 * wheelCircumference;
        double yInches = (yDeg - prevY) / 360.0 * wheelCircumference;

        prevX = xDeg;
        prevY = yDeg;

        // Get current heading in degrees
        double thetaDeg = imuSensor->rotation();
        double thetaRad = thetaDeg * (M_PI / 180.0);

        // Adjust x and y based on heading
        currentPose.x += xInches * cos(thetaRad) - yInches * sin(thetaRad);
        currentPose.y += xInches * sin(thetaRad) + yInches * cos(thetaRad);
        currentPose.theta = thetaDeg;

        wait(10, msec); // Update every 10ms
    }
    return 0;
}

// Public function to start odometry
void startOdom(rotation &xSensor, rotation &ySensor, inertial &imu)
{
    xRot = &xSensor;
    yRot = &ySensor;
    imuSensor = &imu;

    xRot->resetPosition();
    yRot->resetPosition();
    imuSensor->resetRotation();

    prevX = 0;
    prevY = 0;

    thread odomThread(odomTask);
}

// Returns current robot pose
Pose getPose()
{
    return currentPose;
}