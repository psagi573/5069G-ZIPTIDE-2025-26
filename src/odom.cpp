#include "vex.h"
#include "odom.h"

#include "vex.h"
#include "odom.h"

using namespace vex;

// Internal position state
Pose currentPose = {0.0, 0.0, 0.0};

// Sensor pointers
rotation *xRot;
rotation *yRot;
inertial *imuSensor;

// Robot geometry
const double xOffset = 0;   // inches from center to X (lateral) tracker 0 inches
const double yOffset = 1.9; // inches from center to Y (longitudinal) tracker 1.8 inches

// Wheel settings
const double odomWheelDiameter = 2.0;
const double wheelCircumference = odomWheelDiameter * M_PI;

// Previous readings
double prevX = 0;
double prevY = 0;
double prevThetaRad = 0;

int odomTask()
{
    while (true)
    {
        // 1. Get current sensor positions
        double currX = xRot->position(turns);
        double currY = yRot->position(turns);
        double thetaDeg = imuSensor->rotation();
        double thetaRad = thetaDeg * (M_PI / 180.0);

        // 2. Convert encoder readings to inches
        double currXInches = currX * wheelCircumference;
        double currYInches = currY * wheelCircumference;

        double deltaXRaw = currXInches - prevX;
        double deltaYRaw = currYInches - prevY;

        // 3. Calculate heading delta and wrap to [-π, π]
        double deltaTheta = thetaRad - prevThetaRad;
        if (deltaTheta > M_PI)
            deltaTheta -= 2 * M_PI;
        if (deltaTheta < -M_PI)
            deltaTheta += 2 * M_PI;

        // 4. Midpoint angle for rotation correction
        double averageTheta = prevThetaRad + deltaTheta / 2.0;

        // 5. Apply rotational correction for tracker offset
        double deltaXRot = yOffset * deltaTheta;
        double deltaYRot = -xOffset * deltaTheta;

        double deltaXRobot = deltaXRaw - deltaXRot;
        double deltaYRobot = deltaYRaw - deltaYRot;

        // 6. Rotate robot-relative delta into field (global) coordinates
        double cosT = cos(averageTheta);
        double sinT = sin(averageTheta);

        double deltaXGlobal = deltaXRobot * cosT - deltaYRobot * sinT;
        double deltaYGlobal = deltaXRobot * sinT + deltaYRobot * cosT;

        // 7. Update global pose
        currentPose.x += deltaXGlobal;
        currentPose.y += deltaYGlobal;

        // Wrap heading into [0, 360)
        double cleanTheta = fmod(thetaDeg, 360.0);
        if (cleanTheta < 0)
            cleanTheta += 360.0;
        currentPose.theta = cleanTheta;

        // 8. Store for next loop
        prevX = currXInches;
        prevY = currYInches;
        prevThetaRad = thetaRad;

        wait(10, msec);
    }

    return 0;
}

void startOdom(rotation &xSensor, rotation &ySensor, inertial &imu)
{
    xRot = &xSensor;
    yRot = &ySensor;
    imuSensor = &imu;

    xRot->resetPosition();
    yRot->resetPosition();
    imuSensor->resetRotation();

    prevX = xRot->position(degrees) * wheelCircumference / 360.0;
    prevY = yRot->position(degrees) * wheelCircumference / 360.0;
    prevThetaRad = imuSensor->rotation() * (M_PI / 180.0);

    thread odomThread(odomTask);
}

Pose getPose()
{
    return currentPose;
}
