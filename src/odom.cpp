#include "vex.h"
#include "odom.h"

#include "vex.h"
#include "odom.h"

using namespace vex;

// Internal position state
Pose currentPose = {0.0, 0.0, 0.0};

// Odometer thread
thread odomThread;
bool odomRunning = false;

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
    while (odomRunning)
    {
        // 1. Get current sensor positions
        double currXTurns = xRot->position(turns);
        double currYTurns = yRot->position(turns);
        double thetaDeg = imuSensor->rotation();
        double thetaRad = thetaDeg * (M_PI / 180.0);

        // 2. Convert encoder readings to inches
        double currXInches = currXTurns * wheelCircumference;
        double currYInches = currYTurns * wheelCircumference;

        // 3. Calculate raw deltas in robot-local coordinates
        double deltaX = currXInches - prevX;
        double deltaY = currYInches - prevY;

        // 4. Calculate heading delta and wrap to [-π, π]
        double deltaTheta = thetaRad - prevThetaRad;
        if (deltaTheta > M_PI)
            deltaTheta -= 2.0 * M_PI;
        if (deltaTheta < -M_PI)
            deltaTheta += 2.0 * M_PI;

        // 5. Compute local displacement with arc correction (Purdue method)
        double localX = 0.0, localY = 0.0;

        if (fabs(deltaTheta) < 1e-6)
        {
            // Robot moved nearly straight: no arc correction needed
            localX = deltaX;
            localY = deltaY;
        }
        else
        {
            // Arc-based displacement calculation with tracker offsets
            double rFactor = 2.0 * sin(deltaTheta / 2.0) / deltaTheta;
            localX = rFactor * (deltaX + xOffset * deltaTheta);
            localY = rFactor * (deltaY + yOffset * deltaTheta);
        }

        // 6. Rotate local displacement to global coordinates using average heading
        double avgTheta = prevThetaRad + deltaTheta / 2.0;
        double cosT = cos(avgTheta);
        double sinT = sin(avgTheta);

        double deltaXGlobal = localX * cosT - localY * sinT;
        double deltaYGlobal = localX * sinT + localY * cosT;

        // 7. Update global pose with thread-safe mutex lock
        {
            currentPose.x += deltaXGlobal;
            currentPose.y += deltaYGlobal;

            double cleanTheta = fmod(thetaDeg, 360.0);
            if (cleanTheta < 0.0)
                cleanTheta += 360.0;
            currentPose.theta = cleanTheta;
        }

        // 8. Store current readings for next iteration
        prevX = currXInches;
        prevY = currYInches;
        prevThetaRad = thetaRad;

        wait(10, msec); // Update at 100Hz or adjust as needed
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

    prevX = xRot->position(turns) * wheelCircumference;
    prevY = yRot->position(turns) * wheelCircumference;
    prevThetaRad = imuSensor->rotation() * (M_PI / 180.0);

    odomRunning = true;
    odomThread = thread(odomTask);

    wait(10, msec);
}

void stopOdom()
{
    if (odomRunning)
    {
        odomRunning = false;
        odomThread.join();
    }
}

Pose getPose()
{
    Pose poseCopy;
    // Copy the struct fields (atomic on simple doubles generally safe)
    poseCopy.x = currentPose.x;
    poseCopy.y = currentPose.y;
    poseCopy.theta = currentPose.theta;
    return poseCopy;
}

/*
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
*/
