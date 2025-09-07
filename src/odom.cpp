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

// Robot senser offsets
const double xOffset = 2;  // inches from center to X (lateral) tracker 0 inches
const double yOffset = -2; // inches from center to Y (longitudinal) tracker 1.9 inches

// Wheel settings
const double odomWheelDiameter = 2.0;
const double wheelCircumference = odomWheelDiameter * M_PI;

// Previous readings
double prevX = 0;
double prevY = 0;
double prevThetaRad = 0;

// Odometer update task
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

        wait(10, msec); // Update at 100Hz
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

// New function: startOdomAt with custom starting position
void startOdomAt(rotation &xSensor, rotation &ySensor, inertial &imu, double startX, double startY, double startTheta)
{
    xRot = &xSensor;
    yRot = &ySensor;
    imuSensor = &imu;

    xRot->resetPosition();
    yRot->resetPosition();
    imuSensor->resetRotation();

    // Initialize the pose to the specified parameters
    currentPose.x = startX;
    currentPose.y = startY;
    currentPose.theta = startTheta;

    // Convert initial pose to prevX, prevY, prevThetaRad for odomTask
    prevX = xRot->position(turns) * wheelCircumference + startX;
    prevY = yRot->position(turns) * wheelCircumference + startY;
    prevThetaRad = startTheta * (M_PI / 180.0); // convert deg to rad

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
    poseCopy.ySensor = yRot->position(turns) * wheelCircumference;
    poseCopy.xSensor = xRot->position(turns) * wheelCircumference;
    return poseCopy;
}
