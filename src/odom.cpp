#include "vex.h"
#include "odom.h"

using namespace vex;

// Internal position state
Pose currentPose = {0.0, 0.0, 0.0};

// Sensor pointers
rotation *xRot;
rotation *yRot;
inertial *imuSensor;

double prevX = 0;
double prevY = 0;
double prevThetaRad = 0;

int odomTask()
{
    while (true)
    {
        // 1. Get current sensor positions
        double currX = xRot->position(turns); // X rotation in revs
        double currY = yRot->position(turns); // Y rotation in revs

        // 2. Convert wheel movement to inches
        const double odomWheelDiameter = 2.00;
        const double wheelCircumference = odomWheelDiameter * M_PI; // ≈ 6.283 in

        double deltaX = (currX - prevX) * wheelCircumference;
        double deltaY = (currY - prevY) * wheelCircumference;

        // 3. Get current heading in radians
        double thetaDeg = imuSensor->rotation();
        double thetaRad = thetaDeg * (M_PI / 180.0);

        // 4. Wrap heading delta to [-π, π]
        double deltaTheta = thetaRad - prevThetaRad;
        if (deltaTheta > M_PI)
            deltaTheta -= 2 * M_PI;
        if (deltaTheta < -M_PI)
            deltaTheta += 2 * M_PI;

        // 5. Transform local delta into global delta
        double deltaXGlobal, deltaYGlobal;
        if (fabs(deltaTheta) < 0.01)
        {
            deltaXGlobal = deltaX * cos(thetaRad) - deltaY * sin(thetaRad);
            deltaYGlobal = deltaX * sin(thetaRad) + deltaY * cos(thetaRad);
        }
        else
        {
            double avgTheta = prevThetaRad + deltaTheta / 2.0;
            deltaXGlobal = deltaX * cos(avgTheta) - deltaY * sin(avgTheta);
            deltaYGlobal = deltaX * sin(avgTheta) + deltaY * cos(avgTheta);
        }

        // 6. Update position state
        currentPose.x += deltaXGlobal;
        currentPose.y += deltaYGlobal;

        //  Wrap theta to [0, 360]
        double cleanTheta = fmod(thetaDeg, 360.0);
        if (cleanTheta < 0)
            cleanTheta += 360.0;
        currentPose.theta = cleanTheta;

        // 7. Save previous states
        prevX = currX;
        prevY = currY;
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

    prevX = xRot->position(turns);
    prevY = yRot->position(turns);
    prevThetaRad = imuSensor->rotation() * (M_PI / 180.0);

    thread odomThread(odomTask);
}

Pose getPose()
{
    return currentPose;
}
