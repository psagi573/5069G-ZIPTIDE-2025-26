#include "motion.h"
#include "pid.h"
#include "profile.h"
#include "robot-config.h"
#include <cmath>
#include <algorithm>

using namespace vex;

template <typename T>
T clamp(T value, T minVal, T maxVal)
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}

// Constants
const double wheelTrack = 14.5; // in inches (left-right distance)

const double maxVelDefault = 45; // max linear speed in inches/sec
const double accelDefault = 120; // acceleration in inches/sec^2

const double maxVelShort = 25; // slower for short moves
const double accelShort = 60;  // slower accel for short moves

// Motor groups

// PID controllers
PID distPID(0.15, 0, 0.9);
PID fastTurnPID(0.043, 0.0001, 0.39);

void setDrive(double left, double right)
{
    // Clamp values for safety
    left = clamp(left, -11.0, 11.0);
    right = clamp(right, -11.0, 11.0);

    // Send voltage to motors (volt units)
    L1.spin(fwd, left, volt);
    L2.spin(fwd, left, volt);
    L3.spin(fwd, left, volt);

    R6.spin(fwd, right, volt);
    R7.spin(fwd, right, volt);
    R8.spin(fwd, right, volt);
}

void stop()
{
    // Stop all motors
    L1.stop(brake);
    L2.stop(brake);
    L3.stop(brake);
    R6.stop(brake);
    R7.stop(brake);
    R8.stop(brake);
}

double minVolt(double v)
{
    if (v > -3.0 && v < 0)
        return -3.0;
    if (v > 0 && v < 3.0)
        return 3.0;
    return v;
}

void drive(double distInches)
{
    distPID.reset();

    double startX = getPose().x;
    double startY = getPose().y;
    double target = distInches;

    double lastError = 0;
    int elapsed = 0;
    const int timeout = 3000;

    while (true)
    {
        Pose pose = getPose();
        double dx = pose.x - startX;
        double dy = pose.y - startY;
        double traveled = sqrt(dx * dx + dy * dy);
        double error = target - traveled;

        // Compute linear output (PID)
        double linearOut = distPID.compute(target, traveled);
        // For snappy control without profile, clamp linearOut to ±1 as well
        if (linearOut > 1.0)
            linearOut = 1.0;
        if (linearOut < -1.0)
            linearOut = -1.0;

        linearOut = linearOut * 11.0;

        linearOut = minVolt(linearOut);

        double leftVolt = linearOut;
        double rightVolt = linearOut;

        setDrive(leftVolt, rightVolt);

        // Exit conditions: close enough or timeout
        if ((fabs(error) < 0.5 && fabs(error - lastError) < 0.1) || elapsed > timeout)
            break;

        lastError = error;
        elapsed += 10;
        wait(10, msec);
    }

    stop();
}

void turn(double targetHeading)
{
    fastTurnPID.reset();
    double elapsedTime = 0;
    const double timeout = 3000;

    while (true)
    {
        Pose pose = getPose();
        double heading = pose.theta;

        double turnOutput = fastTurnPID.compute(targetHeading, heading, true);

        double remaining = targetHeading - heading;

        // Clamp output between -1 and 1, then scale to volts
        if (turnOutput > 1)
            turnOutput = 1;
        if (turnOutput < -1)
            turnOutput = -1;

        double leftVolt = 11 * turnOutput;
        double rightVolt = -11 * turnOutput;

        leftVolt = minVolt(leftVolt);
        rightVolt = minVolt(rightVolt);

        if (fabs(remaining) < 1.0 || elapsedTime >= timeout)
            break;

        setDrive(leftVolt, rightVolt);

        elapsedTime += 10;
        wait(10, msec);
    }

    stop();
}

void arc(double radiusInches, double angleDeg)
{
    distPID.reset();
    double elapsedTime = 0;
    const double maxTime = 3000;
    const double distExit = 0.5;

    // Calculate the arc length
    double arcLength = 2.0 * M_PI * radiusInches * (angleDeg / 360.0);

    Pose startPose = getPose();
    double targetDistance = fabs(arcLength);

    //  wheel ratio for arc
    double turnRatio = (radiusInches - (wheelTrack / 2.0)) /
                       (radiusInches + (wheelTrack / 2.0));

    // Determine direction of turn
    int turnDir = (angleDeg >= 0) ? 1 : -1;

    while (true)
    {
        Pose pose = getPose();
        double dx = pose.x - startPose.x;
        double dy = pose.y - startPose.y;
        double traveled = sqrt(dx * dx + dy * dy);
        double remaining = targetDistance - traveled;

        // Exit condition
        if (fabs(remaining) <= distExit || elapsedTime >= maxTime)
        {
            break;
        }

        // PID output for forward motion
        double linearOut = distPID.compute(targetDistance, traveled);

        // Clamp to motor safe range (-1 to 1)
        linearOut = clamp(linearOut, -1.0, 1.0);

        // Convert to volts for VEX
        linearOut *= 11.0;

        // Apply turn ratio
        double leftOut = linearOut;
        double rightOut = linearOut;

        if (turnDir > 0)
        { // Turning right
            leftOut *= turnRatio;
        }
        else
        { // Turning left
            rightOut *= turnRatio;
        }

        setDrive(leftOut, rightOut);

        elapsedTime += 10;
        wait(10, msec);
    }

    stop();
}
