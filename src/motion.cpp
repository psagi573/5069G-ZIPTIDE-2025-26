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
const double wheelTrack = 14.8; // in inches (left-right distance)
const double maxVel = 30;       // max linear speed in inches/sec
const double accel = 60;        // acceleration in inches/sec^2

// PID controllers
PID distPID(1.2, 0.0, 0.6, 100);
PID headingPID(3.0, 0.0, 1.0, 50);

void setDrive(double left, double right)
{
    // Clamp values for safety
    left = clamp(left, -100.0, 100.0);
    right = clamp(right, -100.0, 100.0);

    // Send voltage to motors (percent units)
    // Replace these with your actual motor names
    L1.spin(fwd, left, percent);
    L2.spin(fwd, left, percent);
    L3.spin(fwd, left, percent);

    R7.spin(fwd, right, percent);
    R7.spin(fwd, right, percent);
    R8.spin(fwd, right, percent);
}

void drive(double distInches, double headingDeg)
{
    distPID.reset();
    headingPID.reset();
    Profile profile(maxVel, accel);

    double startX = getPose().x;
    double startY = getPose().y;
    double target = distInches;

    while (true)
    {
        Pose pose = getPose();
        double dx = pose.x - startX;
        double dy = pose.y - startY;
        double traveled = sqrt(dx * dx + dy * dy);
        double remaining = target - traveled;

        if (remaining <= 0.5)
            break;

        double linearOut = distPID.compute(target, traveled);
        double velLimit = profile.getTargetVelocity(remaining, traveled);
        linearOut = clamp(linearOut, -velLimit, velLimit);

        double headingError = headingDeg - pose.theta;
        if (headingError > 180)
            headingError -= 360;
        if (headingError < -180)
            headingError += 360;
        double turnOut = headingPID.compute(0, -headingError);

        double left = linearOut - turnOut;
        double right = linearOut + turnOut;

        setDrive(left, right);
        wait(10, msec);
    }

    setDrive(0, 0);
}

void turn(double targetHeading)
{
    headingPID.reset();
    while (true)
    {
        double current = getPose().theta;
        double error = targetHeading - current;
        if (error > 180)
            error -= 360;
        if (error < -180)
            error += 360;
        if (fabs(error) <= 1.0)
            break;

        double output = headingPID.compute(0, -error);
        setDrive(-output, output);
        wait(10, msec);
    }
    setDrive(0, 0);
}

void arc(double radiusInches, double angleDeg)
{
    distPID.reset();
    headingPID.reset();

    // Compute arc length
    double arcLength = 2 * M_PI * radiusInches * (angleDeg / 360.0);
    Profile profile(maxVel, accel);

    double startX = getPose().x;
    double startY = getPose().y;
    double target = arcLength;

    // Ratio between left and right wheels
    double turnRatio = (radiusInches - (wheelTrack / 2.0)) / (radiusInches + (wheelTrack / 2.0));

    while (true)
    {
        Pose pose = getPose();
        double dx = pose.x - startX;
        double dy = pose.y - startY;
        double traveled = sqrt(dx * dx + dy * dy);
        double remaining = target - traveled;
        if (remaining <= 0.5)
            break;

        double linearOut = distPID.compute(target, traveled);
        double velLimit = profile.getTargetVelocity(remaining, traveled);
        linearOut = clamp(linearOut, -velLimit, velLimit);

        double left = linearOut * turnRatio;
        double right = linearOut;
        setDrive(left, right);
        wait(10, msec);
    }
    setDrive(0, 0);
}
void sweep(double power, double angleDeg)
{
    headingPID.reset();
    double startHeading = getPose().theta;
    double direction = (angleDeg > 0) ? 1 : -1;

    while (true)
    {
        double currentHeading = getPose().theta;
        double delta = currentHeading - startHeading;

        if (delta > 180)
            delta -= 360;
        if (delta < -180)
            delta += 360;
        if (direction * delta >= fabs(angleDeg))
            break;

        if (direction > 0)
            setDrive(0, power); // sweep right
        else
            setDrive(power, 0); // sweep left

        wait(10, msec);
    }
    setDrive(0, 0);
}