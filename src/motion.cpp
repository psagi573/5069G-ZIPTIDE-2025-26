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

const double maxVelDefault = 45; // max linear speed in inches/sec
const double accelDefault = 120; // acceleration in inches/sec^2

const double maxVelShort = 25; // slower for short moves
const double accelShort = 60;  // slower accel for short moves

// PID controllers
PID distPID(0.15, 0.0, 0.075, 11.0);
PID headingPID(0.3, 0.0, 0.7, 11.0);
PID fastTurnPID(0.038, 0.0001, 0.35, 11.0);

void setDrive(double left, double right)
{
    // Clamp values for safety
    left = clamp(left, -11.0, 11.0);
    right = clamp(right, -11.0, 11.0);

    // Send voltage to motors (percent units)
    // Replace these with your actual motor names
    L1.spin(fwd, left, volt);
    L2.spin(fwd, left, volt);
    L3.spin(fwd, left, volt);

    R6.spin(fwd, right, volt);
    R7.spin(fwd, right, volt);
    R8.spin(fwd, right, volt);
}

void drive(double distInches, double headingDeg)
{
    distPID.reset();
    headingPID.reset();
    double maxVelLocal;
    double accelLocal;

    if (fabs(distInches) < 12.0)
    {
        maxVelLocal = maxVelShort;
        accelLocal = accelShort;
    }
    else
    {
        maxVelLocal = maxVelDefault;
        accelLocal = accelDefault;
    }

    Profile profile(maxVelLocal, accelLocal);

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
        double direction = (remaining >= 0) ? 1.0 : -1.0;
        double velLimit = profile.getTargetVelocity(fabs(remaining), traveled, direction);
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
    fastTurnPID.reset();
    const double timeout = 1500;
    double elapsedtime = 0;
    double lastError = 0;

    while (true)
    {
        double heading = getPose().theta;
        double error = targetHeading - heading;

        if (error > 180)
            error -= 360;
        if (error < -180)
            error += 360;

        double output = fastTurnPID.compute(0, -error);
        output = clamp(output, -1.0, 1.0);
        double voltage = output * 11.0;

        setDrive(-voltage, voltage);

        if ((fabs(error) < 1.0 && fabs(error - lastError) < 0.2) || elapsedtime > timeout)
            break;

        lastError = error;

        elapsedtime += 10;
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

    double maxVelLocal;
    double accelLocal;

    if (arcLength < 6.0)
    {
        maxVelLocal = maxVelShort;
        accelLocal = accelShort;
    }
    else
    {
        maxVelLocal = maxVelDefault;
        accelLocal = accelDefault;
    }

    Profile profile(maxVelLocal, accelLocal);

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

        double direction = (remaining >= 0) ? 1.0 : -1.0;
        double velLimit = profile.getTargetVelocity(fabs(remaining), traveled, direction);
        linearOut = clamp(linearOut, -velLimit, velLimit);

        double left = linearOut * turnRatio;
        double right = linearOut;
        setDrive(left, right);
        wait(10, msec);
    }
    setDrive(0, 0);
}
/*void sweep(double angleDeg, bool rightSideMoving = true)
{
    headingPID.reset();
    double startHeading = getPose().theta;
    double targetHeading = startHeading + angleDeg;
    double maxVelLocal;
    double accelLocal;

    if (targetHeading < 90)
    {
        maxVelLocal = maxVelShort;
        accelLocal = accelShort;
    }
    else
    {
        maxVelLocal = maxVelDefault;
        accelLocal = accelDefault;
    }

    Profile profile(maxVelLocal, accelLocal);

    // Normalize target heading between 0 and 360
    if (targetHeading > 360)
        targetHeading -= 360;
    if (targetHeading < 0)
        targetHeading += 360;

    while (true)
    {
        double current = getPose().theta;
        double error = targetHeading - current;

        // Wrap error to range [-180, 180]
        if (error > 180)
            error -= 360;
        if (error < -180)
            error += 360;

        // Break if close enough
        if (fabs(error) <= 1.0)
            break;

        double traveled = angleDeg - error;
        double direction = (remaining >= 0) ? 1.0 : -1.0;
        double velLimit = profile.getTargetVelocity(fabs(error), fabs(traveled), direction);
        double output = headingPID.compute(angleDeg, traveled); // PID targets total sweep angle

        output = clamp(output, -velLimit, velLimit);

        // Only one side moves
        if (rightSideMoving)
            setDrive(0, output); // right sweep (right moves)
        else
            setDrive(output, 0); // left sweep (left moves)

        wait(10, msec);
    }

    setDrive(0, 0);
}
*/