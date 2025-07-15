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

// Motor groups
motor_group R = motor_group(R6, R7, R8);
motor_group L = motor_group(L1, L2, L3);

// PID controllers
PID distPID(0, 0, 0., 11.0);
PID headingPID(0., 0.0, 0, 11.0);
PID fastTurnPID(0.08, 0, 0.5, 11.0);

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
        double error = target - traveled;

        double linearOut = distPID.compute(target, traveled);
        double direction = (error >= 0) ? 1.0 : -1.0;
        double velLimit = profile.getTargetVelocity(fabs(error), traveled, direction);
        if (velLimit < 1.0)
            velLimit = 1.0;
        linearOut = clamp(linearOut, -velLimit, velLimit);

        double headingError = headingDeg - pose.theta;
        if (headingError > 180)
            headingError -= 360;
        if (headingError < -180)
            headingError += 360;
        double turnOut = headingPID.compute(headingError, 0);

        double left = linearOut - turnOut;
        double right = linearOut + turnOut;

        setDrive(left, right);

        static double lastError = 0;
        static int elapsed = 0;
        const int timeout = 3000; // in milliseconds

        if ((fabs(error) < 0.5 && fabs(error - lastError) < 0.1) || elapsed > timeout)
            break;

        lastError = error;
        elapsed += 10;
        wait(10, msec);
    }

    setDrive(0, 0);
}

void turn(double targetHeading)
{
    fastTurnPID.reset();
    double turnIntegral = 0;
    double lastError = 0;
    double elapsedTime = 0;
    const double timeout = 3000;

    while (true)
    {
        double heading = getPose().theta;
        double error = targetHeading - heading;

        // Wrap error to [-180, 180]
        if (error > 180)
            error -= 360;
        if (error < -180)
            error += 360;

        // Integral restraint (like your old code)
        if (fabs(error) < 10 && fabs(error) > 10)
            turnIntegral += error;

        // Compute PID using manual integral and derivative
        double derivative = error - lastError;

        // Manually build the PID output using class constants
        double output = (0.02 * error) + (0.0 * turnIntegral) + (0 * derivative);

        if (output > 1)
            output = 1;
        if (output < -1)
            output = -1;

        L.spin(forward, 11 * output, volt);
        R.spin(forward, -11 * output, volt);

        output = clamp(output, -11.0, 11.0);
        setDrive(-output, output); // Left / Right voltages for turn

        // Break condition (very similar to last year)
        if ((error > -1 && error < 1 && error - lastError > -0.2 && error - lastError < 0.2) || (elapsedTime >= timeout))
            break;

        lastError = error;
        elapsedTime += 10;
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
void moveTo(double targetX, double targetY, double targetTheta, bool turnAtEnd = true)
{
    distPID.reset();
    headingPID.reset();

    Pose start = getPose();
    double totalDist = hypot(targetX - start.x, targetY - start.y);

    // Choose profile based on distance
    double maxVelLocal = (totalDist < 12.0) ? maxVelShort : maxVelDefault;
    double accelLocal = (totalDist < 12.0) ? accelShort : accelDefault;
    Profile profile(maxVelLocal, accelLocal);

    double prevError = totalDist;
    double elapsedTime = 0;
    const double timeout = 2000;        // ms
    const double settleThreshold = 0.5; // inches
    const double slopeThreshold = 0.2;  // inches

    while (true)
    {
        Pose pose = getPose();

        double dx = targetX - pose.x;
        double dy = targetY - pose.y;

        double remainingDist = hypot(dx, dy);
        double traveled = totalDist - remainingDist;
        double error = totalDist - traveled;

        double direction = (error >= 0) ? 1.0 : -1.0;

        // Break condition: close and stable, or timeout
        double errorDelta = fabs(prevError - error);
        if ((remainingDist < settleThreshold && errorDelta < slopeThreshold) || elapsedTime > timeout)
        {
            break;
        }

        // Calculate linear PID and velocity profile limit
        double linearOut = distPID.compute(totalDist, traveled);
        double velLimit = profile.getTargetVelocity(fabs(error), traveled, direction);
        if (velLimit < 1.0)
            velLimit = 1.0;
        linearOut = clamp(linearOut, -velLimit, velLimit);

        // Kickstart: if output is tiny but still far, push minimum voltage
        if (fabs(linearOut) < 1.0 && fabs(error) > 1.0)
        {
            linearOut = direction * 1.0;
        }

        // Calculate heading and heading error normalized [-180,180]
        double desiredHeading = atan2(dy, dx) * (180.0 / M_PI);
        double headingError = desiredHeading - pose.theta;
        if (headingError > 180)
            headingError -= 360;
        if (headingError < -180)
            headingError += 360;

        // Heading PID — **NO minus sign here**, matches your turn()
        double turnOut = headingPID.compute(0, headingError);

        // Combine linear and turn outputs for left/right voltages
        double left = linearOut - turnOut;
        double right = linearOut + turnOut;

        setDrive(left, right);

        prevError = error;
        wait(10, msec);
        elapsedTime += 10;
    }

    setDrive(0, 0);

    // Optional final turn to targetTheta
    if (turnAtEnd)
    {
        turn(targetTheta);
    }
}
