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

// PID controllers
PID distPID(0.15, 0, 0.9, 1.0);
PID headingPID(0.04, 0.0, 0.33, 1.0);
PID fastTurnPID(0.04, 0, 0.33, 1.0);

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
void longdrive(double distInches, double headingDeg)
{
    distPID.reset();
    headingPID.reset();

    Profile profile(maxVelDefault, accelDefault);

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
        double direction = (error >= 0) ? 1.0 : -1.0;

        if ((fabs(error) < 0.5 && fabs(error - lastError) < 0.1) || elapsed > timeout)
            break;

        // If using motion profile, apply velocity limit and clamp output
        // Motion profiling clamp only
        double velLimit = profile.getTargetVelocity(fabs(error), traveled, direction);
        linearOut = clamp(linearOut, -velLimit, velLimit);

        // Clamp linear output to ±1 before scaling volts for smooth control
        double turnOut = 0.0;
        double headingError = headingDeg - pose.theta;

        // Normalize heading error to [-180, 180]
        if (headingError > 180)
            headingError -= 360;
        if (headingError < -180)
            headingError += 360;

        if (fabs(headingError) > 5)
        {
            turnOut = headingPID.compute(headingError, 0);

            double headingScale = 0.02; // Tunable
            turnOut *= fabs(headingError) * headingScale;

            // Soft cap to avoid overcorrection
            turnOut = clamp(turnOut, -0.3, 0.3); // ~3.3V max correction
        }

        // Scale outputs from PID range [-1, 1] to volts [-11, 11]
        linearOut = linearOut * 11.0;
        turnOut = turnOut * 11.0;

        double leftVolt = linearOut - turnOut;
        double rightVolt = linearOut + turnOut;

        setDrive(leftVolt, rightVolt);

        // Exit conditions: close enough or timeout
        lastError = error;
        elapsed += 10;
        wait(10, msec);
    }

    setDrive(0, 0);
}

void fastdrive(double distInches)
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

    setDrive(0, 0);
}

void turn(double targetHeading)
{
    fastTurnPID.reset();
    double elapsedTime = 0;
    const double timeout = 3000;

    double error = 0;
    double lastError = 0;
    double turnOutput = 0;

    while (true)
    {
        double heading = getPose().theta;
        error = targetHeading - heading;

        // Wrap angle error [-180, 180]
        if (error > 180)
            error -= 360;
        if (error < -180)
            error += 360;

        // Only accumulate integral if error is small enough
        if (fabs(error) < 10)
        {
            fastTurnPID.integral += error;
        }

        // Use your PID class for final output
        fastTurnPID.prevError = lastError; // For derivative
        turnOutput = fastTurnPID.kP * error + fastTurnPID.kI * fastTurnPID.integral + fastTurnPID.kD * (error - lastError);

        // Clamp output between -1 and 1, then scale to volts
        if (turnOutput > 1)
            turnOutput = 1;
        if (turnOutput < -1)
            turnOutput = -1;

        double leftVolt = 11 * turnOutput;
        double rightVolt = -11 * turnOutput;

        setDrive(leftVolt, rightVolt);

        // Exit condition (same as old version)
        if (fabs(error) < 1.0 || elapsedTime >= timeout)
            break;

        lastError = error;
        elapsedTime += 10;
        wait(10, msec);
    }

    setDrive(0, 0);
}

/*void turn(double targetHeading)
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
        double output = (0.7 * error) + (0.0 * turnIntegral) + (0.8 * derivative);

        output = clamp(output, -100.0, 100.0);
        output = output / 100;     // converts to decimal units
        output = output * 11;      // converts to volt units
        setDrive(output, -output); // Left / Right voltages for turn

        // Break condition (very similar to last year)
        if (fabs(error) < 1.5 && fabs(derivative) < 0.5)
            break;

        lastError = error;
        elapsedTime += 10;
        wait(10, msec);
    }

    setDrive(0, 0);
}*/

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
