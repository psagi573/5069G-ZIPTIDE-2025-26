#include "motion.h"
#include "pid.h"
#include "profile.h"
#include "robot-config.h"
#include <cmath>
#include <algorithm>
#include "utils.h"

using namespace vex;

const double wheelTrack = 14.5; // in inches (left-right distance)

// Motor groups

// PID class values
PID distPID(0.15, 0, 0.9);
PID fastTurnPID(0.043, 0.0001, 0.39);

const double maxVelDefault = 45; // max linear speed in inches/sec
const double accelDefault = 120; // acceleration in inches/sec^2

// Set left and right motor voltages
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

// Stop all motors
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

// Check volatage and apply minimum voltage if needed
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

    // Define starting postion
    double startX = getPose().x;
    double startY = getPose().y;
    double target = distInches;

    double lastError = 0;
    int elapsed = 0;
    const int timeout = 3000;

    while (true)
    {
        // Current pose and distance traveled
        Pose pose = getPose();
        double dx = pose.x - startX;
        double dy = pose.y - startY;
        double traveled = sqrt(dx * dx + dy * dy);
        double error = target - traveled;

        // Compute linear output (PID)
        double linearOut = distPID.compute(target, traveled);
        // Clamp linearOut to [-1,+1]
        linearOut = clamp(linearOut, -1.0, 1.0);

        // Scale to volts
        linearOut = linearOut * 11.0;
        // Check and apply minimum voltage
        linearOut = minVolt(linearOut);

        double leftVolt = linearOut;
        double rightVolt = linearOut;

        setDrive(leftVolt, rightVolt);

        // Exit conditions
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
    const double timeout = 3000; // ms timeout

    while (true)
    {
        Pose pose = getPose();
        double heading = pose.theta;

        // PID output for turning
        double turnOutput = fastTurnPID.compute(targetHeading, heading, true);

        double remaining = targetHeading - heading;

        // Clamp output between -1 and 1, then scale to volts
        turnOutput = clamp(turnOutput, -1.0, 1.0);

        double leftVolt = 11 * turnOutput;
        double rightVolt = -11 * turnOutput;

        // Apply minimum voltage
        leftVolt = minVolt(leftVolt);
        rightVolt = minVolt(rightVolt);

        // Exit conditions
        if (fabs(remaining) < 1.0 || elapsedTime >= timeout)
            break;

        setDrive(leftVolt, rightVolt);

        elapsedTime += 10;
        wait(10, msec);
    }

    stop();
}

void arc(double radiusInches, double angleDeg, bool forward, bool right)
{
    distPID.reset();
    double elapsedTime = 0;
    const double maxTime = 3000; // ms timeout
    const double distExit = 0.5; // inches tolerance
    const double angExit = 1.0;  // degrees tolerance

    // Calculate arc length
    double arcLength = 2.0 * M_PI * radiusInches * (fabs(angleDeg) / 360.0);
    double targetDistance = arcLength;
    // Starting pose
    Pose startPose = getPose();
    double targetHeading = startPose.theta + (right ? angleDeg : -angleDeg);

    // Normalize target heading to [0, 360)
    if (targetHeading < 0)
        targetHeading += 360.0;
    if (targetHeading >= 360.0)
        targetHeading -= 360.0;

    // Wheel ratio for arc
    double turnRatio = (radiusInches - (wheelTrack / 2.0)) /
                       (radiusInches + (wheelTrack / 2.0));

    while (true)
    {
        Pose pose = getPose();
        // Distance traveled along the arc
        double traveled = fabs(pose.ySensor - startPose.ySensor);
        double remainingDist = targetDistance - traveled;

        double headingError = targetHeading - pose.theta;
        if (headingError > 180)
            headingError -= 360;
        if (headingError < -180)
            headingError += 360;

        // Exit conditions
        if ((fabs(remainingDist) <= distExit && fabs(headingError) <= angExit) ||
            elapsedTime >= maxTime)
            break;

        // PID output for forward motion
        double linearOut = distPID.compute(targetDistance, traveled);
        linearOut = clamp(linearOut, -1.0, 1.0) * 11.0; // Convert to volts

        // Apply forward/reverse factor
        linearOut *= (forward ? 1.0 : -1.0);

        double leftOut = linearOut;
        double rightOut = linearOut;

        if (right) // Arc to the right
            leftOut *= turnRatio;
        else // Arc to the left
            rightOut *= turnRatio;

        setDrive(leftOut, rightOut);

        elapsedTime += 10;
        wait(10, msec);
    }

    stop();
}

void Sweep(double targetAngleDeg, bool left, bool forward)
{
    fastTurnPID.reset();
    double elapsedTime = 0;
    const double timeout = 3000; // ms timeout

    while (true)
    {
        // Current pose and heading
        Pose pose = getPose();
        double heading = pose.theta;
        // PID output for turning
        double turnOutput = fastTurnPID.compute(targetAngleDeg, heading, true);

        double remaining = targetAngleDeg - heading;
        // Exit conditions
        if (fabs(remaining) < 1.0 || elapsedTime >= timeout)
            break;

        // Clamp turn output and convert to volts
        turnOutput = clamp(turnOutput, -1.0, 1.0);
        double Volts = minVolt(turnOutput * 11.0);

        // Apply forward/reverse factor
        Volts *= (forward ? 1.0 : -1.0);

        double leftvolt = 0;
        double rightvolt = 0;

        // Set voltages based on turn direction
        if (left)
            leftvolt = Volts;
        else
            rightvolt = Volts;

        setDrive(leftvolt, rightvolt);

        elapsedTime += 10;
        wait(10, msec);
    }

    stop();
}

void driveTo(double targetX, double targetY)
{
    distPID.reset();

    // Current pose
    Pose pose = getPose();
    double dx = targetX - pose.x;
    double dy = targetY - pose.y;

    // Distance and angle to target
    double distance = sqrt(dx * dx + dy * dy);
    double angleToTarget = atan2(dy, dx) * (180.0 / M_PI); // in degrees

    // Normalize to [0,360)
    if (angleToTarget < 0)
        angleToTarget += 360.0;

    double headingError = angleToTarget - pose.theta;
    // Wrap heading error to [-180,180]
    if (headingError > 180)
        headingError -= 360;
    if (headingError < -180)
        headingError += 360;

    // If heading off by more than 10°, turn first
    if (fabs(headingError) > 10.0)
    {
        turn(angleToTarget); // use your existing turn() function
    }

    // Initialize motion profile for the distance
    Profile profile(maxVelDefault, accelDefault);

    double startX = getPose().x;
    double startY = getPose().y;
    double lastError = 0;
    int elapsed = 0;
    const int timeout = 4000; // extend timeout for long moves

    while (true)
    {
        Pose currPose = getPose();
        double traveledX = currPose.x - startX;
        double traveledY = currPose.y - startY;
        double traveled = sqrt(traveledX * traveledX + traveledY * traveledY);
        double remaining = distance - traveled;
        double direction = (remaining >= 0) ? 1.0 : -1.0;

        // Exit condition
        if ((fabs(remaining) < 0.5 && fabs(remaining - lastError) < 0.1) || elapsed > timeout)
            break;

        // Distance PID output
        double linearOut = distPID.compute(distance, traveled);
        double velLimit = profile.getTargetVelocity(fabs(remaining), traveled, direction);

        linearOut = clamp(linearOut, -velLimit, velLimit);

        // Scale to volts
        double leftVolt = linearOut * 11.0;
        double rightVolt = linearOut * 11.0;

        setDrive(leftVolt, rightVolt);

        lastError = remaining;
        elapsed += 10;
        wait(10, msec);
    }

    setDrive(0, 0);
}
