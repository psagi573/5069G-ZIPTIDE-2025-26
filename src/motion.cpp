#include "motion.h"
#include "pid.h"
#include "profile.h"
#include "robot-config.h"
#include <cmath>
#include <algorithm>
#include "utils.h"

using namespace vex;

const double wheelTrack = 11.5; // in inches (left-right distance)

// Motor groups

// PID class values
PID distPID(0.05, 0, 0.6);
PID fastTurnPID(0.043, 0.0001, 0.39);

PID arcPID(0.15, 0, 0);
PID sweepPID(0.04, 0, 0.5);

PID drivePID(0.05, 0, 0.8);
PID headingPID(0, 0, 0);

const double maxVelDefault = 60; // max linear speed in inches/sec
const double accelDefault = 30;  // acceleration in inches/sec^2

const double maxVelshort = 60; // max linear speed in inches/sec
const double accelshort = 20;  // acceleration in inches/sec^2

// Set left and right motor voltages
void setDrive(double left, double right)
{
    // Clamp values for safety
    left = clamp(left, -12.0, 12.0);
    right = clamp(right, -12.0, 12.0);

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
    L1.stop(coast);
    L2.stop(coast);
    L3.stop(coast);
    R6.stop(coast);
    R7.stop(coast);
    R8.stop(coast);
}

void stops()
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
    if (v > -2.0 && v < 0)
        return -2.0;
    if (v > 0 && v < 2.0)
        return 2.0;
    return v;
}

void drive(double distInches)
{
    distPID.reset();

    //distInches = distInches-2; //compensate for overshoot
    // Define starting postion
    double startX = getPose().x;
    double startY = getPose().y;
    double target = distInches;

    double lastError = 0;
    int elapsed = 0;
    const int timeout = 1500;

    while (true)
    {
        // Current pose and distance traveled
        Pose pose = getPose();
        double dx = pose.x - startX;
        double dy = pose.y - startY;
        double traveled = sqrt(dx * dx + dy * dy);
        if (distInches < 0)
        {
            traveled = -traveled;
        }
        double error = target - traveled;

        // Compute linear output (PID)
        double linearOut = distPID.compute(target, traveled);
        // Clamp linearOut to [-1,+1]
        // linearOut = clamp(linearOut, -1.0, 1.0);

        // Scale to volts
        linearOut = linearOut * 12.0;
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

    stops();
}

void turn(double targetHeading)
{
    fastTurnPID.reset();
    double elapsedTime = 0;
    const double timeout = 1500; // ms timeout

    while (true)
    {
        Pose pose = getPose();
        double heading = pose.theta;

        // PID output for turning
        double turnOutput = fastTurnPID.compute(targetHeading, heading, true);

        double remaining = targetHeading - heading;

        // Clamp output between -1 and 1, then scale to volts
        turnOutput = clamp(turnOutput, -1.0, 1.0);

        double leftVolt = 12 * turnOutput;
        double rightVolt = -12 * turnOutput;

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

    stops();
}

void arc(double radiusInches, double angleDeg)
{
    arcPID.reset();
    double elapsedTime = 0;
    const double maxTime = 3000;       // ms timeout
    const double distTolerance = 0.5;  // inches
    const double angleTolerance = 1.0; // degrees

    // Determine arc direction from radius sign
    bool arcLeft = (radiusInches < 0);

    // Use absolute radius for calculations
    double absRadius = fabs(radiusInches);

    // Calculate arc length (always positive)
    double arcLength = 2.0 * M_PI * absRadius * (fabs(angleDeg) / 360.0);

    // Starting pose
    Pose startPose = getPose();

    // Calculate target heading and normalize to [-180, 180]
    double targetHeading = startPose.theta + angleDeg;
    while (targetHeading > 180)
        targetHeading -= 360;
    while (targetHeading < -180)
        targetHeading += 360;

    // Calculate wheel speed ratio
    double innerRadius = absRadius - (wheelTrack / 2.0);
    double outerRadius = absRadius + (wheelTrack / 2.0);
    double speedRatio = innerRadius / outerRadius;

    while (true)
    {
        Pose pose = getPose();

        // Calculate traveled distance as Euclidean distance
        double dx = pose.x - startPose.x;
        double dy = pose.y - startPose.y;
        double traveled = sqrt(dx * dx + dy * dy);

        traveled = traveled / 2;
        // Calculate heading error normalized to [-180, 180]
        double headingError = targetHeading - pose.theta;
        while (headingError > 180)
            headingError -= 360;
        while (headingError < -180)
            headingError += 360;

        // Exit conditions
        if ((fabs(traveled - arcLength) <= distTolerance && fabs(headingError) <= angleTolerance) ||
            elapsedTime >= maxTime)
            break;

        // PID output for linear distance
        double linearOut = arcPID.compute(arcLength, traveled);
        linearOut *= 12.0; // convert to volts

        // Determine direction (forward or reverse) from sign of angleDeg
        linearOut *= (angleDeg >= 0) ? 1.0 : -1.0;

        double leftVolt = linearOut;
        double rightVolt = linearOut;

        // Adjust voltages based on arc direction
        if (!arcLeft)
            rightVolt *= speedRatio; // right wheel is inner wheel on left arc
        else
            leftVolt *= speedRatio; // left wheel is inner wheel on right arc

        setDrive(leftVolt, rightVolt);

        elapsedTime += 10;
        wait(10, msec);
    }

    stops();
}

void Sweep(double targetAngleDeg, bool left)
{
    sweepPID.reset();
    double elapsedTime = 0;
    const double timeout = 2000; // ms timeout

    while (true)
    {
        // Current pose and heading
        Pose pose = getPose();
        double heading = pose.theta;
        // PID output for turning
        double turnOutput = sweepPID.compute(targetAngleDeg, heading, true);

        double remaining = targetAngleDeg - heading;

        if (remaining < -180)
            remaining += 360;
        if (remaining > 180)
            remaining -= 360;
        // Exit conditions
        if (fabs(remaining) < 1.0 || elapsedTime >= timeout)
            break;

        // Clamp turn output and convert to volts

        turnOutput = clamp(turnOutput, -1.0, 1.0);
        double Volts = minVolt(turnOutput * 12.0);

        // Set voltages based on turn direction
        if (left)
            setDrive(Volts, 0);
        else
            setDrive(0, Volts);

        elapsedTime += 10;
        wait(10, msec);
    }

    stops();
}

// Drive to a target (x,y) using odom, motion profiling, and PID
void driveTo(double targetX, double targetY)
{
    drivePID.reset();
    headingPID.reset();

    Pose start = getPose();
    double dx = targetX - start.x;
    double dy = targetY - start.y;
    double distance = sqrt(dx * dx + dy * dy);
    distance = distance - 2;

    double targetHeading = atan2(dy, dx) * 180.0 / M_PI;

    if (targetHeading < 0)
        targetHeading += 360;

    double traveled = 0;
    double lastRemaining = distance;
    int elapsed = 0;
    const int timeout = 2000; // safety timeout

    double lastTurnCorrection = 0;
    const double alpha = 0.1; // smoothing factor

    while (true)
    {
        Pose pose = getPose();

        // Distance traveled from start
        double dxt = pose.x - start.x;
        double dyt = pose.y - start.y;
        traveled = sqrt(dxt * dxt + dyt * dyt);

        double remaining = distance - traveled;
        double direction = (remaining >= 0) ? 1.0 : -1.0;

        // Heading error normalized to [-180, 180]
        double headingError = targetHeading - pose.theta;
        while (headingError > 180)
            headingError -= 360;
        while (headingError < -180)
            headingError += 360;

        // Exit condition

        if ((fabs(remaining - lastRemaining) < 0.05 && fabs(remaining) < 0.5)|| elapsed > timeout)
            break;

        // Feedback PID for distance
        double driveOutput = drivePID.compute(distance, traveled);

        // Heading correction (simple P controller)
        double turnCorrection = headingPID.compute(targetHeading, pose.theta, true);

        // Scale down turn correction to reduce influence
        turnCorrection *= 0.2;

        // Clamp max turn correction to prevent sudden large turns
        const double maxTurnCorrection = 0.3;
        if (turnCorrection > maxTurnCorrection)
            turnCorrection = maxTurnCorrection;
        else if (turnCorrection < -maxTurnCorrection)
            turnCorrection = -maxTurnCorrection;

        // Smooth turn correction using exponential moving average
        turnCorrection = alpha * turnCorrection + (1 - alpha) * lastTurnCorrection;
        lastTurnCorrection = turnCorrection;

        if (fabs(headingError) < 2.0 || fabs(remaining) < 5.0)
            turnCorrection = 0;

        driveOutput = clamp(driveOutput, -1.0, 1.0);
        // Convert velocities to voltages
        double leftVolt = (driveOutput - 0) * 12.0;
        double rightVolt = (driveOutput + 0) * 12.0;

        leftVolt = minVolt(leftVolt);
        rightVolt = minVolt(rightVolt);

        setDrive(leftVolt, rightVolt);

        lastRemaining = remaining;
        elapsed += 10;
        wait(10, msec);
    }

    stops();
}
