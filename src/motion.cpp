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
PID distPID(0.05, 0, 0.4);
PID fastTurnPID(0.043, 0.0001, 0.39);
PID arcPID(0.15, 0, 0);
PID sweepPID(0.04, 0, 0.5);

const double maxVelDefault = 45; // max linear speed in inches/sec
const double accelDefault = 120; // acceleration in inches/sec^2

const double maxVelshort = 45; // max linear speed in inches/sec
const double accelshort = 120; // acceleration in inches/sec^2

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
    const int timeout = 1500;

    while (true)
    {
        // Current pose and distance traveled
        Pose pose = getPose();
        double dx = pose.x - startX;
        double dy = pose.y - startY;
        double traveled = sqrt(dx * dx + dy * dy);
        if (distInches < 0){
            traveled = -traveled;
        }
        double error = target - traveled;

        // Compute linear output (PID)
        double linearOut = distPID.compute(target, traveled);
        // Clamp linearOut to [-1,+1]
        //linearOut = clamp(linearOut, -1.0, 1.0);

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
    const double maxTime = 3000; // ms timeout
    const double distTolerance = 0.5; // inches
    const double angleTolerance = 1.0; // degrees

    // Determine arc direction from radius sign
    bool arcLeft = (radiusInches > 0);

    // Use absolute radius for calculations
    double absRadius = fabs(radiusInches);

    // Calculate arc length (always positive)
    double arcLength = 2.0 * M_PI * absRadius * (fabs(angleDeg) / 360.0);

    // Starting pose
    Pose startPose = getPose();

    // Calculate target heading and normalize to [-180, 180]
    double targetHeading = startPose.theta + angleDeg;
    while (targetHeading > 180) targetHeading -= 360;
    while (targetHeading < -180) targetHeading += 360;

    // Calculate wheel speed ratio
    double innerRadius = absRadius - (wheelTrack / 2.0);
    double outerRadius = absRadius + (wheelTrack / 2.0);
    double speedRatio = innerRadius / outerRadius;

    while (true)
    {
        Pose pose = getPose();

        // Calculate traveled distance as Euclidean distance
        double dx = pose.xSensor - startPose.xSensor;
        double dy = pose.ySensor - startPose.ySensor;
        double traveled = sqrt(dx*dx + dy*dy);

        // Calculate heading error normalized to [-180, 180]
        double headingError = targetHeading - pose.theta;
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;

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
        if (arcLeft)
            rightVolt *= speedRatio;  // right wheel is inner wheel on left arc
        else
            leftVolt *= speedRatio;   // left wheel is inner wheel on right arc

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
            setDrive(Volts,0);
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
    distPID.reset();
    fastTurnPID.reset();

    Pose start = getPose();
    double dx = targetX - start.x;
    double dy = targetY - start.y;
    double distance = sqrt(dx * dx + dy * dy);

    // Initialize motion profile (short vs long move scaling)
    double maxVel = (distance < 10) ? maxVelshort : maxVelDefault;
    double accel = (distance < 10) ? accelshort : accelDefault;
    Profile profile(maxVel, accel);

    double traveled = 0;
    double lastRemaining = distance;
    int elapsed = 0;
    const int timeout = 4000; // safety timeout

    while (true)
    {
        Pose pose = getPose();

        // --- Distance calculations ---
        double dxt = pose.x - start.x;
        double dyt = pose.y - start.y;
        traveled = sqrt(dxt * dxt + dyt * dyt);

        double remaining = distance - traveled;
        double direction = (remaining >= 0) ? 1.0 : -1.0;


        // --- Dynamic exit condition ---
        bool closeEnough = fabs(remaining) < 0.5;
        bool nearlyStopped = fabs(remaining - lastRemaining) < 0.05;
        if ((closeEnough && nearlyStopped) || elapsed > timeout)
            break;
        
        // // --- Heading calculations ---
        // double angleToTarget = atan2(targetY - pose.y, targetX - pose.x) * (180.0 / M_PI);
        // if (angleToTarget < 0)
        //     angleToTarget += 360.0;

        // double headingError = angleToTarget - pose.theta;
        // if (headingError > 180)
        //     headingError -= 360;
        // if (headingError < -180)
        //     headingError += 360;

        // --- Feedforward (motion profile) ---
        double ffVel = profile.getTargetVelocity(fabs(remaining), traveled, direction);

        // --- Feedback (PID) ---
        double pidCorrection = distPID.compute(distance, traveled);
        pidCorrection *= maxVelDefault; // scale to velocity domain

        // --- Combine feedforward + feedback ---
        double commandedVel = ffVel + pidCorrection;
        commandedVel = clamp(commandedVel, -maxVelDefault, maxVelDefault);

        // --- Turn blending (soft heading correction) ---
        // double turnOut = 0.0;
        // if (fabs(headingError) > 2.0)
        // { // ignore tiny noise
        //     turnOut = fastTurnPID.compute(headingError, 0);
        //     // scale softly so it doesn’t fight drive
        //     double blend = 0.15 + 0.1 * (1.0 - fabs(commandedVel) / maxVelDefault);
        //     turnOut = clamp(turnOut * blend, -0.15, 0.15); // volts fraction
        // }

        // --- Convert velocity (in/s) to volts ---
        double linearVolts = (commandedVel / maxVelDefault) * 12.0;
        double leftVolt = linearVolts * 12.0;
        double rightVolt = linearVolts * 12.0;

        setDrive(leftVolt, rightVolt);

        lastRemaining = remaining;
        elapsed += 10;
        wait(10, msec);
    }

    stop();
}
