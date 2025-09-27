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
PID headingPID(0.03, 0.0001, 0.1); // Added I and D terms for better heading control

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

void drive(double distInches, double timeout)
{
    distPID.reset();
    double startX = getPose().x;
    double startY = getPose().y;
    double target = distInches;
    double start = getPose().ySensor;
    double lastError = 0;
    int elapsed = 0;

    while (true)
    {
        // Current pose and distance traveled
        Pose pose = getPose();
        double dx = pose.x - startX;
        double dy = pose.y - startY;
        double dstart = pose.ySensor - start;
        double traveled = dstart;
        if (distInches < 0)
        {
            traveled = traveled;
        }
        double error = target - traveled;
        // Compute linear output (PID)
        double linearOut = distPID.compute(target, traveled);
        // Scale to volts
        linearOut = linearOut * 12.0;
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

        // FIXED: Proper angle difference calculation
        double remaining = targetHeading - heading;
        while (remaining > 180) remaining -= 360;
        while (remaining < -180) remaining += 360;

        // Use the normalized remaining angle for PID
        double turnOutput = fastTurnPID.compute(targetHeading, heading, true);

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

    bool arcLeft = (radiusInches < 0);
    double absRadius = fabs(radiusInches);

    double arcLength = 2.0 * M_PI * absRadius * (fabs(angleDeg) / 360.0);
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

        // FIXED: Correct arc direction logic
        if (arcLeft) {
            // Left arc: right wheel is inner (slower), left wheel is outer (faster)
            rightVolt *= speedRatio;
        } else {
            // Right arc: left wheel is inner (slower), right wheel is outer (faster)  
            leftVolt *= speedRatio;
        }

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
    distance = distance - 2; // Compensate for overshoot

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

        // FIXED: More robust exit condition
        bool stoppedMoving = fabs(remaining - lastRemaining) < 0.02;
        bool closeToTarget = fabs(remaining) < 0.3;
        bool timedOut = elapsed > timeout;

        if ((stoppedMoving && closeToTarget) || timedOut)
            break;

        // Feedback PID for distance
        double driveOutput = drivePID.compute(distance, traveled);

        // Heading correction (PID controller)
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

        // Disable turn correction when close to target or aligned
        if (fabs(headingError) < 2.0 || fabs(remaining) < 5.0)
            turnCorrection = 0;

        driveOutput = clamp(driveOutput, -1.0, 1.0);
        
        // FIXED: Apply turn correction to motor voltages
        double leftVolt = (driveOutput - turnCorrection) * 12.0;
        double rightVolt = (driveOutput + turnCorrection) * 12.0;

        leftVolt = minVolt(leftVolt);
        rightVolt = minVolt(rightVolt);

        setDrive(leftVolt, rightVolt);

        lastRemaining = remaining;
        elapsed += 10;
        wait(10, msec);
    }

    stops();
}

// NEW FUNCTION: Advanced moveTo with optional final heading
void moveTo(double targetX, double targetY, double finalHeading, double maxVelocity, double acceleration)
{
    drivePID.reset();
    headingPID.reset();
    
    Pose start = getPose();
    double dx = targetX - start.x;
    double dy = targetY - start.y;
    double distance = sqrt(dx * dx + dy * dy);
    
    // Calculate initial target heading (point towards target)
    double initialTargetHeading = atan2(dy, dx) * 180.0 / M_PI;
    if (initialTargetHeading < 0) initialTargetHeading += 360;
    
    // Use provided max velocity and acceleration or defaults
    double maxVel = (maxVelocity > 0) ? maxVelocity : maxVelDefault;
    double accel = (acceleration > 0) ? acceleration : accelDefault;
    
    // Create motion profile
    Profile profile(maxVel, accel);
    
    double traveled = 0;
    int elapsed = 0;
    const int timeout = 3000; // Longer timeout for moveTo
    
    // Phase tracking: 0=driving to point, 1=turning to final heading
    int phase = 0;
    double phaseSwitchDistance = 3.0; // Switch to turning when this close to target
    
    while (true)
    {
        Pose pose = getPose();
        
        // Distance traveled from start
        double dxt = pose.x - start.x;
        double dyt = pose.y - start.y;
        traveled = sqrt(dxt * dxt + dyt * dyt);
        double remaining = distance - traveled;
        
        // Phase switching logic
        if (phase == 0 && remaining < phaseSwitchDistance) {
            phase = 1; // Switch to final heading adjustment
        }
        
        double targetHeading = (phase == 0) ? initialTargetHeading : finalHeading;
        
        // Heading error normalized to [-180, 180]
        double headingError = targetHeading - pose.theta;
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;
        
        // Motion profile output
        double profileOutput = profile.getTargetVelocity(traveled, elapsed, 1);
        
        // Distance PID for fine adjustment
        double driveOutput = drivePID.compute(distance, traveled);
        
        // Combine profile and PID (profile dominates during motion, PID for precision)
        double combinedOutput = (profileOutput * 0.7) + (driveOutput * 0.3);
        combinedOutput = clamp(combinedOutput, -1.0, 1.0);
        
        // Heading correction - more aggressive in phase 1 (final approach)
        double turnGain = (phase == 0) ? 0.15 : 0.25;
        double turnCorrection = headingPID.compute(targetHeading, pose.theta, true) * turnGain;
        turnCorrection = clamp(turnCorrection, -0.4, 0.4);
        
        // Apply voltages with turn correction
        double leftVolt = (combinedOutput - turnCorrection) * 12.0;
        double rightVolt = (combinedOutput + turnCorrection) * 12.0;
        
        leftVolt = minVolt(leftVolt);
        rightVolt = minVolt(rightVolt);
        
        setDrive(leftVolt, rightVolt);
        
        // Exit conditions
        bool positionGood = (phase == 1) && (fabs(remaining) < 0.5);
        bool headingGood = (fabs(headingError) < 2.0);
        bool timedOut = elapsed > timeout;
        
        if ((positionGood && headingGood) || timedOut)
            break;
        
        elapsed += 10;
        wait(10, msec);
    }
    
    stops();
}

// Overloaded moveTo without final heading (uses point-towards-target)
void moveTo(double targetX, double targetY, double maxVelocity, double acceleration)
{
    Pose start = getPose();
    double dx = targetX - start.x;
    double dy = targetY - start.y;
    double finalHeading = atan2(dy, dx) * 180.0 / M_PI;
    if (finalHeading < 0) finalHeading += 360;
    
    moveTo(targetX, targetY, finalHeading, maxVelocity, acceleration);
}

// Overloaded moveTo with only position
void moveTo(double targetX, double targetY)
{
    moveTo(targetX, targetY, maxVelDefault, accelDefault);
}