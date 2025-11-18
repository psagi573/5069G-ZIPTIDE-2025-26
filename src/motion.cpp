/*#include "motion.h"
#include "pid.h"
#include "profile.h"
#include "robot-config.h"
#include <cmath>
#include <algorithm>
#include "odometry.h"

using namespace vex;

const double wheelTrack = 11.5; // in inches (left-right distance)

// Motor groups

// PID class values
PID distPID(0.05, 0, 0.5);
PID fastTurnPID(0.043, 0.0001, 0.39);

PID arcPID(0.15, 0, 0);
PID sweepPID(0.05, 0, 0.5);

PID drivePID(0.05, 0, 0.6);
PID headingPID(0.043, 0.0001, 0.39);

const double maxVelDefault = 60; // max linear speed in inches/sec
const double accelDefault = 30;  // acceleration in inches/sec^2

const double maxVelshort = 60; // max linear speed in inches/sec
const double accelshort = 20;  // acceleration in inches/sec^2



template <typename T>
T clamp(T value, T minVal, T maxVal)
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}


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

const float WHEEL_CIRCUMFERENCE = 3.1416 * 3.125;               // ~8.639 inches
const float INCHES_PER_MOTOR_TURN = WHEEL_CIRCUMFERENCE * 0.8; // ~5.399 inches

float getAverageDistance()
{
    // Get positions from all motors (in turns/rotations)
    float left1_pos = L1.position(turns);
    float left2_pos = L2.position(turns);
    float left3_pos = L3.position(turns);
    float right1_pos = R6.position(turns);
    float right2_pos = R7.position(turns);
    float right3_pos = R8.position(turns);

    // Calculate average motor rotations
    float avg_rotations = (left1_pos + left2_pos + left3_pos +
                           right1_pos + right2_pos + right3_pos) /
                          6.0;

    // Convert average motor rotations to inches traveled
    float distance_inches = avg_rotations * INCHES_PER_MOTOR_TURN;

    return distance_inches;
}

void drive(double distInches, double timeout)
{
    distPID.reset();
    double target = distInches;
    double start = getAverageDistance(); // in inches
    double lastError = 0;
    int elapsed = 0;

    while (true)
    {
        // Current pose and distance traveled
        Pose pose = Odom::getPose();
        double dstart = getAverageDistance() - start;
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
        Pose pose = Odom::getPose();
        double heading = inertial19.heading();

        // FIXED: Proper angle difference calculation
        double remaining = targetHeading - heading;
        while (remaining > 180)
            remaining -= 360;
        while (remaining < -180)
            remaining += 360;

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
    Pose startPose = Odom::getPose();

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
        Pose pose = Odom::getPose();

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
        if (arcLeft)
        {
            // Left arc: right wheel is inner (slower), left wheel is outer (faster)
            rightVolt *= speedRatio;
        }
        else
        {
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
        Pose pose = Odom::getPose();
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



void moveToPose(double targetX, double targetY, double finalHeading, double timeout)
{
    drivePID.reset();
    headingPID.reset();

    Pose start = Odom::getPose();
    double dxStart = targetX - start.x;
    double dyStart = targetY - start.y;
    double initialDistance = sqrt(dxStart * dxStart + dyStart * dyStart);

    // Calculate the straight-line heading from start to target
    double targetHeadingToPoint = atan2(dyStart, dxStart) * 180.0 / M_PI;
    if (targetHeadingToPoint < 0) targetHeadingToPoint += 360;

    double traveled = 0;
    double lastRemaining = initialDistance;
    int elapsed = 0;
    
    // Tracking for smoother turn correction
    double lastTurnCorrection = 0;
    const double ALPHA = 0.2; // Smoothing factor for exponential moving average

    // Phase tracking
    const double FINAL_TURN_RADIUS = 5.0; // Start transitioning to final angle when this close
    bool performingFinalTurn = false;

    while (true)
    {
        Pose pose = Odom::getPose();

        // 1. Distance Calculation
        double dx = targetX - pose.x;
        double dy = targetY - pose.y;
        double remainingDistance = sqrt(dx * dx + dy * dy);
        
        traveled = initialDistance - remainingDistance; // Distance traveled

        // 2. Heading Target Determination
        double currentTargetHeading;
        if (finalHeading == -1.0 || remainingDistance > FINAL_TURN_RADIUS)
        {
            // Always face the point while driving (Phase 0: Driving)
            currentTargetHeading = atan2(dy, dx) * 180.0 / M_PI;
            if (currentTargetHeading < 0) currentTargetHeading += 360;
        }
        else
        {
            // Transition to the final desired heading (Phase 1: Final Approach)
            currentTargetHeading = finalHeading;
            performingFinalTurn = true;
        }

        // 3. Distance PID (Linear Speed)
        // Use initial distance for PID target, use traveled for current
        double driveOutput = drivePID.compute(initialDistance, traveled, false);
        // Clamp overall drive power (e.g., to prevent over-acceleration)
        driveOutput = clamp(driveOutput, -1.0, 1.0); 

        // 4. Heading PID (Correction)
        double turnCorrection = headingPID.compute(currentTargetHeading, pose.theta, true);
        
        // Scale down turn correction to prevent oversteer (TUNE THIS)
        turnCorrection *= 0.35; 
        
        // Smooth turn correction using exponential moving average
        turnCorrection = ALPHA * turnCorrection + (1 - ALPHA) * lastTurnCorrection;
        lastTurnCorrection = turnCorrection;
        
        // 5. Apply Voltages
        // The driveOutput is the forward power, turnCorrection adjusts left/right
        double leftVolt = (driveOutput - turnCorrection)* 12;
        double rightVolt = (driveOutput + turnCorrection)* 12;

        setDrive(minVolt(leftVolt), minVolt(rightVolt));


        // 6. Exit Conditions
        bool stoppedMoving = fabs(remainingDistance - lastRemaining) < 0.02; // Check velocity near target
        bool closeToTarget = remainingDistance < 0.5; // Within 0.5 inches
        
        // Final Heading Check (only relevant if finalHeading was provided)
        double finalHeadingError = currentTargetHeading - pose.theta;
        while (finalHeadingError > 180) finalHeadingError -= 360;
        while (finalHeadingError < -180) finalHeadingError += 360;
        bool finalHeadingGood = fabs(finalHeadingError) < 2.0;
        
        bool timedOut = elapsed > timeout;

        // If performing final turn, need both position AND heading to be good
        if (timedOut || (closeToTarget && (performingFinalTurn ? finalHeadingGood : true)))
            break;
        
        lastRemaining = remainingDistance;
        elapsed += 10;
        wait(10, msec);
    }

    stops();
}

// Convenience wrapper functions
void moveToPoint(double targetX, double targetY, double timeout)
{
    // Drives to X, Y but stops facing the point
    moveToPose(targetX, targetY, -1.0, timeout); 
}



*/