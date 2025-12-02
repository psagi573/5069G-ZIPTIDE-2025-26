#include "motion.h"
#include "pid.h"
#include "profile.h"
#include "robot-config.h"
#include <cmath>
#include <algorithm>
#include "odometry.h"
#include "BOOM.h"
#include "PTOManager.h"



using namespace vex;

const double wheelTrack = 11.5; // in inches (left-right distance)

// PID class values (Existing values)
PID distPID(0.15, 0, 0.99);
PID fastTurnPID(0.03, 0.0001, 0.3);

PID arcPID(0.15, 0, 0);
PID sweepPID(0.05, 0, 0.5);

PID drivePID(0.05, 0, 0.6);
PID headingPID(0.043, 0.0001, 0.39);


/**
 * @brief Clamps a value between a minimum and maximum.
 */
template <typename T>
T clamp(T value, T minVal, T maxVal)
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}



// Global or external PTO instance
extern PTOManager pto;

// PTO–aware drive function
void setDrivePTO(double left, double right) {
    left = clamp(left, -12.0, 12.0);
    right = clamp(right, -12.0, 12.0);

    // Get only motors engaged in the current configuration
    auto leftMotors = pto.getActiveLeftMotors();
    auto rightMotors = pto.getActiveRightMotors();

    for (auto* m : leftMotors)
        m->spin(vex::forward, left, vex::volt);

    for (auto* m : rightMotors)
        m->spin(vex::forward, right, vex::volt);
}

void stopsPTO() {
    auto leftMotors = pto.getActiveLeftMotors();
    auto rightMotors = pto.getActiveRightMotors();

    for (auto* m : leftMotors)
        m->stop(vex::brake);

    for (auto* m : rightMotors)
        m->stop(vex::brake);
}

/**
 * @brief Sets left and right motor voltages.
 */
void setDrive(double left, double right)
{
    // Clamp values for safety
    left = clamp(left, -12.0, 12.0);
    right = clamp(right, -12.0, 12.0);

    // Send voltage to motors (volt units)
    L1.spin(fwd, left, volt);
    L2.spin(fwd, left, volt);
    PTOL3.spin(fwd, left, volt);

    R6.spin(fwd, right, volt);
    R7.spin(fwd, right, volt);
    PTOR8.spin(fwd, right, volt);
}


/**
 * @brief Stops all drive motors with brake mode.
 */
void stops()
{
    // Stop all motors
    L1.stop(brake);
    L2.stop(brake);
    PTOL3.stop(brake);
    R6.stop(brake);
    R7.stop(brake);
    PTOR8.stop(brake);
}

/**
 * @brief Applies a minimum voltage threshold to overcome motor friction.
 */
double minVolt(double v)
{
    const double MIN_V = 2.0;
    if (v > -MIN_V && v < 0)
        return -MIN_V;
    if (v > 0 && v < MIN_V)
        return MIN_V;
    return v;
}

const float WHEEL_CIRCUMFERENCE = M_PI * 3.25;               // ~10.21 inches
const float INCHES_PER_MOTOR_TURN = WHEEL_CIRCUMFERENCE * 0.8; // Gear ratio is 48:60, so multiply by 0.8

/**
 * @brief Calculates the average distance traveled based on motor rotations.
 */
float getAverageDistance()
{
    // Get positions from all motors (in turns/rotations)
    float left1_pos = L1.position(turns);
    float left2_pos = L2.position(turns);
    float left3_pos = PTOL3.position(turns);
    float right1_pos = R6.position(turns);
    float right2_pos = R7.position(turns);
    float right3_pos = PTOR8.position(turns);

    // Calculate average motor rotations
    float avg_rotations = (left1_pos + left2_pos + left3_pos +
                           right1_pos + right2_pos + right3_pos) /
                          6.0;

    // Convert average motor rotations to inches traveled
    float distance_inches = avg_rotations * INCHES_PER_MOTOR_TURN;

    return distance_inches;
}

/**
 * @brief Drives the robot a linear distance using motor encoders and IMU heading stabilization.
 */
void drive(double distInches, double timeout)
{
    distPID.reset();
    double target = distInches-2.5; 
    Odom::Pose startPose = Odom::getPose();
    double startX = startPose.x;
    double startY = startPose.y;
    double lastError = 0;
    int elapsed = 0;

    while (true)
    {
        // Current pose and distance traveled
        Odom::Pose pose = Odom::getPose();
        double dx = pose.x - startX;
        double dy = pose.y - startY;
        double traveled = sqrt(dx * dx + dy * dy);
        
        if (distInches < 0)
        {
            traveled = traveled; 
        }

        double error = target - traveled;
        // Compute linear output (PID)
        double linearOut = distPID.compute(target, traveled);
        // Clamp output between -1 and 1
        linearOut = clamp(linearOut, -1.0, 1.0);
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

/**
 * @brief Turns the robot to a target absolute heading.
 */
void turn(double targetHeading)
{
    fastTurnPID.reset();
    double elapsedTime = 0;
    const double timeout = 1500; // ms timeout

    while (true)
    {
        // Assuming inertial19 is defined in robot-config.h
        double heading = inertial19.heading(); 

        // Proper angle difference calculation
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

/**
 * @brief Executes a wide, smooth arc path.
 */
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
    Odom::Pose startPose = Odom::getPose();

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
        Odom::Pose pose = Odom::getPose();

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

        // Correct arc direction logic
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

/**
 * @brief Performs a sweep turn (pivots one side only).
 */
void Sweep(double targetAngleDeg, bool left)
{
    sweepPID.reset();
    double elapsedTime = 0;
    const double timeout = 2000; // ms timeout

    while (true)
    {
        // Current pose and heading
        Odom::Pose pose = Odom::getPose();
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


/* ============================
   Boomerang-based motion funcs
   ============================ */

// Blocking move to point using Boomerang controller
bool moveToPointBoomerang(double tx, double ty, int timeoutMs) {
    // Create controller with default config
    Boomerang::Config cfg;
    Boomerang::Controller ctrl(cfg);

    // Set target pose; pass current heading as target heading (unused in XY stage)
    ctrl.setTarget(tx, ty, 0.0);

    int elapsed = 0;
    const int dt = 10; // ms loop period (matches rest of file)
    // Main loop
    while (elapsed < timeoutMs) {
        // Get current pose
        Odom::Pose p = Odom::getPose();

        // Run controller update
        ctrl.update(p.x, p.y, p.theta);

        // Apply outputs through existing setDrive (uses volt units)
        setDrivePTO(ctrl.outL, ctrl.outR);

        // Check settled (controller sets reachedXY and will run final-turn when close)
        if (ctrl.isSettled()) {
            // Stop motors and return success
            stopsPTO();
            return true;
        }

        // tick
        wait(dt, msec);
        elapsed += dt;
    }

    // Timeout
    stopsPTO();
    return false;
}

// Blocking move to pose using Boomerang controller
bool moveToPoseBoomerang(double tx, double ty, double thetaDeg, int timeoutMs) {
    Boomerang::Config cfg;
    Boomerang::Controller ctrl(cfg);

    // Set target pose with desired final heading
    ctrl.setTarget(tx, ty, thetaDeg);

    int elapsed = 0;
    const int dt = 10; // ms loop period

    while (elapsed < timeoutMs) {
        Odom::Pose p = Odom::getPose();

        // run controller (it will handle final-turn once XY reached)
        ctrl.update(p.x, p.y, p.theta);

        // send volt commands
        setDrivePTO(ctrl.outL, ctrl.outR);

        // if controller reached XY AND the heading error is within tolerance -> success
        if (ctrl.isSettled()) {
            // check final heading error
            double remaining = thetaDeg - p.theta;
            while (remaining > 180) remaining -= 360;
            while (remaining < -180) remaining += 360;

            if (fabs(remaining) <= 3.0) { // 3 deg tolerance (matches earlier config)
                stops();
                return true;
            }
        }

        wait(dt, msec);
        elapsed += dt;
    }

    // timeout
    stopsPTO();
    return false;
}
