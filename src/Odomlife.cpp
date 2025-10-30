#include "odometry.h"
#include <mutex>

namespace Odom {

// ----------------------------------------------------
// ---  TUNE THESE CRITICAL CONSTANTS  ---
// ----------------------------------------------------

// 1. Wheel Diameter (in inches)
const double WHEEL_DIAMETER = 2.75; 

// 2. Drive Gear Ratio (Motor Turns / Wheel Turns) -> e.g., (30 / 48) = 0.625
const double DRIVE_GEAR_RATIO = 38.0 / 48.0;

// 3. Tracking Width (in inches). Distance between the center of the left and right wheels.
const double TRACKING_WIDTH = 11.625; 

// 4. Sensor Fusion Weights (IMU is generally more reliable for rotation)
const double IMU_WEIGHT = 0.9;
const double ENC_WEIGHT = 0.1;

// ----------------------------------------------------

// Derived Constant
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

// Internal State
static Pose currentPose = {0.0, 0.0, 0.0};
static vex::mutex odomMutex;
static vex::thread odomThread;
static bool isRunning = false;

// Hardware Pointers
static vex::motor_group *leftDrive = nullptr;
static vex::motor_group *rightDrive = nullptr;
static vex::inertial *imuSensor1 = nullptr; // Pointer for IMU 1
static vex::inertial *imuSensor2 = nullptr; // Pointer for IMU 2

// Previous sensor values for delta calculation
static double prevLeftTurns = 0.0;
static double prevRightTurns = 0.0;

// Tracking variables for wrap-around correction and fusion
static double runningThetaRad = 0.0; // The total integrated angle in radians
static double prevThetaDegRaw = 0.0; // The AVERAGE raw IMU reading from the last tick


/**
 * @brief The core odometry tracking task. Runs in a separate thread.
 */
int odomTask() {
    while (isRunning) {
        // 1. Get current sensor values
        double currLeftTurns = leftDrive->position(turns);
        double currRightTurns = rightDrive->position(turns);
        
        // DUAL IMU STEP: Get raw heading from both and average them
        double imu1Deg = imuSensor1->rotation();
        double imu2Deg = imuSensor2->rotation();
        double currThetaDegRaw = (imu1Deg + imu2Deg) / 2.0; 
        
        // 2. Calculate deltas (change since last tick)
        double deltaLeftTurns = currLeftTurns - prevLeftTurns;
        double deltaRightTurns = currRightTurns - prevRightTurns;

        // 3. Convert motor turns to distance in inches
        double deltaL_inches = deltaLeftTurns * DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE;
        double deltaR_inches = deltaRightTurns * DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE;

        // 4. Calculate Heading Deltas
        
        // A. IMU Delta (with Wrap Correction)
        double deltaThetaDegIMU = currThetaDegRaw - prevThetaDegRaw;
        // Normalize delta to be between -180 and 180 degrees
        while (deltaThetaDegIMU > 180.0) deltaThetaDegIMU -= 360.0;
        while (deltaThetaDegIMU < -180.0) deltaThetaDegIMU += 360.0;
        double deltaThetaRadIMU = deltaThetaDegIMU * (M_PI / 180.0);

        // B. Encoder Delta (from differential drive)
        double deltaThetaRadEnc = (deltaR_inches - deltaL_inches) / TRACKING_WIDTH;

        // C. Sensor Fusion
        double deltaThetaFusion = (deltaThetaRadIMU * IMU_WEIGHT) + (deltaThetaRadEnc * ENC_WEIGHT);

        // 5. Calculate average forward displacement (robot-local Y)
        double deltaS = (deltaL_inches + deltaR_inches) / 2.0;
        
        // 6. Calculate global displacement using Arc Correction (Pure Rotation Model)
        // CUSTOM COORDINATE SYSTEM: 0 degrees -> +Y Axis

        double deltaXGlobal = 0.0;
        double deltaYGlobal = 0.0;
        double runningTheta = runningThetaRad; 

        // Use a small epsilon (1e-6) to check if the robot moved in a straight line
        if (fabs(deltaThetaFusion) < 1e-6) { 
            // Case 1: Straight Line Motion (Midpoint Approximation)
            double avgThetaRad = runningThetaRad + (deltaThetaFusion / 2.0); 
            
            // SWAPPED TRIG: SIN tracks X, COS tracks Y
            // Y-axis is correct: COS for Y
            deltaYGlobal = deltaS * cos(avgThetaRad); 
            //  FIX: X-axis sign flip: SIN for X
            deltaXGlobal = deltaS * sin(avgThetaRad); 
        } else {
            // Case 2: Arc Motion (Pure Rotation Model)
            double radius = deltaS / deltaThetaFusion;

            // Rotational displacement components (standard trig: X=cos, Y=sin)
            double sinStart = sin(runningTheta);
            double cosStart = cos(runningTheta);
            double sinEnd = sin(runningTheta + deltaThetaFusion);
            double cosEnd = cos(runningTheta + deltaThetaFusion);
            
            // dx_rotational is change along the X-axis (perpendicular to start heading)
            double dx_rotational = radius * (sinEnd - sinStart);
            // dy_rotational is change along the Y-axis (parallel to start heading)
            double dy_rotational = -radius * (cosEnd - cosStart);

            // ASSIGNMENT SWAP:
            // fix: Remove the negative sign to flip the X-axis back
            deltaXGlobal = dy_rotational; 
            // Y-axis is correct:
            deltaYGlobal = dx_rotational;  
        }

        // 7. Update running angle and global pose (thread-safe)
        runningThetaRad += deltaThetaFusion;

        odomMutex.lock();
        currentPose.x += deltaXGlobal;
        currentPose.y += deltaYGlobal;
        
        // Convert the running total angle back to degrees (0-360 normalized)
        double thetaDeg = runningThetaRad * (180.0 / M_PI);
        currentPose.theta = fmod(thetaDeg + 360.0, 360.0); 
        
        odomMutex.unlock();

        // 8. Store current values for next loop
        prevLeftTurns = currLeftTurns;
        prevRightTurns = currRightTurns;
        prevThetaDegRaw = currThetaDegRaw; 

        vex::wait(10, msec); // Run at 100Hz
    }
    return 0;
}

// =========================================================
// Public Functions (start, stop, getPose, setPose)
// =========================================================

void start(vex::motor_group& left, vex::motor_group& right, vex::inertial& imu1, vex::inertial& imu2) {
    if (isRunning) stop();

    leftDrive = &left;
    rightDrive = &right;
    imuSensor1 = &imu1;
    imuSensor2 = &imu2;


    // Reset all sensor positions and the pose
    setPose(0.0, 0.0, 0.0);

    isRunning = true;
    odomThread = vex::thread(odomTask);
}

void stop() {
    if (isRunning) {
        isRunning = false;
        if (odomThread.joinable()) {
            odomThread.join();
        }
    }
}

Pose getPose() {
    Pose tempPose;
    odomMutex.lock();
    tempPose = currentPose;
    odomMutex.unlock();
    return tempPose;
}

void setPose(double x, double y, double theta) {
    odomMutex.lock();
    currentPose.x = x;
    currentPose.y = y;
    currentPose.theta = fmod(theta + 360.0, 360.0); // Normalize output theta
    odomMutex.unlock();

    // Reset previous values to synchronize hardware and tracking state
    if (leftDrive && rightDrive && imuSensor1 && imuSensor2) {
        leftDrive->resetPosition();
        rightDrive->resetPosition();
        
        //  CRITICAL IMU ALIGNMENT: Adjust IMU rotation to align the physical FORWARD (Y-axis) 
        // with the desired starting angle (theta). -90 degrees is the most common fix.
        double finalImuAngle = theta - 90.0; 
        
        imuSensor1->setRotation(finalImuAngle, degrees); 
        imuSensor2->setRotation(finalImuAngle, degrees); 

        // Synchronize tracking variables
        prevLeftTurns = 0;
        prevRightTurns = 0;
        
        // Calculate the new averaged raw reading after setting the rotation
        double avgImuRot = (imuSensor1->rotation() + imuSensor2->rotation()) / 2.0;
        prevThetaDegRaw = avgImuRot; 
        runningThetaRad = theta * (M_PI / 180.0);
    }
}

void setPose(Pose pose) {
    setPose(pose.x, pose.y, pose.theta);
}

} 