#include "odometry.h"
#include <mutex>

namespace Odom {

// ----------------------------------------------------
// --- Tuning Constants ---
// ----------------------------------------------------

// 1. Wheel Diameter (in inches)
const double WHEEL_DIAMETER = 3.15; 

// 2. Drive Gear Ratio (Motor Turns / Wheel Turns) -> e.g., (30 / 48) = 0.625
const double DRIVE_GEAR_RATIO = 30.0 / 48.0;

// 3. Tracking Width (in inches). Distance between the center of the left and right wheels.
const double TRACKING_WIDTH = 13.0; 

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
        
        // 🚨 DUAL IMU STEP: Get raw heading from both and average them
        double imu1Deg = imuSensor1->rotation();
        double imu2Deg = imuSensor2->rotation();
        double currThetaDegRaw = (imu1Deg + imu2Deg) / 2.0; // The NEW primary raw heading
        
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

        double deltaXGlobal = 0.0;
        double deltaYGlobal = 0.0;

        // Use a small epsilon (1e-6) to check if the robot moved in a straight line
        if (fabs(deltaThetaFusion) < 1e-6) { 
            // Case 1: Straight Line Motion (Use Midpoint Approximation)
            double avgThetaRad = runningThetaRad + (deltaThetaFusion / 2.0); 
            deltaXGlobal = deltaS * cos(avgThetaRad);
            deltaYGlobal = deltaS * sin(avgThetaRad);
        } else {
            // Case 2: Arc Motion (Use Pure Rotation Model)
            double radius = deltaS / deltaThetaFusion;

            // These formulas convert local arc distance to global displacement
            double sinStart = sin(runningThetaRad);
            double cosStart = cos(runningThetaRad);
            double sinEnd = sin(runningThetaRad + deltaThetaFusion);
            double cosEnd = cos(runningThetaRad + deltaThetaFusion);
            
            deltaXGlobal = radius * (sinEnd - sinStart);
            deltaYGlobal = -radius * (cosEnd - cosStart);
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
        prevThetaDegRaw = currThetaDegRaw; // Store the new *averaged* raw reading

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

    if (!imuSensor1->installed() || !imuSensor2->installed()) {
        Brain.Screen.print("ODOM ERROR: IMU(s) NOT DETECTED! Check ports.");
        return;
    }

    // Calibrate BOTH IMUs
    Brain.Screen.print("Calibrating BOTH IMUs... DO NOT TOUCH ROBOT");
    imuSensor1->calibrate();
    imuSensor2->calibrate();
    
    // Wait for both to finish
    while (imuSensor1->isCalibrating() || imuSensor2->isCalibrating()) {
        vex::wait(100, msec);
    }
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1,1);

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
        
        // Set the IMU's angle to the new starting angle
        imuSensor1->setRotation(theta, degrees);
        imuSensor2->setRotation(theta, degrees);

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

} // namespace Odom



// #include "vex.h"
// #include "odometry.h"
// #include <mutex>

// using namespace vex;

// // Internal position state
// Pose currentPose = {0.0, 0.0, 0.0};

// // Odometer thread
// thread odomThread;
// bool odomRunning = false;

// // Sensor pointers
// rotation *xRot = nullptr;
// rotation *yRot = nullptr;
// inertial *imuSensor = nullptr;

// // Thread safety
// mutex poseMutex;

// // Robot sensor offsets - adjust these for your robot!
// double xOffset = 0.75; // inches from center to X (lateral) tracker 0.75
// double yOffset = 6.5;  // inches from center to Y (longitudinal) tracker 6.5

// // Wheel settings
// double odomWheelDiameter = 2.0;
// double wheelCircumference = odomWheelDiameter * M_PI;

// // Previous readings
// double prevX = 0;
// double prevY = 0;
// double prevThetaRad = 0;

// // Odometer update task
// int odomTask()
// {
//     while (odomRunning)
//     {

//         // 1. Get current sensor positions
//         double currXTurns = xRot->position(turns);
//         double currYTurns = yRot->position(turns);
//         double thetaDeg = imuSensor->rotation();
//         double thetaRad = thetaDeg * (M_PI / 180.0);

//         // 2. Convert encoder readings to inches
//         double currXInches = currXTurns * wheelCircumference;
//         double currYInches = currYTurns * wheelCircumference;

//         // 3. Calculate raw deltas in robot-local coordinates
//         double deltaY = currXInches - prevX;
//         double deltaX = currYInches - prevY;

//         // 4. Calculate heading delta and wrap to [-π, π]
//         double deltaTheta = thetaRad - prevThetaRad;
//         while (deltaTheta > M_PI)
//             deltaTheta -= 2.0 * M_PI;
//         while (deltaTheta < -M_PI)
//             deltaTheta += 2.0 * M_PI;

//         // 5. Compute local displacement with arc correction (Purdue method)
//         double localX = 0.0, localY = 0.0;

//         if (fabs(deltaTheta) < 1e-3)
//         {
//             // Robot moved nearly straight: no arc correction needed
//             localX = deltaX;
//             localY = deltaY;
//         }
//         else
//         {
//             // Arc-based displacement calculation with tracker offsets
//             double rFactor = 2.0 * sin(deltaTheta / 2.0) / deltaTheta;
//             localX = rFactor * (deltaX + xOffset * deltaTheta);
//             localY = rFactor * (deltaY + yOffset * deltaTheta);
//         }

//         // 6. Rotate local displacement to global coordinates using average heading
//         double avgTheta = prevThetaRad + deltaTheta / 2.0;
//         double cosT = cos(avgTheta);
//         double sinT = sin(avgTheta);

//         double deltaXGlobal = localX * cosT - localY * sinT;
//         double deltaYGlobal = localX * sinT + localY * cosT;

//         // 7. Update global pose with thread-safe mutex lock
//         poseMutex.lock();
//         currentPose.x += deltaXGlobal;
//         currentPose.y += deltaYGlobal;
//         currentPose.theta = fmod(thetaDeg + 360.0, 360.0);
//         poseMutex.unlock();

//         // 8. Store current readings for next iteration
//         prevX = currXInches;
//         prevY = currYInches;
//         prevThetaRad = thetaRad;

//         wait(10, msec); // Update at 100Hz
//     }
//     return 0;
// }

// bool startOdom(rotation &xSensor, rotation &ySensor, inertial &imu)
// {
//     // Stop if already running
//     if (odomRunning)
//     {
//         stopOdom();
//     }

//     // Validate sensors
//     if (!xSensor.installed() || !ySensor.installed() || !imu.installed())
//     {
//         Controller1.Screen.print("ODOM: Sensor error!");
//         Brain.Screen.print("ODOM: Sensor not installed!");
//         return false;
//     }

//     xRot = &xSensor;
//     yRot = &ySensor;
//     imuSensor = &imu;

//     // Calibrate IMU with timeout
//     // imu.calibrate();
//     // int calibrationWait = 0;
//     // const int CALIBRATION_TIMEOUT = 3000; // 3 seconds

//     // while (imu.isCalibrating() && calibrationWait < CALIBRATION_TIMEOUT)
//     // {
//     //     wait(50, msec);
//     //     calibrationWait += 50;
//     //     Controller1.Screen.print("Calibrating... %d", calibrationWait / 50);
//     // }

//     // if (imu.isCalibrating())
//     // {
//     //     Controller1.Screen.print("ODOM: IMU calib timeout!");
//     //     Brain.Screen.print("ODOM: IMU calibration failed!");
//     //     return false;
//     // }

//     // Reset sensors
//     xRot->resetPosition();
//     yRot->resetPosition();
//     imuSensor->resetRotation();

//     // Initialize previous readings
//     prevX = xRot->position(turns) * wheelCircumference;
//     prevY = yRot->position(turns) * wheelCircumference;
//     prevThetaRad = imuSensor->rotation() * (M_PI / 180.0);

//     // Reset pose
//     setCurrentPose(0, 0, 0);

//     // Start thread
//     odomRunning = true;
//     odomThread = thread(odomTask);

//     wait(100, msec); // Allow thread to start

//     return true;
// }

// bool startOdomAt(rotation &xSensor, rotation &ySensor, inertial &imu,
//                  double startX, double startY, double startTheta)
// {
//     if (!startOdom(xSensor, ySensor, imu))
//     {
//         return false;
//     }

//     setCurrentPose(startX, startY, startTheta);
//     return true;
// }

// void stopOdom()
// {
//     if (odomRunning)
//     {
//         odomRunning = false;
//         if (odomThread.joinable())
//         {
//             odomThread.join();
//         }
//     }

//     xRot = nullptr;
//     yRot = nullptr;
//     imuSensor = nullptr;

//     Controller1.Screen.print("Odom stopped!");
// }

// void resetOdom()
// {
//     setCurrentPose(0, 0, 0);
//     if (xRot)
//         xRot->resetPosition();
//     if (yRot)
//         yRot->resetPosition();
//     if (imuSensor)
//         imuSensor->resetRotation();

//     // Re-initialize previous readings
//     if (xRot && yRot && imuSensor)
//     {
//         prevX = xRot->position(turns) * wheelCircumference;
//         prevY = yRot->position(turns) * wheelCircumference;
//         prevThetaRad = imuSensor->rotation() * (M_PI / 180.0);
//     }
// }

// Pose getPose()
// {
//     Pose poseCopy;
//     poseMutex.lock();
//     poseCopy = currentPose;
//     poseMutex.unlock();
//     return poseCopy;
// }

// void setCurrentPose(double x, double y, double theta)
// {
//     poseMutex.lock();
//     currentPose.x = x;
//     currentPose.y = y;
//     currentPose.theta = fmod(theta + 360.0, 360.0); // Normalize to 0-360
//     poseMutex.unlock();

//     // Reset previous readings to avoid large deltas after pose set
//     if (xRot && yRot && imuSensor)
//     {
//         prevX = xRot->position(turns) * wheelCircumference;
//         prevY = yRot->position(turns) * wheelCircumference;
//         prevThetaRad = theta * (M_PI / 180.0);
//     }
// }

// void printPose()
// {
//     Pose p = getPose();
//     Brain.Screen.clearScreen();
//     Brain.Screen.setCursor(1, 1);
//     Brain.Screen.print("X: %.2f, Y: %.2f", p.x, p.y);
//     Brain.Screen.setCursor(2, 1);
//     Brain.Screen.print("Theta: %.2f deg", p.theta);

//     // Also print to controller
//     Controller1.Screen.clearScreen();
//     Controller1.Screen.setCursor(1, 1);
//     Controller1.Screen.print("X:%.1f Y:%.1f", p.x, p.y);
//     Controller1.Screen.setCursor(2, 1);
//     Controller1.Screen.print("T:%.1f deg", p.theta);
// }

// void getRawSensorReadings(double &xInches, double &yInches, double &headingDeg)
// {
//     if (xRot && yRot && imuSensor)
//     {
//         xInches = xRot->position(turns) * wheelCircumference;
//         yInches = yRot->position(turns) * wheelCircumference;
//         headingDeg = imuSensor->rotation();
//     }
//     else
//     {
//         xInches = 0;
//         yInches = 0;
//         headingDeg = 0;
//     }
// }

// void tuneOffsets(double newXOffset, double newYOffset)
// {
//     xOffset = newXOffset;
//     yOffset = newYOffset;
//     Controller1.Screen.print("Offsets: X=%.2f Y=%.2f", xOffset, yOffset);
// }

// void setWheelDiameter(double newDiameter)
// {
//     odomWheelDiameter = newDiameter;
//     wheelCircumference = odomWheelDiameter * M_PI;
//     Controller1.Screen.print("Wheel diam: %.2f", odomWheelDiameter);
// }