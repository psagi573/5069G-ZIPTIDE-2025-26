#include "odometry.h"

namespace Odom {

// --- Configuration Constants ---
// Drive Geometry
const double DRIVE_WHEEL_DIAMETER = 3.25; // 3.25 inches
const double DRIVE_CIRCUMFERENCE = DRIVE_WHEEL_DIAMETER * M_PI;

// Gear Ratio: 48:60 (60/48 = 1.25). Reduction factor.
const double GEAR_RATIO_REDUCTION = 1.25;

// Conversion Factor: Inches traveled per degree of motor rotation.
// Calculation: Circumference / (360 * Gear Ratio)
const double MOTOR_DEGREES_TO_INCHES = DRIVE_CIRCUMFERENCE / (360.0 * GEAR_RATIO_REDUCTION); 

// The distance between the centers of the left and right drive wheels (TRACKING WIDTH)
// *** THIS VALUE MUST BE PHYSICALLY MEASURED AND CAREFULLY TUNED ***
const double TRACKING_WIDTH = 12.0; // Placeholder in inches

// --- Internal State and Mutex ---
Pose currentPose;
vex::motor_group* leftMotors;
vex::motor_group* rightMotors;
vex::inertial* imuSensor;

double prevLPosDeg = 0.0;
double prevRPosDeg = 0.0;
double runningThetaRad = 0.0; // Internal running heading in radians

bool isRunning = false;
vex::thread odomThread;
vex::mutex poseMutex;

// --- Helper Functions ---
// Normalizes angle to [-pi, pi] radians
static double wrapAngleRad(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ------------------------
// Odometry Loop Task
// ------------------------

int odomTask() {
    
    // Initial setup: reset encoders
    leftMotors->resetPosition();
    rightMotors->resetPosition();
    prevLPosDeg = 0.0;
    prevRPosDeg = 0.0;

    // Wait for the IMU to finish calibrating before starting
    while (imuSensor->isCalibrating()) {
        vex::wait(25, vex::msec);
    }
    
    // Set initial pose
    poseMutex.lock();
    runningThetaRad = currentPose.theta * (M_PI / 180.0);
    imuSensor->setRotation(currentPose.theta, vex::rotationUnits::deg);
    poseMutex.unlock();

    while(isRunning) {
        
        // 1. Get current encoder values and IMU reading
        double currLPosDeg = leftMotors->position(vex::rotationUnits::deg);
        double currRPosDeg = rightMotors->position(vex::rotationUnits::deg);
        
        // 2. Calculate distance traveled since last cycle (dL and dR in inches)
        double dL = (currLPosDeg - prevLPosDeg) * MOTOR_DEGREES_TO_INCHES;
        double dR = (currRPosDeg - prevRPosDeg) * MOTOR_DEGREES_TO_INCHES;
        
        // 3. Compute heading change (from IMU)
        double imuDeg = imuSensor->rotation(); 
        double currThetaRad = imuDeg * (M_PI / 180.0);
        
        double deltaThetaRad = currThetaRad - runningThetaRad;
        deltaThetaRad = wrapAngleRad(deltaThetaRad); 
        
        // 4. Calculate Local Movement
        double deltaS_local = (dL + dR) / 2.0; // Average forward/backward motion

        // 5. Arc Correction (Translating local delta to global delta)
        double deltaX_global, deltaY_global;

        if(fabs(deltaThetaRad) < 1e-6) {
            // Case 1: Straight Line Motion (Midpoint Approximation)
            double avgThetaRad = runningThetaRad + deltaThetaRad / 2.0;
            
            // Standard projection onto global axes (0 deg = +Y convention)
            deltaX_global = deltaS_local * sin(avgThetaRad);
            deltaY_global = deltaS_local * cos(avgThetaRad);

        } else {
            // Case 2: Arc Motion (Pure Rotation Model)
            
            // R = Radius of the arc based on the average distance traveled
            double R = deltaS_local / deltaThetaRad;

            // Arc Model for 0 deg = +Y (Complex formulas derived from differential geometry)
            double term1 = 1.0 - cos(deltaThetaRad);
            double term2 = sin(deltaThetaRad);

            deltaX_global = R * (sin(runningThetaRad) * term1 + cos(runningThetaRad) * term2);
            deltaY_global = R * (cos(runningThetaRad) * term1 - sin(runningThetaRad) * term2);
        }

        // 6. Update pose (Protected by mutex)
        poseMutex.lock();
        currentPose.x += deltaX_global;
        currentPose.y += deltaY_global;
        currentPose.theta = imuDeg; // Use the raw IMU reading for the public theta (degrees)
        poseMutex.unlock();

        // 7. Prepare for next cycle
        runningThetaRad = currThetaRad;
        prevLPosDeg = currLPosDeg;
        prevRPosDeg = currRPosDeg;

        vex::wait(10, vex::msec); // 100Hz loop frequency
    }
    return 0;
}

// ------------------------
// Public Function Definitions
// ------------------------

void start(vex::motor_group* lm, vex::motor_group* rm, vex::inertial* imu) {
    if (isRunning) return;
    leftMotors = lm;
    rightMotors = rm;
    imuSensor = imu;
    isRunning = true;
    odomThread = vex::thread(odomTask);
}

void stop() {
    isRunning = false;
    if (odomThread.joinable()) {
        odomThread.join();
    }
}

Pose getPose() {
    poseMutex.lock();
    Pose p = currentPose;
    poseMutex.unlock();
    return p;
}

void setPose(Pose newPose) {
    poseMutex.lock();
    currentPose = newPose;
    // Update internal running variables
    runningThetaRad = newPose.theta * (M_PI / 180.0);
    if (imuSensor) imuSensor->setRotation(newPose.theta, vex::rotationUnits::deg);
    // Reset motor state
    if (leftMotors) leftMotors->resetPosition();
    if (rightMotors) rightMotors->resetPosition();
    prevLPosDeg = 0.0;
    prevRPosDeg = 0.0;
    poseMutex.unlock();
}

} // namespace Odom