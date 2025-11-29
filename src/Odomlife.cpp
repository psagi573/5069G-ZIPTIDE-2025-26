#include "odometry.h"
#include <cmath>

namespace Odom {

// ------------------------
// Constants (tune these)
// ------------------------
const double WHEEL_DIAMETER = 3.25; // inches
const double DRIVE_GEAR_RATIO = 48.0/60.0; 
const double TRACKING_WIDTH = 11.75; // inches

const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;

// ------------------------
// Internal State
// ------------------------
static Pose currentPose;
static vex::mutex odomMutex;
static vex::thread odomThread;
static bool isRunning = false;

static std::vector<vex::motor*> leftDrive;
static std::vector<vex::motor*> rightDrive;
static std::vector<bool> leftActive;
static std::vector<bool> rightActive;

static vex::inertial* imuSensor = nullptr;

static std::vector<double> prevLeftTurns;
static std::vector<double> prevRightTurns;

static double runningThetaRad = 0.0;

// ------------------------
// Helper Functions
// ------------------------
double avgActiveMotorPos(const std::vector<vex::motor*>& motors,
                         const std::vector<bool>& active,
                         const std::vector<double>& prevTurns,
                         std::vector<double>& outDeltas) {
    double total = 0;
    int count = 0;
    outDeltas.clear();
    for(size_t i=0;i<motors.size();i++) {
        if(active[i]) {
            double pos = motors[i]->position(turns);
            double delta = pos - prevTurns[i];
            outDeltas.push_back(delta);
            total += delta;
            count++;
        } else {
            outDeltas.push_back(0.0);
        }
    }
    return (count > 0) ? (total / count) : 0.0;
}

// ------------------------
// Core Odometry Task
// ------------------------
int odomTask() {
    while(isRunning) {

        // 1. Average active motor delta
        std::vector<double> leftDeltas, rightDeltas;
        double deltaLeftTurns = avgActiveMotorPos(leftDrive, leftActive, prevLeftTurns, leftDeltas);
        double deltaRightTurns = avgActiveMotorPos(rightDrive, rightActive, prevRightTurns, rightDeltas);

        // 2. Convert to inches
        double deltaL_inches = deltaLeftTurns * DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE;
        double deltaR_inches = deltaRightTurns * DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE;

        // 3. Compute heading from IMU
        double imuDeg = imuSensor->heading();
        double deltaThetaRad = (imuDeg * M_PI/180.0) - runningThetaRad;

        // Wrap delta -pi to pi
        while(deltaThetaRad > M_PI) deltaThetaRad -= 2*M_PI;
        while(deltaThetaRad < -M_PI) deltaThetaRad += 2*M_PI;

        // 4. Forward movement
        double deltaS = (deltaL_inches + deltaR_inches)/2.0;

        // 5. Global X/Y update
        double deltaX, deltaY;
        if(fabs(deltaThetaRad) < 1e-6) {
            // straight
            double avgTheta = runningThetaRad + deltaThetaRad/2.0;
            deltaX = deltaS * sin(avgTheta);
            deltaY = deltaS * cos(avgTheta);
        } else {
            // arc
            double radius = deltaS/deltaThetaRad;
            double sinStart = sin(runningThetaRad);
            double cosStart = cos(runningThetaRad);
            double sinEnd = sin(runningThetaRad + deltaThetaRad);
            double cosEnd = cos(runningThetaRad + deltaThetaRad);

            deltaX = radius*(sinEnd - sinStart);
            deltaY = radius*(cosEnd - cosStart);
        }

        // 6. Update pose
        odomMutex.lock();
        currentPose.x += deltaX;
        currentPose.y += deltaY;
        currentPose.theta = fmod(imuDeg + 360.0, 360.0);
        odomMutex.unlock();

        runningThetaRad += deltaThetaRad;

        // 7. Store previous positions
        for(size_t i=0;i<leftDrive.size();i++) prevLeftTurns[i] += leftDeltas[i];
        for(size_t i=0;i<rightDrive.size();i++) prevRightTurns[i] += rightDeltas[i];

        vex::wait(10, vex::msec); // 100Hz
    }
    return 0;
}

// ------------------------
// Public Functions
// ------------------------
void start(const std::vector<vex::motor*>& leftMotors,
           const std::vector<vex::motor*>& rightMotors,
           vex::inertial* imu) {

    if(isRunning) stop();

    leftDrive = leftMotors;
    rightDrive = rightMotors;
    imuSensor = imu;

    leftActive.resize(leftDrive.size(), true);
    rightActive.resize(rightDrive.size(), true);

    prevLeftTurns.resize(leftDrive.size(), 0.0);
    prevRightTurns.resize(rightDrive.size(), 0.0);

    setPose(0,0,0);

    isRunning = true;
    odomThread = vex::thread(odomTask);
}

void stop() {
    if(isRunning) {
        isRunning = false;
        if(odomThread.joinable()) odomThread.join();
    }
}

Pose getPose() {
    Pose p;
    odomMutex.lock();
    p = currentPose;
    odomMutex.unlock();
    return p;
}

void setPose(double x, double y, double theta) {
    odomMutex.lock();
    currentPose.x = x;
    currentPose.y = y;
    currentPose.theta = fmod(theta + 360.0,360.0);
    odomMutex.unlock();
    runningThetaRad = theta*M_PI/180.0;

    // Reset encoders
    for(auto m:leftDrive) m->resetPosition();
    for(auto m:rightDrive) m->resetPosition();
    for(size_t i=0;i<leftDrive.size();i++) prevLeftTurns[i] = 0;
    for(size_t i=0;i<rightDrive.size();i++) prevRightTurns[i] = 0;

    // Reset IMU rotation to match theta
    imuSensor->setRotation(theta, degrees);
}

void setPose(Pose p) { setPose(p.x,p.y,p.theta); }

void setActiveMotors(const std::vector<bool>& leftA, const std::vector<bool>& rightA) {
    leftActive = leftA;
    rightActive = rightA;
}
}