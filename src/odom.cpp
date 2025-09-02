#include "vex.h"
#include "odom.h"

#include "vex.h"
#include "odom.h"

using namespace vex;

// Internal position state
Pose currentPose = {0.0, 0.0, 0.0};

// Odometer thread
thread odomThread;
bool odomRunning = false;

// Sensor pointers
rotation *xRot;
rotation *yRot;
inertial *imuSensor;

// Robot senser offsets
const double xOffset = -8;   // inches from center to X (lateral) tracker 0 inches
const double yOffset = -0.3; // inches from center to Y (longitudinal) tracker 1.9 inches

// Wheel settings
const double odomWheelDiameter = 2.0;
const double wheelCircumference = odomWheelDiameter * M_PI;

// Previous readings
double prevX = 0;
double prevY = 0;
double prevThetaRad = 0;

// Odometer update task
int odomTask()
{
    while (odomRunning)
    {
        // 1. Get current sensor positions
        double currXTurns = xRot->position(turns);
        double currYTurns = yRot->position(turns);
        double thetaDeg = imuSensor->rotation();
        double thetaRad = thetaDeg * (M_PI / 180.0);

        // 2. Convert encoder readings to inches
        double currXInches = currXTurns * wheelCircumference;
        double currYInches = currYTurns * wheelCircumference;

        // 3. Calculate raw deltas in robot-local coordinates
        double deltaX = currXInches - prevX;
        double deltaY = currYInches - prevY;

        // 4. Calculate heading delta and wrap to [-π, π]
        double deltaTheta = thetaRad - prevThetaRad;
        if (deltaTheta > M_PI)
            deltaTheta -= 2.0 * M_PI;
        if (deltaTheta < -M_PI)
            deltaTheta += 2.0 * M_PI;

        // 5. Compute local displacement with arc correction (Purdue method)
        double localX = 0.0, localY = 0.0;

        if (fabs(deltaTheta) < 1e-6)
        {
            // Robot moved nearly straight: no arc correction needed
            localX = deltaX;
            localY = deltaY;
        }
        else
        {
            // Arc-based displacement calculation with tracker offsets
            double rFactor = 2.0 * sin(deltaTheta / 2.0) / deltaTheta;
            localX = rFactor * (deltaX + xOffset * deltaTheta);
            localY = rFactor * (deltaY + yOffset * deltaTheta);
        }

        // 6. Rotate local displacement to global coordinates using average heading
        double avgTheta = prevThetaRad + deltaTheta / 2.0;
        double cosT = cos(avgTheta);
        double sinT = sin(avgTheta);

        double deltaXGlobal = localX * cosT - localY * sinT;
        double deltaYGlobal = localX * sinT + localY * cosT;

        // 7. Update global pose with thread-safe mutex lock
        {
            currentPose.x += deltaXGlobal;
            currentPose.y += deltaYGlobal;

            double cleanTheta = fmod(thetaDeg, 360.0);
            if (cleanTheta < 0.0)
                cleanTheta += 360.0;
            currentPose.theta = cleanTheta;
        }

        // 8. Store current readings for next iteration
        prevX = currXInches;
        prevY = currYInches;
        prevThetaRad = thetaRad;

        wait(10, msec); // Update at 100Hz
    }

    return 0;
}

void startOdom(rotation &xSensor, rotation &ySensor, inertial &imu)
{
    xRot = &xSensor;
    yRot = &ySensor;
    imuSensor = &imu;

    xRot->resetPosition();
    yRot->resetPosition();
    imuSensor->resetRotation();

    prevX = xRot->position(turns) * wheelCircumference;
    prevY = yRot->position(turns) * wheelCircumference;
    prevThetaRad = imuSensor->rotation() * (M_PI / 180.0);

    odomRunning = true;
    odomThread = thread(odomTask);

    wait(10, msec);
}

void stopOdom()
{
    if (odomRunning)
    {
        odomRunning = false;
        odomThread.join();
    }
}

Pose getPose()
{
    Pose poseCopy;
    // Copy the struct fields (atomic on simple doubles generally safe)
    poseCopy.x = currentPose.x;
    poseCopy.y = currentPose.y;
    poseCopy.theta = currentPose.theta;
    poseCopy.ySensor = yRot->position(turns) * wheelCircumference;
    poseCopy.xSensor = xRot->position(turns) * wheelCircumference;
    return poseCopy;
}

/*
#include "vex.h"
#include "odom.h"
#include <cmath>
#include <mutex>

using namespace vex;

// ----------------- Pose Struct -----------------
struct Pose {
    double x = 0.0;      // inches
    double y = 0.0;      // inches
    double theta = 0.0;  // degrees
    double vx = 0.0;     // inches/sec
    double vy = 0.0;     // inches/sec
    double omega = 0.0;  // deg/sec
};

// ----------------- Internal State -----------------
Pose currentPose;
rotation *xRot;
rotation *yRot;
inertial *imuSensor;

const double xOffset = -8.0;  // inches
const double yOffset = -0.3;  // inches

const double odomWheelDiameter = 2.0;
const double wheelCircumference = odomWheelDiameter * M_PI;

double prevX = 0.0;
double prevY = 0.0;
double prevThetaRad = 0.0;
double prevTime = 0.0;

bool odomRunning = false;
thread odomThread;
std::mutex poseMutex;

// Low-pass filter constants (optional smoothing)
const double alpha = 0.5;  // 0 = no smoothing, 1 = max smoothing

// ----------------- Helper Functions -----------------
double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / M_PI; }

double wrapAngleRad(double angle) {
    while (angle > M_PI) angle -= 2*M_PI;
    while (angle < -M_PI) angle += 2*M_PI;
    return angle;
}

// ----------------- Odometry Task -----------------
int odomTask() {
    double lastX = prevX;
    double lastY = prevY;
    double lastTheta = prevThetaRad;
    double lastTime = Brain.Timer.time(msec)/1000.0; // seconds

    while (odomRunning) {
        double currTime = Brain.Timer.time(msec)/1000.0;
        double dt = currTime - lastTime;
        if (dt < 1e-3) dt = 1e-3;  // prevent divide by zero

        // --- Read sensors ---
        double currXTurns = xRot->position(turns);
        double currYTurns = yRot->position(turns);
        double thetaDeg = imuSensor->rotation();
        double thetaRad = deg2rad(thetaDeg);

        // --- Encoder to inches ---
        double currXInches = currXTurns * wheelCircumference;
        double currYInches = currYTurns * wheelCircumference;

        // --- Delta calculations ---
        double deltaX = currXInches - lastX;
        double deltaY = currYInches - lastY;
        double deltaTheta = wrapAngleRad(thetaRad - lastTheta);

        // --- Arc-based displacement ---
        double localX = 0.0, localY = 0.0;
        if (fabs(deltaTheta) < 1e-6) {
            localX = deltaX;
            localY = deltaY;
        } else {
            double rFactor = 2.0 * sin(deltaTheta/2.0) / deltaTheta;
            localX = rFactor * (deltaX + xOffset * deltaTheta);
            localY = rFactor * (deltaY + yOffset * deltaTheta);
        }

        // --- Rotate to global frame ---
        double avgTheta = lastTheta + deltaTheta/2.0;
        double cosT = cos(avgTheta);
        double sinT = sin(avgTheta);
        double deltaXGlobal = localX * cosT - localY * sinT;
        double deltaYGlobal = localX * sinT + localY * cosT;

        // --- Velocity calculation (inches/sec, deg/sec) ---
        double vx = deltaXGlobal / dt;
        double vy = deltaYGlobal / dt;
        double omega = rad2deg(deltaTheta) / dt;

        // --- Low-pass filter for smoothing ---
        {
            std::lock_guard<std::mutex> lock(poseMutex);
            currentPose.x += deltaXGlobal;
            currentPose.y += deltaYGlobal;
            currentPose.theta = fmod(thetaDeg + 360.0, 360.0);
            currentPose.vx = alpha * vx + (1 - alpha) * currentPose.vx;
            currentPose.vy = alpha * vy + (1 - alpha) * currentPose.vy;
            currentPose.omega = alpha * omega + (1 - alpha) * currentPose.omega;
        }

        // --- Store last values ---
        lastX = currXInches;
        lastY = currYInches;
        lastTheta = thetaRad;
        lastTime = currTime;

        wait(10, msec);  // 100Hz update
    }
    return 0;
}

// ----------------- Start / Stop -----------------
void startOdom(rotation &xSensor, rotation &ySensor, inertial &imu) {
    xRot = &xSensor;
    yRot = &ySensor;
    imuSensor = &imu;

    xRot->resetPosition();
    yRot->resetPosition();
    imuSensor->resetRotation();

    prevX = xRot->position(turns) * wheelCircumference;
    prevY = yRot->position(turns) * wheelCircumference;
    prevThetaRad = deg2rad(imuSensor->rotation());

    odomRunning = true;
    odomThread = thread(odomTask);

    wait(10, msec);
}

void stopOdom() {
    if (odomRunning) {
        odomRunning = false;
        odomThread.join();
    }
}

// ----------------- Get Pose -----------------
Pose getPose() {
    std::lock_guard<std::mutex> lock(poseMutex);
    return currentPose;
}
*/