/*#include "bv2odom.h"
#include <cmath>
#include "vex.h"
#include "utils.h"

using namespace vex;

// -------------------- Internal state --------------------
static OdomPose currentPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static rotation *rotX = nullptr; // lateral tracker
static rotation *rotY = nullptr; // longitudinal tracker
static inertial *imuSensor = nullptr;

static double trackerXOffset = -8.0;
static double trackerYOffset = -0.3;
static double odomWheelDiameter = 2.0;
static double wheelCircumference = odomWheelDiameter * M_PI;

static bool odomRunning = false;
static vex::thread odomThread;
static int updateHz = 100;

// smoothing & fusion
static double alphaV = 0.5;
static double alphaOmega = 0.5;
static double imuWeight = 0.95;
static double maxImuJumpDeg = 30.0;

// -------------------- Odom Loop --------------------
static void odomLoop()
{
    double lastX = rotX->position(turns) * wheelCircumference;
    double lastY = rotY->position(turns) * wheelCircumference;
    double lastThetaDeg = imuSensor->rotation();
    double lastTime = Brain.Timer.time(msec) / 1000.0;

    currentPose.x = 0.0;
    currentPose.y = 0.0;
    currentPose.theta = wrapDeg360(lastThetaDeg);
    currentPose.vx = currentPose.vy = currentPose.omega = 0.0;

    const double dtMin = 1e-4;

    while (odomRunning)
    {
        double now = Brain.Timer.time(msec) / 1000.0;
        double dt = now - lastTime;
        if (dt < dtMin)
            dt = dtMin;

        double xTurns = rotX->position(turns);
        double yTurns = rotY->position(turns);
        double currXInches = xTurns * wheelCircumference;
        double currYInches = yTurns * wheelCircumference;
        double imuDeg = imuSensor->rotation();

        double dX = currXInches - lastX;
        double dY = currYInches - lastY;

        double rawDTheta = smallestAngleDiffDeg(imuDeg, lastThetaDeg);
        double usedDThetaDeg = rawDTheta;
        if (fabs(rawDTheta) > maxImuJumpDeg)
            usedDThetaDeg = 0.0;
        double dThetaRad = deg2rad(usedDThetaDeg);

        // Arc correction
        double localX = 0.0, localY = 0.0;
        if (fabs(dThetaRad) < 1e-9)
        {
            localX = dX;
            localY = dY;
        }
        else
        {
            double rFactor = 2.0 * sin(dThetaRad / 2.0) / dThetaRad;
            localX = rFactor * (dX + trackerXOffset * dThetaRad);
            localY = rFactor * (dY + trackerYOffset * dThetaRad);
        }

        double lastThetaRad = deg2rad(lastThetaDeg);
        double avgTheta = lastThetaRad + dThetaRad * 0.5;
        double cosA = cos(avgTheta), sinA = sin(avgTheta);
        double dxGlobal = localX * cosA - localY * sinA;
        double dyGlobal = localX * sinA + localY * cosA;

        double rawVx = dxGlobal / dt;
        double rawVy = dyGlobal / dt;
        double rawOmega = usedDThetaDeg / dt;

        // fused heading
        double prevFusedTheta = currentPose.theta;
        double diff = smallestAngleDiffDeg(imuDeg, prevFusedTheta);
        double fusedThetaDeg = prevFusedTheta + imuWeight * diff;
        fusedThetaDeg = wrapDeg360(fusedThetaDeg);

        // update pose
        currentPose.x += dxGlobal;
        currentPose.y += dyGlobal;
        currentPose.theta = fusedThetaDeg;
        currentPose.vx = alphaV * rawVx + (1 - alphaV) * currentPose.vx;
        currentPose.vy = alphaV * rawVy + (1 - alphaV) * currentPose.vy;
        currentPose.omega = alphaOmega * rawOmega + (1 - alphaOmega) * currentPose.omega;

        lastX = currXInches;
        lastY = currYInches;
        lastThetaDeg = imuDeg;
        lastTime = now;

        wait((int)(1000.0 / std::max(1, updateHz)), msec);
    }
}

// -------------------- Public API --------------------
void startOdom(rotation &xSensor, rotation &ySensor, inertial &imu, bool waitForCalib, int updateHzArg)
{
    rotX = &xSensor;
    rotY = &ySensor;
    imuSensor = &imu;
    wheelCircumference = odomWheelDiameter * M_PI;
    rotX->resetPosition();
    rotY->resetPosition();
    if (waitForCalib)
    {
        imuSensor->calibrate();
        while (imuSensor->isCalibrating())
            wait(10, msec);
    }
    else
        imuSensor->resetRotation();

    updateHz = std::max(10, updateHzArg);

    currentPose.x = currentPose.y = currentPose.vx = currentPose.vy = currentPose.omega = 0.0;
    currentPose.theta = wrapDeg360(imuSensor->rotation());

    odomRunning = true;
    odomThread = vex::thread(odomLoop);
}

void stopOdom()
{
    odomRunning = false;
}

OdomPose getPose() { return currentPose; }

void resetPose(double x, double y, double thetaDeg)
{
    currentPose.x = x;
    currentPose.y = y;
    currentPose.theta = wrapDeg360(thetaDeg);
    currentPose.vx = currentPose.vy = currentPose.omega = 0.0;
    if (rotX)
        rotX->resetPosition();
    if (rotY)
        rotY->resetPosition();
    if (imuSensor)
        imuSensor->resetRotation();
}

void setOdomOffsets(double xOffsetInches, double yOffsetInches, double wheelDiam)
{
    trackerXOffset = xOffsetInches;
    trackerYOffset = yOffsetInches;
    odomWheelDiameter = wheelDiam;
    wheelCircumference = odomWheelDiameter * M_PI;
}

void setVelocitySmoothing(double aV, double aOmega)
{
    alphaV = clamp(aV, 0.0, 1.0);
    alphaOmega = clamp(aOmega, 0.0, 1.0);
}
void setIMUWeight(double imuW) { imuWeight = clamp(imuW, 0.0, 1.0); }
void setIMUJumpThreshold(double maxDeg) { maxImuJumpDeg = std::max(1.0, maxDeg); }
void setUpdateHz(int hz) { updateHz = std::max(10, hz); }
*/