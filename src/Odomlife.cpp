// odom.cpp
#include "vex.h"
#include "odometry.h"

using namespace vex;

// Internal global variables for legacy functions
namespace {
    OdomSystem* globalOdom = nullptr;
    Pose globalPose;
    
    // Wrapper function for thread (must be standalone function)
    int odomTaskWrapper(void* odomSystem) {
        OdomSystem* odom = static_cast<OdomSystem*>(odomSystem);
        return odom->odomTask();
    }
}

// OdomSystem class implementation
OdomSystem::OdomSystem(double xOffset, double yOffset, double wheelDiameter) 
    : xRot(nullptr), yRot(nullptr), imuSensor(nullptr), 
      odomRunning(false), xOffset(xOffset), yOffset(yOffset),
      wheelDiameter(wheelDiameter), prevXEncoder(0), prevYEncoder(0), prevThetaRad(0) {
    wheelCircumference = wheelDiameter * M_PI;
}

OdomSystem::~OdomSystem() {
    stop();
}

int OdomSystem::odomTask() {
    while (odomRunning) {
        if (!xRot || !yRot || !imuSensor) {
            wait(10, msec);
            continue;
        }

        // 1. Get current sensor readings
        double currXTurns = xRot->position(turns);
        double currYTurns = yRot->position(turns);
        double thetaDeg = imuSensor->rotation();
        double thetaRad = thetaDeg * (M_PI / 180.0);

        // 2. Convert encoder readings to inches
        double currXInches = currXTurns * wheelCircumference;
        double currYInches = currYTurns * wheelCircumference;

        // 3. Calculate raw deltas in ENCODER coordinates
        double deltaXEncoder = currXInches - prevXEncoder;
        double deltaYEncoder = currYInches - prevYEncoder;

        // 4. Calculate heading delta and wrap to [-π, π]
        double deltaTheta = thetaRad - prevThetaRad;
        while (deltaTheta > M_PI) deltaTheta -= 2.0 * M_PI;
        while (deltaTheta < -M_PI) deltaTheta += 2.0 * M_PI;

        // 5. Compute local displacement with arc correction
        double localDeltaX = 0.0, localDeltaY = 0.0;

        if (fabs(deltaTheta) < 1e-6) {
            // Straight movement
            localDeltaX = deltaXEncoder;
            localDeltaY = deltaYEncoder;
        } else {
            // Arc movement
            double rFactor = 2.0 * sin(deltaTheta / 2.0) / deltaTheta;
            localDeltaX = rFactor * (deltaXEncoder - yOffset * deltaTheta);
            localDeltaY = rFactor * (deltaYEncoder + xOffset * deltaTheta);
        }

        // 6. Rotate local displacement to global coordinates
        double avgTheta = prevThetaRad + deltaTheta / 2.0;
        double cosT = cos(avgTheta);
        double sinT = sin(avgTheta);

        double deltaXGlobal = localDeltaY * sinT + localDeltaX * cosT;
        double deltaYGlobal = localDeltaY * cosT - localDeltaX * sinT;

        // 7. Thread-safe pose update
        {
            poseMutex.lock();
            currentPose.x += deltaXGlobal;
            currentPose.y += deltaYGlobal;
            currentPose.theta = thetaDeg;
            poseMutex.unlock();
        }

        // 8. Store current readings for next iteration
        prevXEncoder = currXInches;
        prevYEncoder = currYInches;
        prevThetaRad = thetaRad;

        wait(10, msec);
    }
    return 0;
}

void OdomSystem::start(rotation& xSensor, rotation& ySensor, inertial& imu) {
    stop(); // Stop if already running
    
    xRot = &xSensor;
    yRot = &ySensor;
    imuSensor = &imu;

    // Reset sensors
    xRot->resetPosition();
    yRot->resetPosition();
    imu.calibrate();
    while (imu.isCalibrating()) {
        wait(10, msec);
    }
    imu.resetRotation();

    // Initialize previous readings
    prevXEncoder = xRot->position(turns) * wheelCircumference;
    prevYEncoder = yRot->position(turns) * wheelCircumference;
    prevThetaRad = imu.rotation() * (M_PI / 180.0);

    // Reset pose
    setPose(0.0, 0.0, 0.0);

    // Start thread - CORRECTED: Use wrapper function with 'this' pointer
    odomRunning = true;
    odomThread = thread(odomTaskWrapper, this);
}

void OdomSystem::startAt(rotation& xSensor, rotation& ySensor, inertial& imu,
                        double startX, double startY, double startTheta) {
    stop();
    
    xRot = &xSensor;
    yRot = &ySensor;
    imuSensor = &imu;

    // Reset sensors
    xRot->resetPosition();
    yRot->resetPosition();
    imu.calibrate();
    while (imu.isCalibrating()) {
        wait(10, msec);
    }
    imu.resetRotation();

    // Initialize previous readings
    prevXEncoder = xRot->position(turns) * wheelCircumference;
    prevYEncoder = yRot->position(turns) * wheelCircumference;
    prevThetaRad = startTheta * (M_PI / 180.0);

    // Set initial pose
    setPose(startX, startY, startTheta);

    // Start thread - CORRECTED
    odomRunning = true;
    odomThread = thread(odomTaskWrapper, this);
}

void OdomSystem::stop() {
    if (odomRunning) {
        odomRunning = false;
        if (odomThread.joinable()) {
            odomThread.join();
        }
    }
}

Pose OdomSystem::getPose() {
    Pose poseCopy;
    poseMutex.lock();
    poseCopy = currentPose;
    poseMutex.unlock();
    
    // Add raw sensor readings
    if (xRot && yRot) {
        poseCopy.xSensor = xRot->position(turns) * wheelCircumference;
        poseCopy.ySensor = yRot->position(turns) * wheelCircumference;
    }
    
    return poseCopy;
}

void OdomSystem::setPose(const Pose& newPose) {
    poseMutex.lock();
    currentPose = newPose;
    poseMutex.unlock();
}

void OdomSystem::setPose(double x, double y, double theta) {
    poseMutex.lock();
    currentPose.x = x;
    currentPose.y = y;
    currentPose.theta = theta;
    poseMutex.unlock();
}

void OdomSystem::reset() {
    setPose(0.0, 0.0, 0.0);
    if (xRot) xRot->resetPosition();
    if (yRot) yRot->resetPosition();
    if (imuSensor) imuSensor->resetRotation();
}

void OdomSystem::resetTo(double x, double y, double theta) {
    setPose(x, y, theta);
    if (xRot) xRot->resetPosition();
    if (yRot) yRot->resetPosition();
    if (imuSensor) imuSensor->resetRotation();
}

void OdomSystem::getRawEncoders(double& xInches, double& yInches) {
    if (xRot && yRot) {
        xInches = xRot->position(turns) * wheelCircumference;
        yInches = yRot->position(turns) * wheelCircumference;
    } else {
        xInches = 0;
        yInches = 0;
    }
}

double OdomSystem::getIMUHeading() {
    return imuSensor ? imuSensor->rotation() : 0.0;
}

void OdomSystem::printPose() {
    Pose p = getPose();
    Brain.Screen.print("Pose: X=%.2f, Y=%.2f, Theta=%.2f", p.x, p.y, p.theta);
    printf("Pose: X=%.2f, Y=%.2f, Theta=%.2f\n", p.x, p.y, p.theta);
}

void OdomSystem::setWheelDiameter(double newDiameter) {
    wheelDiameter = newDiameter;
    wheelCircumference = wheelDiameter * M_PI;
}

void OdomSystem::setOffsets(double newXOffset, double newYOffset) {
    xOffset = newXOffset;
    yOffset = newYOffset;
}

// Legacy C-style functions implementation
void startOdom(rotation& xSensor, rotation& ySensor, inertial& imu) {
    if (!globalOdom) {
        globalOdom = new OdomSystem();
    }
    globalOdom->start(xSensor, ySensor, imu);
}

void startOdomAt(rotation& xSensor, rotation& ySensor, inertial& imu,
                double startX, double startY, double startTheta) {
    if (!globalOdom) {
        globalOdom = new OdomSystem();
    }
    globalOdom->startAt(xSensor, ySensor, imu, startX, startY, startTheta);
}

void stopOdom() {
    if (globalOdom) {
        globalOdom->stop();
        delete globalOdom;
        globalOdom = nullptr;
    }
}

Pose getPose() {
    if (globalOdom) {
        return globalOdom->getPose();
    }
    return Pose();
}

void printPose() {
    if (globalOdom) {
        globalOdom->printPose();
    } else {
        printf("Odom not initialized!\n");
    }
}