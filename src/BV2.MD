#include "vex.h"
#include <cmath>
#include <algorithm>
#include <vector>

// ---------- CONFIG ----------
#define WHEEL_DIAMETER 2.75
#define WHEEL_TRACK 10.5  // distance between left/right tracking wheels in inches
#define TICKS_PER_REV 360.0
#define PI 3.14159265358979323846

// ---------- CLAMP ----------
template <typename T>
T clamp(T value, T minVal, T maxVal) {
    if (value < minVal) return minVal;
    if (value > maxVal) return maxVal;
    return value;
}

// ---------- BASIC PID ----------
class PID {
public:
    double kP, kI, kD;
    double integral, prevError;
    PID(double p=0, double i=0, double d=0) {
        kP = p; kI = i; kD = d;
        integral = 0; prevError = 0;
    }
    void reset() { integral = 0; prevError = 0; }
    double compute(double target, double current, bool turn=false) {
        double error = target - current;
        if (turn) {
            if (error > 180) error -= 360;
            if (error < -180) error += 360;
        }
        if (fabs(error) < 10) { integral += error; }
        double derivative = error - prevError;
        prevError = error;
        return (error * kP) + (integral * kI) + (derivative * kD);
    }
};

// ---------- GLOBAL PIDs ----------
PID distPID(0.15, 0.0, 0.9);        // position PID
PID headingPID(0.04, 0.0, 0.33);    // boomerang angle correction
PID velPID(0.3, 0.0, 0.05);         // velocity PID for BV2

// ---------- POSE ----------
struct Pose {
    double x, y, theta; // inches, inches, degrees
};

Pose robotPose = {0, 0, 0};

// ---------- ODOMETRY UPDATE ----------
void updateOdometry(double leftDelta, double rightDelta) {
    double leftInches = (leftDelta / TICKS_PER_REV) * (PI * WHEEL_DIAMETER);
    double rightInches = (rightDelta / TICKS_PER_REV) * (PI * WHEEL_DIAMETER);
    double centerArc = (leftInches + rightInches) / 2.0;
    double headingChange = (rightInches - leftInches) / WHEEL_TRACK;

    robotPose.theta += headingChange * (180.0 / PI);
    if (robotPose.theta > 180) robotPose.theta -= 360;
    if (robotPose.theta < -180) robotPose.theta += 360;

    robotPose.x += centerArc * cos(robotPose.theta * PI / 180.0);
    robotPose.y += centerArc * sin(robotPose.theta * PI / 180.0);
}

// ---------- GET POSE ----------
Pose getPose() {
    return robotPose;
}

// ---------- DRIVE CONTROL ----------
void setDrive(double left, double right) {
    left = clamp(left, -11.0, 11.0);
    right = clamp(right, -11.0, 11.0);
    L1.spin(fwd, left, volt);
    L2.spin(fwd, left, volt);
    L3.spin(fwd, left, volt);
    R6.spin(fwd, right, volt);
    R7.spin(fwd, right, volt);
    R8.spin(fwd, right, volt);
}

// ---------- 2D MOTION PROFILE ----------
double trapezoidalProfile(double distance, double maxVel, double accel, double traveled) {
    double accelDist = (maxVel * maxVel) / (2 * accel);
    double decelDist = accelDist;
    double cruiseDist = distance - accelDist - decelDist;

    if (traveled < accelDist) { // accel phase
        return sqrt(2 * accel * traveled);
    } else if (traveled < accelDist + cruiseDist) { // cruise phase
        return maxVel;
    } else { // decel phase
        double remaining = distance - traveled;
        return sqrt(2 * accel * remaining);
    }
}

// ---------- BV2 CONTROLLER ----------
void BV2Follow(double targetX, double targetY, double maxVel=40, double accel=20) {
    distPID.reset();
    velPID.reset();
    headingPID.reset();

    Pose start = getPose();
    double totalDist = sqrt(pow(targetX - start.x, 2) + pow(targetY - start.y, 2));

    while (true) {
        Pose pose = getPose();
        double dx = targetX - pose.x;
        double dy = targetY - pose.y;
        double distToTarget = sqrt(dx*dx + dy*dy);

        if (distToTarget < 0.5) break;

        double targetHeading = atan2(dy, dx) * (180.0 / PI);
        double headingError = headingPID.compute(targetHeading, pose.theta, true);

        double targetVel = trapezoidalProfile(totalDist, maxVel, accel, totalDist - distToTarget);
        double currentVel = 0; // Replace with real velocity sensor if available
        double velCorrection = velPID.compute(targetVel, currentVel);

        double basePower = distPID.compute(totalDist, totalDist - distToTarget);
        basePower += velCorrection;
        basePower = clamp(basePower, -1.0, 1.0) * 11.0;

        double leftPower = basePower + headingError;
        double rightPower = basePower - headingError;

        setDrive(leftPower, rightPower);
        wait(10, msec);
    }
    setDrive(0,0);
}

// ---------- PD CONTROLLER ----------
void PDDrive(double distInches) {
    PID pd(0.2, 0, 1.0);
    pd.reset();
    Pose start = getPose();
    double traveled = 0;
    while (true) {
        Pose pose = getPose();
        double dx = pose.x - start.x;
        double dy = pose.y - start.y;
        traveled = sqrt(dx*dx + dy*dy);
        double error = distInches - traveled;
        if (fabs(error) < 0.5) break;
        double output = pd.compute(distInches, traveled);
        setDrive(output*11.0, output*11.0);
        wait(10, msec);
    }
    setDrive(0,0);
}
