// motion_system.cpp
// Unified motion: fast PD short controllers + BV2 (boomerang velocity + 2D motion profiling)
// Improved polar-corrected odometry (two trackers + IMU)
// Usage: include this in your project and call startOdom(...)

#include "vex.h"
#include <cmath>
#include <vector>
#include <queue>
#include <functional>
#include <algorithm>
#include <mutex>

using namespace vex;

// ------------------------- Configuration (tune these) -------------------------
static constexpr double ODOM_WHEEL_DIAMETER_IN = 2.0;   // inches
static constexpr double ODOM_WHEEL_CIRC = ODOM_WHEEL_DIAMETER_IN * M_PI;
static constexpr double WHEEL_TRACK_IN = 14.5;         // robot track width (left-right distance)
static constexpr double DEFAULT_MAXVEL = 45.0;         // inches/sec for profiles (tunable)
static constexpr double DEFAULT_ACCEL = 120.0;         // inches/sec^2 (tunable)
static constexpr double ODOM_UPDATE_MS = 10.0;         // odom update period (ms)
static constexpr double CONTROL_DT = 10.0;             // control loop dt (ms)

// Offsets: xOffset is lateral (inches) from tracking center to X (side) wheel.
//           yOffset is forward (inches) from tracking center to Y (forward) wheel.
static double xOffset =  2.0;   // set to your lateral tracker offset (positive to robot right)
static double yOffset =  0.0;   // set to your forward tracker offset (positive forward)

// ---------- Hardware (assumed names in your project) ----------
extern motor L1, L2, L3, R6, R7, R8;
extern rotation Xaxis, Yaxis;     // lateral (X) and forward (Y) trackers
extern inertial inertial19;

// ---------- Small utilities ----------
template <typename T>
static T clamp_t(T v, T lo, T hi) { return (v < lo) ? lo : (v > hi) ? hi : v; }

static double deg2rad(double d) { return d * M_PI / 180.0; }
static double rad2deg(double r) { return r * 180.0 / M_PI; }

// ------------------------- Pose / Odometry -------------------------
struct Pose {
  double x = 0.0;    // inches
  double y = 0.0;    // inches
  double theta = 0.0; // degrees (0..360)
};

static Pose currentPose;
static std::mutex odomMutex;

// Internal previous states (inches and radians)
static double prevXInches = 0.0;
static double prevYInches = 0.0;
static double prevThetaRad = 0.0;
static task odomTaskHandle;
static bool odomRunning = false;

// Helper to get pose thread-safely
Pose getPose() {
  std::lock_guard<std::mutex> g(odomMutex);
  return currentPose;
}

// Start odometry task; call once in main after devices initialized
void startOdom(rotation &xSensor, rotation &ySensor, inertial &imu) {
  Xaxis = xSensor; // not actually copying (but keep names consistent); ignore if extern already defined
  Yaxis = ySensor;
  // sensor pointers are globals declared above as extern; ensure they match your project config

  // reset previous readings
  Xaxis.resetPosition();
  Yaxis.resetPosition();
  imu.resetRotation();

  prevXInches = (Xaxis.position(turns) * ODOM_WHEEL_CIRC);
  prevYInches = (Yaxis.position(turns) * ODOM_WHEEL_CIRC);
  prevThetaRad = deg2rad(imu.rotation());

  odomRunning = true;
  odomTaskHandle = task([&](void*) {
    while (odomRunning) {
      // 1) read encoders (turns) -> inches
      double currXInches = Xaxis.position(turns) * ODOM_WHEEL_CIRC;
      double currYInches = Yaxis.position(turns) * ODOM_WHEEL_CIRC;

      // 2) deltas (robot-local)
      double deltaX = currXInches - prevXInches; // lateral movement measured by X tracker (right positive)
      double deltaY = currYInches - prevYInches; // forward movement measured by Y tracker (forward positive)

      // 3) heading
      double thetaDeg = inertial19.rotation();
      double thetaRad = deg2rad(thetaDeg);

      // 4) delta theta (wrap to [-pi, pi])
      double deltaTheta = thetaRad - prevThetaRad;
      if (deltaTheta > M_PI) deltaTheta -= 2.0 * M_PI;
      if (deltaTheta < -M_PI) deltaTheta += 2.0 * M_PI;

      // 5) compute local displacement using polar-corrected method (Purdue)
      double localX = 0.0, localY = 0.0;
      if (fabs(deltaTheta) < 1e-6) {
        // nearly straight -> no arc math needed
        localX = deltaX;
        localY = deltaY;
      } else {
        // (Equation 6 style): local = 2*sin(dtheta/2) * [ deltaX/dtheta + xOffset ; deltaY/dtheta + yOffset ]
        double rFactor = 2.0 * sin(deltaTheta / 2.0) / deltaTheta;
        localX = rFactor * (deltaX + xOffset * deltaTheta);
        localY = rFactor * (deltaY + yOffset * deltaTheta);
      }

      // 6) rotate local by average heading to global
      double avgTheta = prevThetaRad + deltaTheta / 2.0;
      double cosT = cos(avgTheta), sinT = sin(avgTheta);
      double globalDX = localX * cosT - localY * sinT;
      double globalDY = localX * sinT + localY * cosT;

      // 7) commit to pose
      {
        std::lock_guard<std::mutex> g(odomMutex);
        currentPose.x += globalDX;
        currentPose.y += globalDY;
        double clean = fmod(thetaDeg, 360.0);
        if (clean < 0.0) clean += 360.0;
        currentPose.theta = clean;
      }

      // 8) store for next
      prevXInches = currXInches;
      prevYInches = currYInches;
      prevThetaRad = thetaRad;

      wait(ODOM_UPDATE_MS, msec);
    }
    return 0;
  }, nullptr);
}

// Stop odom thread (if needed)
void stopOdom() {
  odomRunning = false;
  // task::stop(odomTaskHandle); // optional
}

// ------------------------- Basic PID class -------------------------
class PID {
public:
  double kP = 0.0, kI = 0.0, kD = 0.0;
  double integral = 0.0;
  double prevError = 0.0;
  double outputCap = 1.0; // typically 1.0 for percent or 11 for volts

  PID() {}
  PID(double p, double i, double d, double cap = 1.0) : kP(p), kI(i), kD(d), outputCap(cap) {}

  void reset() { integral = 0.0; prevError = 0.0; }

  // compute: target - current (units follow usage)
  double compute(double target, double current) {
    double error = target - current;
    integral += error;
    // small integral cap by default
    integral = clamp_t(integral, -1000.0, 1000.0);
    double deriv = error - prevError;
    prevError = error;
    double out = error * kP + integral * kI + deriv * kD;
    out = clamp_t(out, -outputCap, outputCap);
    return out;
  }
};

// ------------------------- Simple 1D Profile (used to compute target linear speed) -------------------------
class Profile {
public:
  double maxVel;
  double accel;
  Profile(double m = DEFAULT_MAXVEL, double a = DEFAULT_ACCEL) : maxVel(m), accel(a) {}

  // distRemaining, distTraveled in inches -> returns inches/sec (signed by direction)
  double getTargetVelocity(double distRemaining, double distTraveled, double direction = 1.0) {
    if (distTraveled < 0.5) distTraveled = 0.5; // avoid small instability
    if (distRemaining < 0.001) distRemaining = 0.0;

    double accelLimit = sqrt(2.0 * accel * distTraveled);
    double decelLimit = sqrt(2.0 * accel * distRemaining);
    double target = std::min({accelLimit, decelLimit, maxVel});
    return target * direction;
  }

  void setMaxVel(double m) { maxVel = m; }
  void setAccel(double a) { accel = a; }
};

// ------------------------- Low-level motor helper (volts) -------------------------
static void setDriveVolts(double leftV, double rightV) {
  leftV = clamp_t(leftV, -11.0, 11.0);
  rightV = clamp_t(rightV, -11.0, 11.0);
  L1.spin(fwd, leftV, volt);
  L2.spin(fwd, leftV, volt);
  L3.spin(fwd, leftV, volt);
  R6.spin(fwd, rightV, volt);
  R7.spin(fwd, rightV, volt);
  R8.spin(fwd, rightV, volt);
}

// ------------------------- Fast PD controllers (short, snappy) -------------------------
PID fastDistPID(0.20, 0.0, 0.9, 1.0);    // output in [-1,1]
PID fastTurnPID(0.043, 0.0001, 0.39, 1.0);

void fastDrive(double distInches) {
  fastDistPID.reset();

  // use wheel encoders to measure traveled (avg revs)
  double startLeft = (L1.position(turns) + L2.position(turns) + L3.position(turns)) / 3.0;
  double startRight = (R6.position(turns) + R7.position(turns) + R8.position(turns)) / 3.0;

  // reset rotations optional
  L1.resetPosition(); L2.resetPosition(); L3.resetPosition();
  R6.resetPosition(); R7.resetPosition(); R8.resetPosition();

  double elapsed = 0;
  const double timeout = 3000;
  double lastError = distInches;

  while (true) {
    // avg wheel distance traveled:
    double avgTurns = ( (L1.position(turns) + L2.position(turns) + L3.position(turns)) / 3.0
                      + (R6.position(turns) + R7.position(turns) + R8.position(turns)) / 3.0 ) / 2.0;
    double traveled = avgTurns * (ODOM_WHEEL_CIRC); // approximate assuming drivetrain wheels same diameter
    double error = distInches - traveled;

    double out = fastDistPID.compute(distInches, traveled);
    // clamp to [-1,1], then convert to volts
    out = clamp_t(out, -1.0, 1.0);
    double v = out * 11.0;
    setDriveVolts(v, v);

    if ((fabs(error) < 0.5 && fabs(error - lastError) < 0.1) || elapsed >= timeout) break;
    lastError = error;
    elapsed += CONTROL_DT;
    wait(CONTROL_DT, msec);
  }
  setDriveVolts(0,0);
}

// Fast turn using IMU + fastTurnPID
void fastTurn(double targetHeadingDeg) {
  fastTurnPID.reset();
  double elapsed = 0;
  const double timeout = 2000;

  while (true) {
    Pose p = getPose();
    double heading = p.theta; // degrees [0..360)
    double error = targetHeadingDeg - heading;
    // wrap error to [-180,180]
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;

    double out = fastTurnPID.compute(0.0, -error); // our PID expects target-current; using -error to get direction
    out = clamp_t(out, -1.0, 1.0);
    double v = out * 11.0;
    setDriveVolts(v, -v);

    if (fabs(error) < 1.0 || elapsed >= timeout) break;
    elapsed += CONTROL_DT;
    wait(CONTROL_DT, msec);
  }
  setDriveVolts(0,0);
}

// ------------------------- BV2: Boomerang Velocity v2 -------------------------
// High-level: generate a curved path (Hermite or polyline samples) and follow it using:
//  - Profile => desired linear speed along path (inches/sec)
//  - Velocity PID => compares actual robot forward speed to target linear speed, outputs base voltage
//  - Angular PID => compares heading to tangent, outputs small correction voltage
//  - combined => left = base - angCorr ; right = base + angCorr
//
// This design separates *path*-level planning (2D target velocities) and *motor* control (voltage via velocity PID).
class BV2 {
public:
  BV2()
    : velPID(1.0, 0.0, 0.05, 11.0), angPID(0.8, 0.0, 0.05, 5.0),
      profile(DEFAULT_MAXVEL, DEFAULT_ACCEL) {}

  // tuning
  void setVelocityGains(double p,double i,double d){ velPID.kP=p; velPID.kI=i; velPID.kD=d; }
  void setAngularGains(double p,double i,double d){ angPID.kP=p; angPID.kI=i; angPID.kD=d; }
  void setProfile(double maxV, double accel){ profile.setMaxVel(maxV); profile.setAccel(accel); }

  // Simple hermite generator between two points p0->p1. tangents are derived from orientations.
  std::vector<Pose> generateHermite(Pose p0, Pose p1, int samples = 60) {
    std::vector<Pose> out;
    // convert thetas to radians for tangents
    double t0 = deg2rad(p0.theta), t1 = deg2rad(p1.theta);
    // Tangent magnitudes; scale by distance between points
    double d = hypot(p1.x - p0.x, p1.y - p0.y);
    double m0 = d * 0.5;
    double m1 = d * 0.5;
    for (int i=0;i<=samples;i++){
      double u = (double)i / (double)samples;
      double h00 = 2*u*u*u - 3*u*u + 1;
      double h10 = u*u*u - 2*u*u + u;
      double h01 = -2*u*u*u + 3*u*u;
      double h11 = u*u*u - u*u;
      double x = h00*p0.x + h10*(m0*cos(t0)) + h01*p1.x + h11*(m1*cos(t1));
      double y = h00*p0.y + h10*(m0*sin(t0)) + h01*p1.y + h11*(m1*sin(t1));
      // approximate derivative for tangent angle
      double dx = 6*(p1.x-p0.x)*u*(1-u);
      double dy = 6*(p1.y-p0.y)*u*(1-u);
      double theta = rad2deg(atan2(dy,dx));
      out.push_back({x,y,theta});
    }
    return out;
  }

  // follow a path (list of points). This is blocking.
  void followPath(const std::vector<Pose>& path, bool forward=true, double lead = 6.0, double timeoutMs = 60000) {
    if (path.empty()) return;
    velPID.reset();
    angPID.reset();

    // compute path length remaining function (approx)
    auto pathRemaining = [&](int idx, double tfrac) {
      // idx: index of segment start, tfrac in [0,1] fraction along segment
      double rem = 0.0;
      // remaining on current segment:
      if (idx < (int)path.size()-1) {
        double sx = path[idx].x, sy = path[idx].y;
        double ex = path[idx+1].x, ey = path[idx+1].y;
        double segLen = hypot(ex - sx, ey - sy);
        rem += segLen * (1.0 - tfrac);
        for (int j = idx+1; j < (int)path.size()-1; ++j) {
          double ax = path[j].x, ay = path[j].y;
          double bx = path[j+1].x, by = path[j+1].y;
          rem += hypot(bx-ax, by-ay);
        }
      }
      return rem;
    };

    // helper to compute robot forward velocity (inches/sec) using encoders
    auto getRobotForwardVel = [&]() -> double {
      // approximate using both sides average rpm of drivetrain (use motor rpm)
      double leftRpm = (L1.velocity(rpm) + L2.velocity(rpm) + L3.velocity(rpm))/3.0;
      double rightRpm = (R6.velocity(rpm) + R7.velocity(rpm) + R8.velocity(rpm))/3.0;
      double avgRpm = (leftRpm + rightRpm) / 2.0;
      // convert rpm to inches/sec: rpm * rev_per_min -> rev/sec = rpm/60 * circumference
      double revPerSec = avgRpm / 60.0;
      double ips = revPerSec * (ODOM_WHEEL_CIRC);
      return ips;
    };

    // iterate through points; for simplicity follow each sample as a target (carrot)
    int idx = 0;
    double tfrac = 0.0;
    double timer = 0.0;
    int maxIdx = (int)path.size() - 1;

    while (timer < timeoutMs) {
      // find next index to follow: choose idx where robot is not beyond next point
      // naive: advance idx if close to next
      Pose robot = getPose();
      // find closest sample index to robot
      int closest = 0;
      double bestDist = 1e9;
      for (int i=0;i<(int)path.size();++i){
        double d = hypot(path[i].x - robot.x, path[i].y - robot.y);
        if (d < bestDist) { bestDist = d; closest = i; }
      }
      idx = closest;
      if (idx >= maxIdx) { // near end, finish
        setDriveVolts(0,0);
        break;
      }

      // carrot target = lookahead along path
      double lookahead = lead; // inches
      double accum = 0.0;
      int seg = idx;
      double frac = 0.0;
      // walk forward along path segments until we've traveled lookahead or end
      while (seg < (int)path.size()-1 && accum < lookahead) {
        double sx = path[seg].x, sy = path[seg].y;
        double ex = path[seg+1].x, ey = path[seg+1].y;
        double segLen = hypot(ex-sx, ey-sy);
        if (accum + segLen >= lookahead) {
          double need = lookahead - accum;
          frac = need / segLen;
          break;
        } else {
          accum += segLen;
          seg++;
        }
      }
      // compute carrot point
      Pose pA = path[seg];
      Pose pB = path[std::min(seg+1, maxIdx)];
      double cx = pA.x + (pB.x - pA.x) * frac;
      double cy = pA.y + (pB.y - pA.y) * frac;
      // desired tangent angle at that seg (in degrees)
      double targTheta = path[seg].theta;

      // remaining along whole path approx:
      double rem = 0.0;
      for (int j = seg; j < (int)path.size()-1; ++j) {
        rem += hypot(path[j+1].x - path[j].x, path[j+1].y - path[j].y);
      }
      rem += (1.0 - frac) * hypot(pB.x - pA.x, pB.y - pA.y);

      // Profile requested linear speed (inches/sec)
      double dir = forward ? 1.0 : -1.0;
      double targetVel = profile.getTargetVelocity(rem, /*distTraveled*/0.0, dir); // distTraveled unknown here; using rem-only profile

      // convert to velocity PID setpoint (we use targetVel directly; actual measured in ips)
      double actualVel = getRobotForwardVel();
      double baseOut = velPID.compute(targetVel, actualVel); // output in volts (cap is 11)
      // angle error: minimal difference between robot theta and tangent
      double angleErr = targTheta - robot.theta;
      if (angleErr > 180) angleErr -= 360;
      if (angleErr < -180) angleErr += 360;
      // angPID produces a small voltage correction
      double angOut = angPID.compute(0.0, -angleErr); // negative because compute(target,current)
      // combine
      double leftV = baseOut - angOut;
      double rightV = baseOut + angOut;
      setDriveVolts(leftV, rightV);

      // exit condition
      if (rem < 1.0) {
        setDriveVolts(0,0);
        break;
      }

      wait(CONTROL_DT, msec);
      timer += CONTROL_DT;
    }
    // safety stop
    setDriveVolts(0,0);
  }

private:
  PID velPID; // targetVel (inches/sec) -> output volts
  PID angPID; // angular correction (degrees) -> volts
  Profile profile;
};

// ------------------------- Example helper: followMoveTo (uses BV2 for curved paths) ----------
BV2 globalBV2;

// usage: call globalBV2.generateHermite(startPose, endPose) then followPath
// convenience wrapper
void moveToBV2(double targetX, double targetY, double targetThetaDeg) {
  Pose start = getPose();
  Pose goal{targetX, targetY, targetThetaDeg};
  auto curve = globalBV2.generateHermite(start, goal, 80);
  globalBV2.followPath(curve, true, 8.0, 20000);
}

// ------------------------- Notes & Tuning Guidance -------------------------
/*
How to use:
1) In main(), after vexcodeInit():
     startOdom(Xaxis, Yaxis, inertial19);

2) For fast short motions (15s auton), use:
     fastDrive(12.0);   // 12 inches forward
     fastTurn(90.0);    // turn to 90 deg

3) For long smooth path or skills:
     Pose p0 = getPose();
     Pose p1{24, 24, 90};   // example
     auto path = globalBV2.generateHermite(p0, p1, 120);
     globalBV2.followPath(path);

Tuning:
 - odom offsets: set xOffset/yOffset to measured distances from your tracking center.
 - PID gains need real-world tuning. Start small P, increase until oscillation, then add D.
 - velPID should control robot forward velocity (inches/sec). If using motor rpm->ips conversion, ensure wheel diameter matches.
 - angPID should be small; it just nudges relative voltage to follow tangent.

Why this is better:
 - Odometry: polar-corrected arc math handles rotation-induced encoder motion (less drift for spins)
 - BV2: separates path planning (sampled curve + profile) from motor control (velocity PID + small angular correction). The profile tells *what speed* to aim for at each path point, the velocity PID makes the robot reach that speed reliably despite battery/motor differences. Angular PID keeps you on the tangent.

Caveats:
 - BV2.followPath above uses a simple lookahead carrot and a crude rem-distance profile; this is a practical midpoint between complexity and reliability.
 - For best paths, refine: compute exact traveled along path (arc length) for profile, compute path curvature to limit lateral acceleration, etc.
*/

// ------------------------- End of file -------------------------
