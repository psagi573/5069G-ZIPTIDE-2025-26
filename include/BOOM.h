// BOOM.h - Boomerang v3.6 Hybrid Controller (header)
/*#pragma once
#ifndef BOOM_V3_6_H
#define BOOM_V3_6_H

#include "vex.h"
#include "odometry.h"
#include "PTOManager.h"
#include <atomic>
#include <vector>

namespace BoomerangV36 {

// Simple internal PID (you may replace with your PID class)
struct SimplePID {
  double kp=0, ki=0, kd=0;
  double integ=0, prev=0, outMin=-12, outMax=12, iMax=100;
  void reset() { integ=0; prev=0; }
  double update(double err, double dt) {
    if(dt <= 0) return 0;
    integ += err * dt;
    if(integ > iMax) integ = iMax;
    if(integ < -iMax) integ = -iMax;
    double deriv = (err - prev) / dt;
    prev = err;
    double out = kp*err + ki*integ + kd*deriv;
    if(out > outMax) out = outMax;
    if(out < outMin) out = outMin;
    return out;
  }
};

// Pose alias (re-use odom struct)
using Pose = Odom::Pose;

struct Config {
  // timing
  int loopDtMs = 8;
  double dt() const { return loopDtMs / 1000.0; }

  // voltages
  double maxVolt = 12.0;
  double minEffectiveVolt = 3.0; // enforce minimum motor voltage

  // lookahead / curvature
  double baseLookahead = 18.0;
  double minLookahead  = 6.0;
  double lookaheadSpeedFactor = 0.08; // scales lookahead with speed
  double K_lat = 0.65;                // lateral weight
  double K_geom = 1.0;                // geometric scale
  double maxCurvature = 0.35;         // clamp curvature
  double curvatureFilterTau = 0.04;   // low-pass tau

  // speed limits
  double maxSpeedIPS = 81.6;
  double speedCurvFactor = 0.8;
  double minSpeedIPS = 12.0;

  // blending
  double headingBlendAlpha = 0.08;
  double curvatureDamping = 0.7;

  // Straight-Line Override thresholds (your custom)
  double angleThresholdDeg = 5.0;    // <= 5 deg -> straight mode
  double lateralThresholdIn = 1.5;   // <= 1.5 in -> straight mode
  double curvThreshold = 0.02;       // <= 0.02 -> straight mode

  // PIDs (internal small stabilizers)
  SimplePID pidHeading = {0.01, 0.0, 0.001};      // stabilizer
  SimplePID pidSpeed   = {0.035, 0.0005, 0.0005}; // speed converter
  SimplePID pidFinalHeading = {3.2, 0.0, 0.06};   // final rotate (deg->rad handled)
};

// Public API
void init(const Config &cfg = Config());
Config getConfig();
void setConfig(const Config &cfg);

// Blocking commands (use PTOManager to send vols)
bool moveToPoint(double tx, double ty, PTOManager &pto, int timeoutMs = 3000);
bool moveToPose(double tx, double ty, double thetaDeg, PTOManager &pto, int timeoutMs = 4500);
void stop(PTOManager &pto);

// helper: predict forward pose (same convention as your odom: 0deg = +Y)
Pose predictPose(const Pose &p, double vx_ips, double omega_rad_s, double dt);

} // namespace BoomerangV36

#endif // BOOM_V3_6_H
*/