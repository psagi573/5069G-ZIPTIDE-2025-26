// BOOM.cpp - Boomerang v3.6 Hybrid Controller (implementation)
#include "BOOM.h"
#include <cmath>
#include <algorithm>

namespace BoomerangV36 {

static Config cfg;
static std::atomic<bool> inited(false);

// curvature filter
struct CurvFilter {
  double tau=0.04;
  double x=0;
  void init(double t){ tau=t; x=0; }
  double update(double v, double dt){
    if(dt<=0){ x=v; return x; }
    double alpha = dt / (tau + dt);
    x = x + alpha*(v - x);
    return x;
  }
} curvFilter;

// small utilities
static inline double clampd(double v, double a, double b) { return (v<a)?a: (v>b)?b: v; }
static inline double deg2rad(double d){ return d * M_PI/180.0; }
static inline double rad2deg(double r){ return r * 180.0/M_PI; }
static inline double wrapRad(double a){ while(a>M_PI) a -= 2.0*M_PI; while(a<-M_PI) a += 2.0*M_PI; return a; }
static inline double wrapDeg180(double a){ while(a>180) a-=360; while(a<=-180) a+=360; return a; }

// minVolt enforcement (useful when output too low to overcome friction)
static inline double applyMinVolt(double v) {
  if (fabs(v) < 1e-9) return 0.0;
  double sign = (v >= 0.0) ? 1.0 : -1.0;
  double mag = fabs(v);
  if(mag > 0 && mag < cfg.minEffectiveVolt) mag = cfg.minEffectiveVolt;
  return sign * mag;
}

// low-level command via PTOManager
static void commandDriveVolt(double leftV, double rightV, PTOManager &pto) {
  leftV  = clampd(leftV,  -cfg.maxVolt, cfg.maxVolt);
  rightV = clampd(rightV, -cfg.maxVolt, cfg.maxVolt);
  // apply minVolt only to nonzero commands to avoid motors creeping
  leftV = (fabs(leftV) > 1e-6) ? applyMinVolt(leftV) : 0.0;
  rightV = (fabs(rightV) > 1e-6) ? applyMinVolt(rightV) : 0.0;

  auto left = pto.getActiveLeftMotors();
  auto right = pto.getActiveRightMotors();
  for(auto m : left)  m->spin(vex::directionType::fwd, leftV, vex::voltageUnits::volt);
  for(auto m : right) m->spin(vex::directionType::fwd, rightV, vex::voltageUnits::volt);
}

// dynamic lookahead: combine base, distance and curvature influence
static double computeLookahead(double distToTarget, double curvature) {
  // if curvature large -> shorten lookahead; if far -> extend
  double la = cfg.baseLookahead;
  // scale with distance (so far -> larger LA)
  la += cfg.lookaheadSpeedFactor * distToTarget;
  // shorten for tight curvature
  la = la / (1.0 + 6.0 * fabs(curvature)); // tuned factor, prevents exploding LA
  la = clampd(la, cfg.minLookahead, cfg.baseLookahead * 2.0);
  return la;
}

// compute geometry (corrected lateral sign and curvature sign aligned so positive -> turn RIGHT)
static double computeGeometricCurvature(const Pose &p, double tx, double ty,
                                        double &distOut, double &headingErrOut, double &lateralOut)
{
  // robot pose
  double rx = p.x;
  double ry = p.y;
  double rtheta = p.theta * M_PI / 180.0; // radians, odom uses 0 = +Y

  // vector to target (world frame)
  double dx = tx - rx;
  double dy = ty - ry;
  double dist = sqrt(dx*dx + dy*dy);
  distOut = dist;
  if(dist < 1e-9) dist = 1e-9;

  // target angle (world frame): note atan2(dx, dy) because +Y is zero direction
  double targetAngle = atan2(dx, dy);
  double headingErr = wrapRad(targetAngle - rtheta);
  headingErrOut = headingErr;

  // lateral error (positive => robot is to the RIGHT of path): corrected sign
  // lateral = dy*cos(theta) - dx*sin(theta)
  double lateralErr = dy * cos(rtheta) - dx * sin(rtheta);
  lateralOut = lateralErr;

  // dynamic lookahead
  double la = computeLookahead(dist, 0.0); // curvature unknown yet -> start with 0

  // compute nominal lookahead point on line robot->target
  double ux = dx / dist;
  double uy = dy / dist;
  double lx = rx + ux * la;
  double ly = ry + uy * la;

  // transform lookahead point into robot local frame (rotate by -rtheta)
  double tx_r = lx - rx;
  double ty_r = ly - ry;
  double x_r =  tx_r * cos(-rtheta) - ty_r * sin(-rtheta); // lateral (right +)
  double y_r =  tx_r * sin(-rtheta) + ty_r * cos(-rtheta); // forward (front +)

  double L = sqrt(x_r*x_r + y_r*y_r);
  if(L < 1e-6) L = 1e-6;

  // classical pure-pursuit curvature (lateral_local = x_r)
  double k_geom = 2.0 * x_r / (L*L);

  // heading-based curvature term (stabilizing)
  double k_heading = 2.0 * sin(headingErr) / (la + 1e-9);

  // lateral weighting based on distance (smaller when far)
  double k_lat = cfg.K_lat * lateralErr / (dist*dist + 1e-9);

  // combine (note: we WANT positive k to mean turn RIGHT relative to robot).
  // In our sign convention x_r positive -> point to the robot's right => k_geom positive -> should produce RIGHT turn.
  // final raw curvature:
  double k = cfg.K_geom * k_geom + 0.35 * k_heading + k_lat;

  // clamp & return
  k = clampd(k, -cfg.maxCurvature, cfg.maxCurvature);
  return k;
}

// predict pose helper (same coordinate conventions)
Pose predictPose(const Pose &p, double vx_ips, double omega_rad_s, double dt) {
  Pose q = p;
  double theta = p.theta * M_PI / 180.0;
  if(fabs(omega_rad_s) < 1e-6) {
    q.x += vx_ips * sin(theta) * dt;
    q.y += vx_ips * cos(theta) * dt;
  } else {
    double r = vx_ips / omega_rad_s;
    double theta_new = theta + omega_rad_s * dt;
    q.x += r * (sin(theta_new) - sin(theta));
    q.y += -r * (cos(theta_new) - cos(theta));
    q.theta = fmod(rad2deg(theta_new) + 360.0, 360.0);
  }
  return q;
}

// high-level SLOM (straight-line override) test
static inline bool straightLineOverride(double headingErrRad, double lateralErr, double curvature) {
  double headingErrDeg = fabs(rad2deg(headingErrRad));
  if(headingErrDeg <= cfg.angleThresholdDeg) return true;
  if(fabs(lateralErr) <= cfg.lateralThresholdIn) return true;
  if(fabs(curvature) <= cfg.curvThreshold) return true;
  return false;
}

// moveToPoint implementation (blocking)
bool moveToPoint(double tx, double ty, PTOManager &pto, int timeoutMs) {
  if(!inited.load()) init(Config());

  cfg.pidHeading.reset();
  cfg.pidSpeed.reset();
  curvFilter.init(cfg.curvatureFilterTau);

  int elapsed = 0;
  const int dtMs = cfg.loopDtMs;
  const double dt = cfg.dt();

  double prevKf = 0.0;
  double lastV = 0.0;

  while(elapsed < timeoutMs) {
    Pose p = Odom::getPose();

    double dist, headingErr, lateralErr;
    double rawK = computeGeometricCurvature(p, tx, ty, dist, headingErr, lateralErr);

    // check finish
    if(dist <=  cfg.minLookahead * 0.2 || dist <= 1.0) {
      stop(pto);
      return true;
    }

    // dynamic lookahead recalculated with curvature now (recompute for better k)
    double la = computeLookahead(dist, rawK);
    // recompute curvature with chosen lookahead by repeating transform (cheap)
    // (for accuracy we do computeGeometricCurvature again but we avoid duplicated code: quick reuse)
    // (But computeGeometricCurvature uses dynamic lookahead internal - acceptable for this implementation.)

    // filter curvature
    double kf = curvFilter.update(rawK, dt);

    // Straight-line override decision
    bool goStraight = straightLineOverride(headingErr, lateralErr, kf);

    // target speed (inches/sec)
    double vTarget = cfg.maxSpeedIPS / (1.0 + cfg.speedCurvFactor * fabs(kf));
    vTarget = clampd(vTarget, cfg.minSpeedIPS, cfg.maxSpeedIPS);

    // If straight-line override -> use stronger dist behaviour: map distance to a lower speed near goal
    if(goStraight) {
      // reduce max speed modestly to favor accuracy
      vTarget = std::min(vTarget, cfg.maxSpeedIPS * 0.9);
    }

    // speed controller (convert speed error to voltage)
    double speedErr = vTarget - lastV;
    double P_F = cfg.pidSpeed.update(speedErr, dt);
    P_F = clampd(P_F, -cfg.maxVolt, cfg.maxVolt);

    // heading micro-stabilizer (PID uses radians)
    double P_T_head = cfg.pidHeading.update(headingErr, dt);

    // main curvature -> produce turn command
    // IMPORTANT: we invert sign so that positive kf results in RIGHT turn (matching VEX frame)
    // leftVolt = P_F - P_T, rightVolt = P_F + P_T, so positive P_T biases right wheel higher -> turn left.
    // To make positive curvature cause RIGHT turn we negate curvature influence on P_T.
    double P_T_curv = - kf * (P_F); // scaled by forward power

    // damping curvature influence at high speeds
    P_T_curv *= (1.0 - clampd(cfg.curvatureDamping * fabs(P_F)/cfg.maxVolt, 0.0, 0.9));

    // combine curvature and heading micro-PID
    double P_T = (1.0 - cfg.headingBlendAlpha) * P_T_curv + cfg.headingBlendAlpha * P_T_head;

    // small lateral stabilizer (kept tiny) - lateralErr positive means robot to the RIGHT of path,
    // so adding a small negative turn will move robot left -> but we want lateral->drive back to path:
    // choose sign consistent with curvature sign choices above:
    P_T += - (cfg.K_lat * lateralErr * 0.02);

    // If straight-line override -> zero curvature contribution (only micro-heading)
    if(goStraight) {
      P_T = cfg.pidHeading.update(headingErr, dt); // pure stabilizer (small)
    }

    // safety clamp and enforce minVolt when non-zero
    double Vleft = clampd(P_F - P_T, -cfg.maxVolt, cfg.maxVolt);
    double Vright = clampd(P_F + P_T, -cfg.maxVolt, cfg.maxVolt);

    // apply minVolt BEFORE sending (so small commands don't stall)
    Vleft  = (fabs(Vleft) > 1e-6) ? applyMinVolt(Vleft) : 0.0;
    Vright = (fabs(Vright) > 1e-6) ? applyMinVolt(Vright) : 0.0;

    // send to motors
    commandDriveVolt(Vleft, Vright, pto);

    // update lastV limited follow (simple first-order)
    double followRate = 6.0; // ips / s
    double dv = clampd(vTarget - lastV, -followRate * dt, followRate * dt);
    lastV += dv;

    // progress time
    vex::wait(dtMs, vex::msec);
    elapsed += dtMs;
  }

  // timeout
  stop(pto);
  return false;
}

bool moveToPose(double tx, double ty, double thetaDeg, PTOManager &pto, int timeoutMs) {
  if(!inited.load()) init(Config());
  // half for translation, half for final rotate
  int half = timeoutMs/2;
  bool ok = moveToPoint(tx, ty, pto, half);
  if(!ok) return false;

  // final heading align (blocking)
  cfg.pidFinalHeading.reset();
  int elapsed = 0;
  const int dtMs = cfg.loopDtMs;
  const double dt = cfg.dt();
  double desired = deg2rad(thetaDeg);
  while(elapsed < half) {
    Pose p = Odom::getPose();
    double rtheta = p.theta * M_PI / 180.0;
    double err = wrapRad(desired - rtheta);
    // pidFinalHeading expects radians input -> produce volt-like output
    double out = cfg.pidFinalHeading.update(err, dt);
    // convert to left/right (rotate)
    double L = -out;
    double R =  out;
    L = clampd(L, -cfg.maxVolt, cfg.maxVolt);
    R = clampd(R, -cfg.maxVolt, cfg.maxVolt);
    L = (fabs(L) > 1e-6) ? applyMinVolt(L) : 0.0;
    R = (fabs(R) > 1e-6) ? applyMinVolt(R) : 0.0;
    commandDriveVolt(L, R, pto);
    if(fabs(rad2deg(err)) <= 2.0) { stop(pto); return true; }
    vex::wait(dtMs, vex::msec);
    elapsed += dtMs;
  }
  stop(pto);
  return false;
}

void stop(PTOManager &pto) { commandDriveVolt(0.0,0.0, pto); }

void init(const Config &c) {
  cfg = c;
  curvFilter.init(cfg.curvatureFilterTau);
  inited.store(true);
}

Config getConfig(){ return cfg; }
void setConfig(const Config &c){ cfg = c; curvFilter.init(cfg.curvatureFilterTau); }

} // namespace BoomerangV36
