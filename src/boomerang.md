// Boomerang with Hermite Splines + Markers + Real Odometry
// ========================================================

#include "vex.h"
#include <cmath>
#include <vector>
#include <queue>
#include <functional>

using namespace vex;

struct Point
{
  double x, y, theta; // inches + degrees
  Point(double x = 0, double y = 0, double t = 0) : x(x), y(y), theta(t) {}
};

class PID
{
public:
  double kP, kI, kD, integral = 0, prevError = 0, outputCap;
  PID(double p, double i, double d, double cap) : kP(p), kI(i), kD(d), outputCap(cap) {}

  void reset()
  {
    integral = 0;
    prevError = 0;
  }

  double compute(double target, double current)
  {
    double err = target - current;
    integral += err;
    double deriv = err - prevError;
    prevError = err;
    double out = err * kP + integral * kI + deriv * kD;
    return std::clamp(out, -outputCap, outputCap);
  }
};

struct Pose
{
  double x = 0, y = 0, theta = 0; // inches, degrees
};
Pose getPose(); // Hook up to your odometry

void setVoltage(double L, double R)
{
  L = std::clamp(L, -11.0, 11.0);
  R = std::clamp(R, -11.0, 11.0);
  L1.spin(fwd, L, volt);
  L2.spin(fwd, L, volt);
  L3.spin(fwd, L, volt);
  R6.spin(fwd, R, volt);
  R7.spin(fwd, R, volt);
  R8.spin(fwd, R, volt);
}

struct Marker
{
  double x, y;
  std::function<void()> action;
};

class Boomerang
{
private:
  PID linearPID{0.7, 0.0, 1.2, 11};
  PID angularPID{4.5, 0.01, 0.2, 11};

  double settleLinear = 0.5, settleAngular = 1.0;
  double timeoutMs = 3000;

  bool reverse = false, asyncRunning = false;
  std::queue<Point> pathQueue;
  std::vector<Marker> markers;
  std::function<void()> onComplete = nullptr;

public:
  // Follow a single target smoothly with carrot approach
  void follow(Point tgt, double lead = 2.0)
  {
    linearPID.reset();
    angularPID.reset();
    double elapsed = 0, dt = 10;

    while (true)
    {
      Pose p = getPose();
      double dx = tgt.x - p.x, dy = tgt.y - p.y;
      double dist = sqrt(dx * dx + dy * dy);
      double angTo = atan2(dy, dx);

      double leadX = tgt.x - lead * cos(angTo);
      double leadY = tgt.y - lead * sin(angTo);
      double linErr = sqrt((leadX - p.x) * (leadX - p.x) + (leadY - p.y) * (leadY - p.y));

      double angErr = tgt.theta - p.theta;
      if (angErr > 180)
        angErr -= 360;
      if (angErr < -180)
        angErr += 360;

      double linOut = linearPID.compute(0, -linErr);
      double angOut = angularPID.compute(0, -angErr);

      double L = linOut - angOut, R = linOut + angOut;
      if (reverse)
        std::swap(L, R);
      setVoltage(L, R);

      for (auto &m : markers)
        if (fabs(p.x - m.x) < 2 && fabs(p.y - m.y) < 2)
          m.action();

      if ((fabs(linErr) < settleLinear && fabs(angErr) < settleAngular) || elapsed > timeoutMs)
        break;
      wait(dt, msec);
      elapsed += dt;
    }
    setVoltage(0, 0);
  }

  // Queue future targets
  void followAsync(Point tgt)
  {
    pathQueue.push(tgt);
    if (!asyncRunning)
      startAsync();
  }

  // Internal queue handler
  void startAsync()
  {
    asyncRunning = true;
    task([this]
         {
      while(!pathQueue.empty()){
        follow(pathQueue.front());
        pathQueue.pop();
      }
      asyncRunning = false;
      if(onComplete) onComplete(); });
  }

  void clearQueue()
  {
    std::queue<Point>().swap(pathQueue);
    markers.clear();
  }
  void setOnComplete(std::function<void()> cb) { onComplete = cb; }
  void setReverse(bool r) { reverse = r; }
  void setTimeout(double t) { timeoutMs = t; }
  void setLinearGains(double p, double i, double d) { linearPID = PID(p, i, d, 11); }
  void setAngularGains(double p, double i, double d) { angularPID = PID(p, i, d, 11); }
  void addMarker(double x, double y, std::function<void()> cb) { markers.push_back({x, y, cb}); }

  // Generate curved path using Hermite spline
  std::vector<Point> generateHermite(
      Point p0, Point p1,
      double t0x = 1, double t0y = 0, double t1x = 1, double t1y = 0,
      int samples = 30)
  {
    std::vector<Point> path;
    for (int i = 0; i <= samples; i++)
    {
      double t = i / (double)samples;
      double h00 = 2 * t * t * t - 3 * t * t + 1;
      double h10 = t * t * t - 2 * t * t + t;
      double h01 = -2 * t * t * t + 3 * t * t;
      double h11 = t * t * t - t * t;

      double x = h00 * p0.x + h10 * t0x + h01 * p1.x + h11 * t1x;
      double y = h00 * p0.y + h10 * t0y + h01 * p1.y + h11 * t1y;

      double dx = 6 * (p1.x - p0.x) * t * (1 - t);
      double dy = 6 * (p1.y - p0.y) * t * (1 - t);
      double theta = atan2(dy, dx) * 180 / M_PI;
      path.emplace_back(x, y, theta);
    }
    return path;
  }

  // Break curved path into points and follow them
  void followSpline(Point p0, Point p1)
  {
    auto curve = generateHermite(p0, p1);
    for (auto &pt : curve)
      follow(pt);
  }
};

Boomerang pathFollower;

// Hook into your odometry system
Pose getPose()
{
  return currentPose;
}
