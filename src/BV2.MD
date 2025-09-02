// BV2_New.cpp
// Point-based BV2 system with STRAIGHT, TURN, SWEEP, CURVE (Hermite)
// Uses: odometry, boomerang lookahead, 1D/2D motion profiling, velocity PIDs, Hermite splines

#include "vex.h"
#include <vector>
#include <cmath>
#include <thread>
#include <atomic>

using namespace vex;

// ========================= CONFIG =========================
static const double WHEEL_BASE_IN = 12.0;
static const double MAX_VOLT = 11.0;
static const int CONTROL_DT_MS = 10;

// BV2 tuning
static const double VMAX = 24.0;    // in/sec
static const double AMAX = 48.0;    // in/sec^2
static const double OMEGA_MAX = 180.0 * M_PI/180.0;
static const double ALPHA_MAX = 360.0 * M_PI/180.0;
static const double LOOKAHEAD_IN = 8.0;

// PID gains
static const double KV_P = 0.65, KV_I = 0.002, KV_D = 0.08;
static const double KW_P = 3.2, KW_I = 0.005, KW_D = 0.12;
static const double BOOMERANG_KP_FWD = 0.9;
static const double BOOMERANG_KP_ANG = 2.0;

// Feedforward coefficients
static const double FF_V = 0.35;
static const double FF_W = 0.03;

// Tolerances
static const double POS_TOL = 0.5;
static const double ANG_TOL = 2.0;

// ==================== UTILITIES =========================
static inline double clampD(double v, double lo, double hi) { return (v<lo)?lo:(v>hi?hi:v); }
static inline double degToRad(double d){ return d*M_PI/180.0; }
static inline double radToDeg(double r){ return r*180.0/M_PI; }
static inline double wrapDeg(double a){ while(a<=-180)a+=360; while(a>180)a-=360; return a; }
static inline double wrapRad(double r){ while(r<=-M_PI)r+=2*M_PI; while(r>M_PI)r-=2*M_PI; return r; }

// ==================== STRUCTS ===========================
struct Pose2D { double x=0,y=0,theta=0; };
struct Waypoint { double x=0,y=0,theta=0; Waypoint(double X, double Y, double T):x(X),y(Y),theta(T){} };
using Path = std::vector<Waypoint>;

// Movement types
enum class MoveType { STRAIGHT, TURN, SWEEP, CURVE };

// ==================== SIMPLE PID ========================
class PID {
public:
  PID(double p=0,double i=0,double d=0){setGains(p,i,d); reset();}
  void setGains(double p,double i,double d){kP=p;kI=i;kD=d;}
  void reset(){integral=0;prevError=0;first=true;}
  double compute(double setpoint,double measure,double dt){
    double err = setpoint-measure;
    integral += err*dt;
    double deriv = (!first)?(err-prevError)/dt:0.0;
    prevError=err; first=false;
    return kP*err + kI*integral + kD*deriv;
  }
private:
  double kP=0,kI=0,kD=0;
  double integral=0, prevError=0;
  bool first=true;
};

// ==================== MOTION PROFILER ===================
struct MotionProfile1D {
  double v=0,vmax,amax;
  MotionProfile1D(double vmax_, double amax_):vmax(vmax_),amax(amax_){}
  double stepToTarget(double targetRemaining,double dt){
    double sign = (targetRemaining>=0)?1.0:-1.0;
    double dist = fabs(targetRemaining);
    double v_allowed = sqrt(2*amax*dist);
    double v_goal = std::min(vmax,v_allowed);
    if(fabs(v)<v_goal){ v+=sign*amax*dt; if(fabs(v)>v_goal) v=sign*v_goal; }
    else { v-=sign*amax*dt; if(sign*v<0)v=0; }
    if(fabs(v)>vmax) v=sign*vmax;
    return v;
  }
};

// ==================== BOOMERANG LOOKAHEAD ===============
struct Boomerang {
  double lookahead;
  Boomerang(double la):lookahead(la){}
  std::pair<int,std::pair<double,double>> getLookaheadPoint(const Path &path, const Pose2D &pose){
    if(path.empty()) return {-1,{pose.x,pose.y}};
    for(size_t i=0;i+1<path.size();++i){
      double x1=path[i].x, y1=path[i].y, x2=path[i+1].x, y2=path[i+1].y;
      double vx=x2-x1, vy=y2-y1, wx=pose.x-x1, wy=pose.y-y1;
      double segLen = sqrt(vx*vx+vy*vy);
      if(segLen<1e-6) continue;
      double proj = clampD((wx*vx+wy*vy)/(segLen*segLen),0.0,1.0);
      double px=x1+proj*vx, py=y1+proj*vy;
      double px0=pose.x, py0=pose.y;
      double A=vx*vx+vy*vy, B=2*(vx*(x1-px0)+vy*(y1-py0)), C=(x1-px0)*(x1-px0)+(y1-py0)*(y1-py0)-lookahead*lookahead;
      double disc = B*B-4*A*C; if(disc<0) continue;
      double sqrtD=sqrt(disc), s1=(-B+sqrtD)/(2*A), s2=(-B-sqrtD)/(2*A);
      double s=-1; if(s1>=0 && s1<=1) s=s1; else if(s2>=0 && s2<=1) s=s2;
      if(s>=0){ double lx=x1+s*vx, ly=y1+s*vy; return {(int)i,{lx,ly}}; }
    }
    const Waypoint &last = path.back();
    return {(int)path.size()-1,{last.x,last.y}};
  }
};

// ==================== HERMITE SPLINE GENERATOR ===========
struct HermiteSpline {
  // Simple cubic Hermite interpolation between two points with optional tangent vectors
  static Waypoint interpolate(const Waypoint &p0,const Waypoint &p1,double t){
    double h00 = 2*t*t*t - 3*t*t +1;
    double h10 = t*t*t - 2*t*t + t;
    double h01 = -2*t*t*t +3*t*t;
    double h11 = t*t*t - t*t;
    double tx0=cos(degToRad(p0.theta)), ty0=sin(degToRad(p0.theta));
    double tx1=cos(degToRad(p1.theta)), ty1=sin(degToRad(p1.theta));
    double x = h00*p0.x + h10*tx0 + h01*p1.x + h11*tx1;
    double y = h00*p0.y + h10*ty0 + h01*p1.y + h11*ty1;
    double theta = h00*p0.theta + h01*p1.theta; // approximate
    return {x,y,theta};
  }
};

// ==================== BV2 CONTROLLER ====================
class BV2Controller{
public:
  BV2Controller():linearPID(KV_P,KV_I,KV_D),angularPID(KW_P,KW_I,KW_D),
  linearProfiler(VMAX,AMAX),angularProfiler(OMEGA_MAX,ALPHA_MAX),boom(LOOKAHEAD_IN),
  running(false){}
  
  void MoveTo(const Waypoint &target, MoveType type, const Path &curvePath={}){
    path.clear();
    if(type==MoveType::CURVE) path = curvePath;
    else path.push_back(target);
    moveType = type;
    running=true;
    linearPID.reset(); angularPID.reset();
    linearProfiler.v=0; angularProfiler.v=0;
    controllerThread = std::thread(&BV2Controller::controlLoop,this,target);
    if(controllerThread.joinable()) controllerThread.join();
  }
  
private:
  PID linearPID, angularPID;
  MotionProfile1D linearProfiler, angularProfiler;
  Boomerang boom;
  Path path;
  MoveType moveType;
  std::atomic<bool> running;
  std::thread controllerThread;
  
  Pose2D fetchPose(){ Pose p=getPose(); return {p.x,p.y,p.theta}; }
  double measureLinearVelocity(){ return 0; } // implement from odom
  double measureAngularVelocity(){ return 0; } // implement from odom
  void setDrive(double l,double r){ /* map to motors */ }
  
  void controlLoop(Waypoint target){
    const double dt=CONTROL_DT_MS/1000.0;
    while(running){
      Pose2D pose = fetchPose();
      double dx=target.x-pose.x, dy=target.y-pose.y;
      double dist = sqrt(dx*dx+dy*dy);
      double headingError = wrapDeg(target.theta-pose.theta);
      
      if(moveType==MoveType::TURN){ dist=0; dx=dy=0; }
      
      // Boomerang forward/lateral
      double thetaRad=degToRad(pose.theta);
      double fwdX=cos(thetaRad), fwdY=sin(thetaRad);
      double forwardComp = dx*fwdX + dy*fwdY;
      
      double v_cmd=linearProfiler.stepToTarget(forwardComp,dt);
      double w_cmd=angularProfiler.stepToTarget(headingError,dt);
      
      double v_fb = linearPID.compute(v_cmd, measureLinearVelocity(), dt);
      double w_fb = angularPID.compute(w_cmd, measureAngularVelocity(), dt);
      
      double left = v_fb - w_fb*WHEEL_BASE_IN/2.0;
      double right = v_fb + w_fb*WHEEL_BASE_IN/2.0;
      left = clampD(left,-VMAX,VMAX);
      right = clampD(right,-VMAX,VMAX);
      
      setDrive(left,right);
      
      if(dist<POS_TOL && fabs(headingError)<ANG_TOL){ running=false; break; }
      wait(CONTROL_DT_MS, msec);
    }
    setDrive(0,0);
  }
};

BV2Controller BV2;
















/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BV2_Full.cpp
// Full-featured BV2 = Boomerang + Velocity PIDs + 2D Motion Profiling + Action Triggers
#include "vex.h"
#include <vector>
#include <cmath>
#include <atomic>
#include <functional>

using namespace vex;

// ========================= CONFIG =========================
static const double WHEEL_BASE_IN = 12.0;     
static const double MAX_VOLT = 11.0;

static const int CONTROL_DT_MS = 10;          

// BV2 tuning
static const double VMAX = 24.0;       // in/s
static const double AMAX = 48.0;       // in/s^2
static const double OMEGA_MAX = M_PI;  // rad/s
static const double ALPHA_MAX = 2*M_PI; // rad/s^2

// Boomerang gains
static const double BOOMERANG_KP_FWD = 0.9;
static const double BOOMERANG_KP_ANG = 2.0;
static const double LOOKAHEAD_IN = 8.0;

// Velocity PID gains
static const double KV_P = 0.65, KV_I = 0.002, KV_D = 0.08;
static const double KW_P = 3.2, KW_I = 0.005, KW_D = 0.12;

// Feedforward coefficients
static const double FF_V = 0.35;
static const double FF_W = 0.03;

// Stopping tolerances
static const double POS_TOLERANCE_IN = 0.5;
static const double ANG_TOLERANCE_DEG = 2.0;

// ========================= UTILITIES =========================
static inline double clampD(double v, double lo, double hi){ return (v<lo?lo:(v>hi?hi:v)); }
static inline double degToRad(double d){ return d*M_PI/180.0; }
static inline double radToDeg(double r){ return r*180.0/M_PI; }
static inline double wrapDeg(double a){ while(a<=-180)a+=360; while(a>180)a-=360; return a; }
static inline double wrapRad(double r){ while(r<=-M_PI) r+=2*M_PI; while(r>M_PI) r-=2*M_PI; return r; }

// ========================= PID =========================
class PID{
public:
    PID(double p=0,double i=0,double d=0){ setGains(p,i,d); reset(); }
    void setGains(double p,double i,double d){ kP=p; kI=i; kD=d; }
    void reset(){ integral=0.0; prevError=0.0; first=true; }
    double compute(double setpoint,double measurement,double dt){
        double err=setpoint-measurement;
        integral += err*dt;
        double deriv=0.0;
        if(!first) deriv = (err-prevError)/dt;
        prevError=err; first=false;
        return kP*err + kI*integral + kD*deriv;
    }
private:
    double kP=0,kI=0,kD=0;
    double integral=0,prevError=0;
    bool first=true;
};

// ========================= POSE & PATH =========================
struct Pose2D{ double x,y,theta; }; 

struct Waypoint{
    double x,y,heading;
    std::function<void()> preAction;
    std::function<void()> postAction;
    std::function<void(const Pose2D&)> midAction;

    Waypoint(double _x=0,double _y=0,double _h=0) 
        : x(_x), y(_y), heading(_h),
          preAction(nullptr), postAction(nullptr), midAction(nullptr) {}
};

using Path = std::vector<Waypoint>;

// ========================= MOTION PROFILER =========================
struct MotionProfile1D{
    double v=0.0;
    double vmax,amax;
    MotionProfile1D(double vmax_,double amax_):vmax(vmax_),amax(amax_){}
    double stepToTarget(double targetRem,double dt){
        double sign=(targetRem>=0?1.0:-1.0);
        double dist=fabs(targetRem);
        double v_allowed=sqrt(2*amax*dist);
        double v_goal=std::min(vmax,v_allowed);

        if(fabs(v)<v_goal){ v+=sign*amax*dt; if(fabs(v)>v_goal) v=sign*v_goal; }
        else{ v-=sign*amax*dt; if(sign*v<0)v=0; }
        if(fabs(v)>vmax) v=sign*vmax;
        return v;
    }
};

// ========================= BOOMERANG =========================
struct Boomerang{
    double lookahead;
    Boomerang(double la):lookahead(la){}
    std::pair<int,std::pair<double,double>> getLookaheadPoint(const Path &path,const Pose2D &pose){
        if(path.empty()) return {-1,{pose.x,pose.y}};
        for(size_t i=0;i+1<path.size();++i){
            double x1=path[i].x,y1=path[i].y;
            double x2=path[i+1].x,y2=path[i+1].y;
            double vx=x2-x1,vy=y2-y1;
            double wx=pose.x-x1,wy=pose.y-y1;
            double segLen=sqrt(vx*vx+vy*vy);
            if(segLen<1e-6) continue;
            double proj=clampD((wx*vx+wy*vy)/(segLen*segLen),0.0,1.0);
            double px=x1+proj*vx,py=y1+proj*vy;




            double px0=pose.x,py0=pose.y;
            double A=vx*vx+vy*vy;
            double B=2*(vx*(x1-px0)+vy*(y1-py0));
            double C=(x1-px0)*(x1-px0)+(y1-py0)*(y1-py0)-lookahead*lookahead;
            double disc=B*B-4*A*C;
            if(disc<0) continue;
            double sqrtD=sqrt(disc);
            double s1=(-B+sqrtD)/(2*A), s2=(-B-sqrtD)/(2*A);
            double s=-1;
            if(s1>=0 && s1<=1) s=s1;
            else if(s2>=0 && s2<=1) s=s2;
            if(s>=0){ return {(int)i,{x1+s*vx,y1+s*vy}}; }
        }
        const Waypoint &last=path.back();
        return {(int)path.size()-1,{last.x,last.y}};
    }
};

// ========================= BV2 CONTROLLER =========================
class BV2Controller{
public:
    BV2Controller()
    : linearPID(KV_P,KV_I,KV_D), angularPID(KW_P,KW_I,KW_D),
      linearProfiler(VMAX,AMAX), angularProfiler(OMEGA_MAX,ALPHA_MAX),
      boom(LOOKAHEAD_IN), running(false), pathIndex(0) {}

    void startPath(const Path &p,bool resetPoseToFirst=false){
        if(p.empty()) return;
        path=p;
        pathIndex=0;
        wpPreTriggered.assign(path.size(),false);
        wpPostTriggered.assign(path.size(),false);

        if(resetPoseToFirst){ /* implement odom reset if needed */ }

        linearPID.reset(); angularPID.reset();
        linearProfiler.v=0; angularProfiler.v=0;
        running=true;
        controllerThread=std::thread(&BV2Controller::controlLoop,this);
    }

    void stop(){
        running=false;
        if(controllerThread.joinable()) controllerThread.join();
        stopDrive();
    }

    bool isRunning() const { return running; }

private:
    PID linearPID, angularPID;
    MotionProfile1D linearProfiler, angularProfiler;
    Boomerang boom;

    Path path;
    std::atomic<bool> running;
    std::thread controllerThread;
    std::atomic<int> pathIndex;

    std::vector<bool> wpPreTriggered, wpPostTriggered;

    void stopDrive(){ setDrive(0.0,0.0); wait(20,msec); }

    void controlLoop(){
        const double dt = CONTROL_DT_MS/1000.0;
        while(running){
            Pose2D pose = fetchPose();
            if(path.empty()){ running=false; break; }
            Waypoint &wp = path[pathIndex];

            // PRE-ACTION
            if(wp.preAction && !wpPreTriggered[pathIndex]){
                wp.preAction(); wpPreTriggered[pathIndex]=true;
            }
            // MID-ACTION
            if(wp.midAction) wp.midAction(pose);

            // Lookahead and Boomerang calculations
            auto look = boom.getLookaheadPoint(path,pose);
            double dx = look.second.first - pose.x;
            double dy = look.second.second - pose.y;
            double distToLA = sqrt(dx*dx+dy*dy);

            double thetaRad = degToRad(pose.theta);
            double robotFwdX = cos(thetaRad);
            double robotFwdY = sin(thetaRad);

            double forwardComp = dx*robotFwdX + dy*robotFwdY;
            double lateralComp = -dx*robotFwdY + dy*robotFwdX;

            double v_geom = BOOMERANG_KP_FWD*forwardComp;
            double omega_geom = clampD(BOOMERANG_KP_ANG*(lateralComp/LOOKAHEAD_IN),-1.0,1.0)*degToRad(90.0);

            // Profiling
            Waypoint lastWP=path.back();
            double dxEnd=lastWP.x-pose.x,dyEnd=lastWP.y-pose.y;
            double distToEnd = sqrt(dxEnd*dxEnd+dyEnd*dyEnd);

            double v_set = linearProfiler.stepToTarget((forwardComp>=0?distToEnd:-distToEnd), dt);
            if(v_geom<0) v_set=-fabs(v_set);

            double curvature=fabs(omega_geom)/degToRad(90.0);
            double v_curved_max=VMAX*clampD(1.0-0.6*curvature,0.2,1.0);
            if(fabs(v_set)>v_curved_max) v_set=(v_set>=0?v_curved_max:-v_curved_max);

            double omega_set=angularProfiler.stepToTarget(omega_geom, dt);

            // Heading correction near endpoint
            double headingErrorDeg=0.0;
            if(distToEnd<12.0){
                double desiredHeading=lastWP.heading;
                if(fabs(desiredHeading)<1e-6) desiredHeading=radToDeg(atan2(dyEnd,dxEnd));
                headingErrorDeg=wrapDeg(desiredHeading-pose.theta);
                omega_set += degToRad(0.02*headingErrorDeg);
            }

            // Velocity measurement
            double v_meas = measureLinearVelocity();
            double omega_meas = measureAngularVelocity();

            // PID + feedforward
            double v_out_volts = FF_V*v_set + linearPID.compute(v_set,v_meas,dt);
            double w_out_volts = FF_W*omega_set*(WHEEL_BASE_IN/2.0) + angularPID.compute(omega_set,omega_meas,dt);

            // Convert to left/right voltages
            double leftVolts = v_out_volts - (w_out_volts*(WHEEL_BASE_IN/2.0)/12.0);
            double rightVolts = v_out_volts + (w_out_volts*(WHEEL_BASE_IN/2.0)/12.0);

            leftVolts = clampD(leftVolts,-MAX_VOLT,MAX_VOLT);
            rightVolts = clampD(rightVolts,-MAX_VOLT,MAX_VOLT);

            setDrive(leftVolts,rightVolts);

            // POST-ACTION and waypoint advance
            if(distToEnd<POS_TOLERANCE_IN && wp.postAction && !wpPostTriggered[pathIndex]){
                wp.postAction(); wpPostTriggered[pathIndex]=true;
                if(pathIndex+1<path.size()) pathIndex++;
            }

            // Check finish condition
            if(distToEnd<POS_TOLERANCE_IN && fabs(radToDeg(omega_meas))<5.0 && fabs(headingErrorDeg)<ANG_TOLERANCE_DEG){
                setDrive(0.0,0.0); wait(50,msec); running=false; break;
            }

            wait(CONTROL_DT_MS,msec);
        }
        setDrive(0.0,0.0);
    }

    // ------------------ HOOKS ------------------
    Pose2D fetchPose(){
        Pose p = getPose(); // map to your existing odom
        return {p.x,p.y,p.theta};
    }
    double measureLinearVelocity(){
        static double lastY=0.0,lastTime=Brain.timer(msec);
        double now=Brain.timer(msec);
        double dt=(now-lastTime)/1000.0;
        double y=getPose().ySensor; 
        double v=(dt>1e-6?(y-lastY)/dt:0.0);
        lastY=y; lastTime=now;
        return v;
    }
    double measureAngularVelocity(){
        static double lastTheta=0.0,lastTime=Brain.timer(msec);
        double now=Brain.timer(msec);
        double dt=(now-lastTime)/1000.0;
        double theta=getPose().theta;
        double omega=(dt>1e-6?degToRad(wrapDeg(theta-lastTheta))/dt:0.0);
        lastTheta=theta; lastTime=now;
        return omega;
    }
};

// ----------------- SINGLE GLOBAL BV2 INSTANCE -----------------
static BV2Controller BV2;

// ================== EXAMPLE USAGE ==================
// Path p; 
// p.push_back(Waypoint(0,0,0));
// p.push_back(Waypoint(48,0,0));
// p[1].preAction=[](){runIntakeForward();};
// BV2.startPath(p,false);
// while(BV2.isRunning()){wait(10,msec);}
// BV2.stop();
