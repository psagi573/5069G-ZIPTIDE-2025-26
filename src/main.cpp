/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Rodri                                            */
/*    Created:      Wed Sep 04 2024                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller
// L1                   motor         1
// R6                   motor         6
// Yaxis                rotation      21
// Xaxis                rotation      9
// inertial19           inertial      11
// L2                   motor         2
// L3                   motor         3
// R7                   motor         7
// R8                   motor         8
// Controller2          controller
// mogo                 digital_out   C
// WallStake            digital_out   H
// Doinker              digital_out   G
// Hood                 digital_out   A
// Hook                 motor         5
// RollerIntake         motor         10
// ---- END VEXCODE CONFIGURED DEVICES ----

#include <algorithm> //allows you to use the algorith library/extention witch lets you use mroe advanced math algorithims such as sum of, mean, median, ect
#include <cmath>     //allows you to use the cmath library/extention witch lets you do square root, absolute value, sin cos tan, ect
#include "vex.h"     //you need it so vex stuff works
// #include "auton.cpp"

using namespace vex; // you need it so vex stuff works

motor_group R = motor_group(R6, R7, R8);
motor_group L = motor_group(L1, L2, L3);
drivetrain Drivetrain = drivetrain(R, L);
motor_group Arm = motor_group(RollerIntake, Hook);

competition Competition; // you need it so it works at a competition

float tovolt(float percent)
{
  return (percent * 12.0 / 100.0);
}

int DriveTrainControls() // we create a integer function named "DriveTrainControls", later in the code we plan to turnpid this into a Thread that controls the drivetrain
{
  // IN ORDER FOR THIS TO WORK, ALL MOTORS ON THE RIGHT SIDE OF THE DRIVETRAIN MUST BE SET TO "REVERSE"

  // Makes the motors set to "coast" when they arent being used aka the joystick isnt being moved
  L1.setStopping(brake);
  L2.setStopping(brake);
  L3.setStopping(brake);
  R6.setStopping(brake);
  R7.setStopping(brake);
  R8.setStopping(brake);
  Hook.setStopping(coast);

  L1.setVelocity(100, pct);
  L2.setVelocity(100, pct);
  L3.setVelocity(100, pct);
  R6.setVelocity(100, pct);
  R7.setVelocity(100, pct);
  R8.setVelocity(100, pct);
  Hook.setVelocity(100, pct);
  RollerIntake.setVelocity(100, pct);

  /*
  There a 3 stopping modes for all motors
  "coast" --> makes the motors "ragdoll"/"die", They wont stop instantly, they will stop being powered, so the can still manually/pasively be spun
  "brake" --> makes the motors first stop, and then "ragdoll", same as coast but first itll stop them in place
  "hold" --> makes the motors stop and "hold" as the name implies, the motors stay/resist to stay at the same angle, this however drains motors and battery becouse the motor is still being powered to provide the resistance
  */
  while (true)
  {
    // Arcade Control
    R.spin(forward, tovolt(Controller1.Axis3.position(percent) - (Controller1.Axis1.position(pct) * 2)), volt);     // controlls any motors on the right side of the drivetrain
    L.spin(forward, tovolt(Controller1.Axis3.position(percent) + (Controller1.Axis1.position(percent) * 2)), volt); // controlls any motors on the left side of the drivetrain
    wait(10, msec);
    // Tank Control
    // R6.spin(forward, Controller1.Axis2.value(), percent);
    // R7.spin(forward, Controller1.Axis2.value(), percent);
    // R8.spin(forward, Controller1.Axis2.value(), percent);
    // L1.spin(forward, Controller1.Axis3.value(), percent);
    // L2.spin(forward, Controller1.Axis3.value(), percent);
    // L3.spin(forward, Controller1.Axis3.value(), percent);
    // wait(10, msec);
    /* BIG NOTE

    R.spin(forward, ( Controller1.Axis3.position(percent)-Controller1.Axis1.position(percent) ) *2, rpm);
    ---------------------------------------------------------------------------------------------^
    You may notice this 2 right here, this number is either 1, 2, or 6 depending on what kind of motor your drivetrain uses

    Red Motors --> put "1"
    Green Motors --> put "2"
    Blue Motors --> put "6"

    */
    /*
    if you had 2-3 motors per side on your drivetrain you'd do something like this:

    BackRight.spin(forward, Controller1.Axis3.position(percent)-Controller1.Axis1.position(percent), rpm);
    MiddleRight.spin(forward, Controller1.Axis3.position(percent)-Controller1.Axis1.position(percent), rpm);
    FrontRight.spin(forward, Controller1.Axis3.position(percent)-Controller1.Axis1.position(percent), rpm);
    BackLeft.spin(forward, Controller1.Axis3.position(percent)+Controller1.Axis1.position(percent), rpm);
    MiddleLeft.spin(forward, Controller1.Axis3.position(percent)+Controller1.Axis1.position(percent), rpm);
    FrontLeft.spin(forward, Controller1.Axis3.position(percent)+Controller1.Axis1.position(percent), rpm);

    Just copy paste the same code witht he extra motors, just make sure the Right side is + and the left sideis -
    */
  }
}

int ArmControls()
{
  // Arm.spin(forward); // Move arm forward when program starts
  while (true)
  {
    if (Controller1.ButtonL2.pressing())
    {
      Arm.spin(reverse);                           // Move arm forward when R1 is pressed again
      waitUntil(!Controller1.ButtonL2.pressing()); // keeps it spining until the user let go of R1
      Arm.stop();
    }
    if (Controller1.ButtonL1.pressing())
    {
      Arm.spin(forward);                           // Change direction to reverse when R2 is presse
      waitUntil(!Controller1.ButtonL1.pressing()); // keeps it spining until the user let go of R1
      Arm.stop();
    }
    if (Controller1.ButtonR2.pressing())
    {
      RollerIntake.spin(forward); // Move arm forward when R1 is pressed again
      Hook.spin(reverse, 100, rpm);
      Hood.set(true);
      waitUntil(!Controller1.ButtonR2.pressing()); // keeps it spining until the user let go of R1
      RollerIntake.stop();
      Hood.set(false);
      Hook.stop();
    }
  }
}

// int ArmControls()
//{
// while (true)
//{
// if (Controller1.ButtonL1.pressing())
//{
// Arm.spin(forward); //spins the lifter upwards if R1 is pressed
// waitUntil(!Controller1.ButtonL1.pressing()); //keeps it spining until the user let go of R1
// Arm.stop(hold); //stops the motor and holds it in place
//}
// if (Controller1.ButtonL2.pressing())
//{
// Arm.spin(reverse); //spins the lifter upwards if R2 is pressed
// waitUntil(!Controller1.ButtonL2.pressing()); //keeps it spining until the user let go of R2
// Arm.stop(hold); //stops the motor and holds it in place
//}
//}
//}

int MoGoControls()
{
  bool mogob = false;
  while (true)
  {
    if (Controller1.ButtonR1.pressing())
    {
      if (mogob)
      {
        mogob = false;
      }
      else if (!mogob)
      {
        mogob = true;
      }
      while (Controller1.ButtonR1.pressing())
      {
        wait(5, msec);
      }

      if (mogob)
      {
        Hook.spin(reverse);
        mogo.set(true);
        wait(300, msec);
        Hook.stop();
      }
      else
      {
        Hook.spin(reverse);
        mogo.set(false);
        wait(300, msec);
        Hook.stop();
      }
    }
  }
}

int BigDControls()
{
  bool BigD = false;

  while (true)
  {
    if (Controller1.ButtonB.pressing())
    {
      if (BigD)
      {
        BigD = false;
      }
      else if (!BigD)
      {
        BigD = true;
      }
      while (Controller1.ButtonB.pressing())
      {
        wait(5, msec);
      }

      if (BigD)
      {
        Doinker.set(true);
      }
      else
      {
        Doinker.set(false);
      }
    }
  }
}
/*

int RedirectControls()
{
  bool Redirectd = false;
  while (true)
  {
    if(Controller1.ButtonR2.pressing())
    {
      if(Redirectd){
        Redirectd=false;
        Hook.stop();
      }
      else if(!Redirectd){
        Redirectd=true;
        Hook.spin(forward);
      }
    while(Controller1.ButtonR2.pressing()){
    wait(5, msec);
   }

    if(Redirectd){
      Redirect.set(true);
      Hook.stop();
    }
    else{
      Redirect.set(false);
      Hook.spin(forward);
    }
  }
}
}*/

int Hstake()
{
  bool BigGc = false;
  while (true)
  {
    if (Controller1.ButtonDown.pressing())
    {
      if (BigGc)
      {
        // WallStake.set(true);
        BigGc = false;
      }
      else if (!BigGc)
      {
        // WallStake.set(false);
        BigGc = true;
      }
      while (Controller1.ButtonDown.pressing())
      {
        wait(5, msec);
      }
      if (BigGc)
      {
        WallStake.set(true);
      }
      else
      {
        WallStake.set(false);
      }
    }
  }
}

task intakeTask;
task jkfhk;
// task colorsort;
void usercontrol() // A function named "usercontrol", in this case, any code in the brackets will run once (unless in a loop) when its driver control
{
  intakeTask.stop();
  jkfhk.stop();
  // colorsort.stop();
  task a(DriveTrainControls); // creates a Thread Named "a" that runs the function "DriveTrainControls", This thread controls the drivetrain
  task b(ArmControls);        // same as drivetrain controls but for the lifter
  task c(MoGoControls);       // same as drivetrain controls but for the cla
  task d(Hstake);
  task e(BigDControls);
}
// define your global instances of motors and other devices here
void pre_auton(void)
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  {
    inertial19.calibrate();
    // 3seconds for the inertial sensor to calabrate
    // waits for the Inertial Sensor to calibrate
    // All activities that occur before the competition starts
    // Example: clearing encoders, setting servo positions, ...
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
class PID
{
private:
  const float kP, kI, kD;
  float integral = 0;
  float prevError = 0;
  float integralLimit = 1000;

public:
  PID(float p, float i, float d) : kP(p), kI(i), kD(d) {}

  float update(float Error)
  {
    integral = fmax(-integralLimit, fmin(integralLimit, integral + Error));
    float derivative = Error - prevError;
    prevError = Error;
    return kP * Error + kI * integral + kD * derivative;
  }

  void reset()
  {
    integral = 0;
    prevError = 0;
  }
};

class Odometry
{
public:
  float currentX = 0.0;
  float currentY = 0.0;
  float currentTheta = 0.0;
  float previousParallel = 0.0;
  float previousPerp = 0.0;
  float previousTheta = 0.0;
  const float dX = -4.0; // Track width adjustment
  const float dY = 0.5;

  void update()
  {
    float currentParallel = Yaxis.position(rev) * (2 * M_PI);
    float currentPerp = Xaxis.position(rev) * (2 * M_PI);

    float deltaParallel = currentParallel - previousParallel;
    float deltaPerp = currentPerp - previousPerp;
    float currentTheta = inertial19.heading() * (M_PI / 180.0);
    float deltaTheta = currentTheta - previousTheta;

    // Local displacement calculation
    float deltaXLocal, deltaYLocal;
    if (fabs(deltaTheta) < 0.01)
    { // Threshold for straight movement
      deltaXLocal = deltaPerp;
      deltaYLocal = deltaParallel;
    }
    else
    {
      deltaXLocal = 2 * sin(deltaTheta / 2) * (deltaPerp / deltaTheta + dX);
      deltaYLocal = 2 * sin(deltaTheta / 2) * (deltaParallel / deltaTheta + dY);
    }

    // Global conversion
    float avgTheta = previousTheta + deltaTheta / 2;
    currentX += deltaXLocal * cos(avgTheta) - deltaYLocal * sin(avgTheta);
    currentY += deltaXLocal * sin(avgTheta) + deltaYLocal * cos(avgTheta);

    previousTheta = currentTheta;
    previousParallel = currentParallel;
    previousPerp = currentPerp;

    Brain.Screen.clearScreen();
    Brain.Screen.printAt(10, 20, "X: %.2f Y: %.2f", currentParallel, currentPerp);
  }
  /*
      Point getPosition() {
                return new Point(currentX, currentY, currentTheta);
            }*/
//};

// int linearPID(float p, float i, float d);
// int angularPID(float tp, float ti, float td);
/*
class Point
{
public:
  float currentX;
  float currentY;
  float currentTheta;

  // Constructor
  Point(float currentX, float currentY, float currentTheta) : currentX(currentX), currentY(currentY), currentTheta(currentTheta) {}

  // Method to calculate distance between two points
  float distance(const Point &other) const
  {
    float deltaX = currentX - other.currentX;
    float deltaY = currentY - other.currentY;
    return sqrt(deltaX * deltaX + deltaY * deltaY);
  }

  // Method to convert degrees to radians  float
  float degreesToRadians(float degrees) { return degrees * M_PI / 180.0; }

  // angular error
  float angleError(const Point &other) { return other.currentTheta - currentTheta; }
};
// Add this Robot class to manage state
class Robot
{
public:
  Odometry odom;
  Point getPosition()
  {
    odom.update(); // Update position before retrieval
    return Point(odom.currentX, odom.currentY, odom.currentTheta);
  }
} robot;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// PID linearPID(0.8, 0, 0);
// PID angularPID(0.8, 0, 0);
float Perror = 0;
float Pterror = 0;

void boomerang(float targetX, float targetY, float targetTheta, float dlead)
{
  // PID linearPID(0.095, 0, 0.65);
  // PID angularPID(0.3, 0.0001, 0.23);

  PID linearPID(6, 0, 4);
  PID angularPID(6, 0, 4);

  const float settleThreshold = 0.5;
  const float angleThreshold = 0.5;

  float elapsedtime = 0;
  float timeout = 2000;

  // Use radians consistently
  float targetThetaRad = targetTheta * M_PI / 180.0;
  Point target(targetX, targetY, targetThetaRad);

  while (true)
  {
    // Get current position from odometry
    Point current = robot.getPosition();

    // Calculate distance to target
    float dist = current.distance(target);

    // Fixed carrot calculation
    Point carrot(
        target.currentX - dist * cos(current.currentTheta) * dlead,
        target.currentY - dist * sin(current.currentTheta) * dlead,
        0.0 // theta unused in carrot
    );

    // Calculate errors
    float linearError = current.distance(carrot);
    float angularError = targetThetaRad - current.currentTheta;

    // Normalize angular error
    while (angularError > M_PI)
      angularError -= 2 * M_PI;
    while (angularError < -M_PI)
      angularError += 2 * M_PI;

    // PID updates
    float linearPower = linearPID.update(linearError);
    float angularPower = angularPID.update(angularError);

    // Motor control with clamping
    float leftPower = fmax(-100, fmin(100, linearPower + angularPower));
    float rightPower = fmax(-100, fmin(100, linearPower - angularPower));

    // float leftPower = linearPower + angularPower;
    // float rightPower = linearPower - angularPower;

    L.spin(forward, leftPower, percent);
    R.spin(forward, rightPower, percent);

    if (((fabs(linearError - Perror) < settleThreshold && fabs(angularError - Pterror) < angleThreshold) && (fabs(linearError) < 1 && fabs(angularError) < 1)) || (elapsedtime >= timeout))
    {
      L.stop(brake);
      R.stop(brake);
      break;
    }

    Perror = linearError;
    Pterror = angularError;
    wait(10, msec);
    elapsedtime += 10;
  }
  L.stop(brake);
  R.stop(brake);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float kP = 0.15;
float kD = 0.9;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float turnkP = 0.038;
float turnkI = 0.0001;
float turnkD = 0.35;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// TUNE HERE// TUNE HERE// TUNE HERE// TUNE HERE// TUNE HERE// TUNE HERE//
//-----------------------------------------------------------------------------------------//
float error = 0;     // sensor - desired
float prevError = 0; // position from last loop (previous error)
float derivative = 0;
float output = 0;
float integral = 0;
float prevoutput = 0;
float TotalError = 0; // Integral Total error = totalError + error,
float settlingLimit = 0.2;
float totalTime = 150;
bool enabledrivepid;
float settlingTime = 30;

float turnerror = 0;     // sensor - desired
float turnPrevError = 0; // pos from last loop
float turnDerivative = 0.1;
float turnintegral = 0;
float turnoutput = 0;
float prevturnoutput = 0;

// Settings

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float averageposition;

void drive(float targetvalue, float timeout)
{
  float elapsedtime = 0;
  Yaxis.resetPosition();
  enabledrivepid = true;
  while (enabledrivepid)
  {
    averageposition = (Yaxis.position(deg) / 360 * (2 * M_PI));

    error = targetvalue - averageposition;
    derivative = error - prevError;

    output = (kP * error) + (kD * derivative);

    L.spin(forward, 11 * output, volt);
    R.spin(forward, 11 * output, volt);

    if ((fabs(error) < 1 && fabs(error - prevError) < 0.2) || (elapsedtime >= timeout))
    {
      break;
    }

    elapsedtime += 10;
    prevError = error;
    task::sleep(10);
  }
  L.stop(brake);
  R.stop(brake);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float restrain(float num, float min, float max)
{
  if (num > max)
    num -= (max - min);
  if (num < min)
    num += (max - min);
  return num;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool enableturnPID;

void turn(float turnTargetvalue, float timeout)
{
  // Inertial19.setHeading(0, degrees);
  // float startingPoint = Inertial19.heading(degrees);
  float elapsedtime = 0;

  enableturnPID = true;
  while (enableturnPID)
  {
    turnerror = restrain(turnTargetvalue - inertial19.heading(degrees), -180, 180);

    if (turnerror < 10 && turnerror > -10)
    {
      turnintegral += turnerror;
    }

    turnDerivative = (turnerror - turnPrevError);

    turnoutput = (turnkP * turnerror) + (turnkI * turnintegral) + (turnkD * turnDerivative);

    if (turnoutput > 1)
      turnoutput = 1;
    if (turnoutput < -1)
      turnoutput = -1;

    L.spin(forward, 11 * turnoutput, volt);
    R.spin(forward, -11 * turnoutput, volt);

    if ((turnerror > -1 && turnerror < 1 && turnerror - turnPrevError > -0.2 && turnerror - turnPrevError < 0.2) || (elapsedtime >= timeout))
    {
      break;
    }

    prevturnoutput = turnoutput;
    turnPrevError = turnerror;
    elapsedtime += 10;
    task::sleep(10);
  }
  L.stop(brake);
  R.stop(brake);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool HookJam;
// Hook.setStopping(coast);
int AntiJam()
{
  while (true)
  {
    // opti.setLight(ledState::on);
    if (!HookJam)
    {
      Hook.stop();
    }
    if ((HookJam))
    {
      Hook.spin(forward, 600, rpm);
      task::sleep(500);
      if ((Hook.velocity(rpm) < 10))
      {
        Hook.spin(reverse, 600, rpm);
        // RollerIntake.stop();
        task::sleep(400);
        Hook.spin(forward, 600, rpm);
        // RollerIntake.spin(forward,600,rpm);
        task::sleep(200);
      }
    }
  }
  return 0;
}

bool oo;
int ppg()
{
  while (true)
  {
    if (oo)
    {
      Doinker.set(true);
    }
    if (!oo)
    {
      Doinker.set(false);
    }
  }
}

float bunzv = 0;
float pp = 0;
float qq = 0;
float dir = 0;
float g = 0;
float ket = 0;
float jj; ////////////////////////////////////////////////80%/////////////////////////////
void togoal(float btargetvalue, float fkP, float fkD, float fc, bool Gsr, float btimeout)
{
  float belapsedtime = 0;
  Yaxis.resetPosition();
  jkfhk = task(ppg);
  while (true)
  {

    pp = (Yaxis.position(deg) / 360 * (2 * M_PI));
    g = btargetvalue - pp;
    dir = g - ket;
    jj = (fkP * g) + (fkD * dir);

    bunzv = (pp / btargetvalue) * 100;
    if (fc <= bunzv)
    {
      oo = Gsr;
    }
    L.spin(forward, 11 * jj, volt);
    R.spin(forward, 11 * jj, volt);

    if ((fabs(g) < 1 && fabs(g - ket) < 0.2) || (belapsedtime >= btimeout))
    {
      break;
    }

    belapsedtime += 10;
    ket = g;
    wait(10, msec);
  }
  L.stop(brake);
  R.stop(brake);
}

class DriveController
{
private:
  PID linearPID;
  PID turnPID;
  Odometry odom;

public:
  DriveController(float linP, float linI, float linD, float angP, float angI, float angD)
      : linearPID(linP, linI, linD), turnPID(angP, angI, angD) {}
  void to(float targetX, float targetY)
  {
    odom.update();
    float dx = targetX - odom.currentX;
    float dy = targetY - odom.currentY;
    float linearError = sqrt(dx * dx + dy * dy);
    float linearOutput = linearPID.update(linearError);
    float leftPower = linearOutput;
    float rightPower = linearOutput;
    L.spin(forward, leftPower, volt);  // percent
    R.spin(forward, rightPower, volt); // percent
  }
  void turnToHeading(float targetDegrees)
  {
    odom.update();
    float targetRad = targetDegrees * (M_PI / 180.0);
    float angleError = targetRad - odom.currentTheta;
    angleError = fmod(angleError + M_PI, 2 * M_PI) - M_PI;
    float turnOutput = turnPID.update(angleError);
    L.spin(forward, turnOutput, volt);
    R.spin(reverse, turnOutput, volt);
  }

  void resetControllers()
  {
    linearPID.reset();
    turnPID.reset();
  }
};
*/

#include "odom.h"
#include "motion.h"
#include "autonSelector.h"
#include "PID.h"
#include "profile.h"

/*startOdom(xRot, yRot, imu);

// Example usage
drive(24.0, 0);        // Drive 24 inches, keep heading 0°
turn(90.0);            // Turn to 90° heading
arc(30.0, 90.0, true); // Arc left, 30" radius, 90° angle
sweep(90.0, false);    // Sweep turn right 90° (left side moves)
*/
void auton() // A function named "auton", in this case, any code in the brackets will run once (unless in a loop) when its autonomous
{
  drive(24.0, 0);

  /*if (autonRoutine == "Red Left")
  {
    // Call your Red Left auton code here
    turn(90.0);
  }
  else if (autonRoutine == "Red Right")
  {
  }
  else if (autonRoutine == "Blue Left")
  {
  }
  else if (autonRoutine == "Blue Right")
  {
  }
  else if (autonRoutine == "Skills")
  {
  }
  else
  {
    // Default fallback or do nothing
  }*/
}

int main()
{

  vexcodeInit();
  startOdom(Xaxis, Yaxis, inertial19);
  Competition.autonomous(auton);          // what function to run when autonomous begins, in this case it would run the function "auton"
  Competition.drivercontrol(usercontrol); // what function to run when driver control begins, in this case it would run the function "usercontrol"

  while (true)
  {
    Pose pose = getPose();

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("X: %.2f in", pose.x);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Y: %.2f in", pose.y);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Theta: %.1f deg", pose.theta);

    wait(100, msec); // Update every 100ms
    task::sleep(10);
  }
}
