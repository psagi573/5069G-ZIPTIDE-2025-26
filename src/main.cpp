/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\psagi                                            */
/*    Created:      Wed Sep 03 2025                                           */
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
#include "PID.h"
#include "odometry.h"
#include "motion.h"
#include "profile.h"
#include <string>
#include "PTOManager.h"

// Ensure 'autons' and 'selectedAuton' are declared as extern if defined elsewhere

using namespace vex; // you need it so vex stuff works

competition Competition; // you need it so it works at a competition

void set()
{
  L1.setStopping(brake);
  L2.setStopping(brake);
  PTOL3.setStopping(brake);
  R6.setStopping(brake);
  R7.setStopping(brake);
  PTOR8.setStopping(brake);
  IntakePTO.setStopping(brake);
  DrivePTO.setStopping(brake);

  L1.setVelocity(600, rpm);
  L2.setVelocity(600, rpm);
  PTOL3.setVelocity(600, rpm);
  R6.setVelocity(600, rpm);
  R7.setVelocity(600, rpm);
  PTOR8.setVelocity(600, rpm);
  IntakePTO.setVelocity(600, rpm);
  DrivePTO.setVelocity(600, rpm);
}

float tovolt(float percentage)
{
  return (percentage * 12.0 / 100.0);
}



PTOManager pto(
    { &L1, &L2, &PTOL3, &LIntake }, // left motors
    { &R6, &R7, &PTOR8, &RIntake }, // right motors
    DrivePTOPiston,
    IntakePTOPiston
);


int DriveTrainControls()
{
  set();
  while (true)
  {
    float forward = Controller1.Axis3.position(percent);
    float turn = Controller1.Axis1.position(percent);

    float leftVolt = tovolt(forward + turn);
    float rightVolt = tovolt(forward - turn);

    // Spin only active motors
    auto leftActive = pto.getActiveLeftMotors();
    auto rightActive = pto.getActiveRightMotors();

    for (auto m : leftActive)
      m->spin(vex::directionType::fwd, leftVolt, vex::voltageUnits::volt);
    for (auto m : rightActive)
      m->spin(vex::directionType::fwd, rightVolt, vex::voltageUnits::volt);

    wait(10, msec);
  }
}

int IntakeControls()
{
  while (true)
  {

    if (Controller1.ButtonR1.pressing())
    {
      if (pto.getCurrentDriveMode() == DRIVE_6_MOTOR) {
        Funnel.set(false);
        Intake2.spin(forward);
        waitUntil(!Controller1.ButtonR1.pressing()); // keeps it spinning until the user let go of R2
        Intake2.stop(); 
      }
      else {
        pto.setDriveMode(DRIVE_6_MOTOR);
      }
 
    }
    wait(10, msec);
  }
}

int IntakerevControls()
{
  while (true)
  {
    if (Controller1.ButtonR2.pressing())
    {
      if (pto.getCurrentDriveMode() == DRIVE_6_MOTOR) {
        Funnel.set(false);
        Intake2.spin(reverse);
        waitUntil(!Controller1.ButtonR2.pressing()); // keeps it spinning until the user let go of R2
        Intake2.stop(); 
      }
      else {
        pto.setDriveMode(DRIVE_6_MOTOR);
      }
 
    }
    wait(10, msec);
  }
}


int OutakeControls()
{
  while (true)
  {

    if (Controller1.ButtonL1.pressing())
    {
      if (pto.getCurrentDriveMode() == DRIVE_4_MOTOR) {
        Funnel.set(true);
        Intake4.spin(forward);
        waitUntil(!Controller1.ButtonL1.pressing()); // keeps it spinning until the user let go of L1
        Intake4.stop(); 
        
      }
      else {
        pto.setDriveMode(DRIVE_4_MOTOR);
      }
 
    }
    wait(10, msec);
  }
}

int OutakerevControls()
{
  while (true)
  {
    if (Controller1.ButtonL2.pressing())
    {
      if (pto.getCurrentDriveMode() == DRIVE_4_MOTOR) {
        Funnel.set(true);
        Intake4.spin(reverse);
        waitUntil(!Controller1.ButtonL2.pressing()); // keeps it spinning until the user let go of L2
        Intake4.stop(); 
        
      }
      else {
        pto.setDriveMode(DRIVE_4_MOTOR);
      }
 
    }
    wait(10, msec);
  }
}

int slowOutakeControls()
{
  while (true)
  {
    if (Controller1.ButtonUp.pressing())
    {
      if (pto.getCurrentDriveMode() == DRIVE_4_MOTOR) {
        Funnel.set(true);
        Intake4.spin(reverse, 300, rpm);
        waitUntil(!Controller1.ButtonUp.pressing()); // keeps it spinning until the user let go of L2
        Intake4.stop(); 
        
      }
      else {
        pto.setDriveMode(DRIVE_4_MOTOR);
      }
 
    }
    wait(10, msec);
  }
}


int Loadercontrols()
{

  bool Loader1 = true;
  while (true)
  {
    if (Controller1.ButtonY.pressing())
    {
      if (Lifter)
      {
        Loader1 = false;
      }
      else if (!Lifter)
      {
        Loader1 = true;
      }
      while (Controller1.ButtonY.pressing())
      {

        wait(5, msec);
      }

      if (Loader1)
      {
        Loader.set(true);
      }
      else
      {
        Loader.set(false);
      }
    }
  }
}
int DrivePTOcontrols()
{

  bool DrivePTO = false;
  while (true)
  {
    if (Controller1.ButtonRight.pressing())
    {
      if (DrivePTO)
      {
        DrivePTO = false;
      }
      else if (!DrivePTO)
      {
        DrivePTO = true;
      }
      while (Controller1.ButtonRight.pressing())
      {

        wait(5, msec);
      }

      if (DrivePTO)
      {
        pto.setDriveMode(DRIVE_8_MOTOR);
      }
      else
      {
        pto.setDriveMode(DRIVE_6_MOTOR);
      }
    }
  }
}


int Parkcontrols()
{

  bool Park = false;
  while (true)
  {
    if (Controller1.ButtonX.pressing())
    {
      if (Park)
      {
        Park = false;
      }
      else if (!Park)
      {
        Park = true;
      }
      while (Controller1.ButtonX.pressing())
      {

        wait(5, msec);
      }

      if (Park)
      {
        Doublepark.set(true);
      }
      else
      {
        Doublepark.set(false);
      }
    }
  }
}

int Lliftercontrols()
{

  bool lifter = false;
  while (true)
  {
    if (Controller1.ButtonB.pressing())
    {
      if (lifter)
      {
        lifter = false;
      }
      else if (!lifter)
      {
        lifter = true;
      }
      while (Controller1.ButtonB.pressing())
      {

        wait(5, msec);
      }

      if (lifter)
      {
        Lifter.set(true);
      }
      else
      {
        Lifter.set(false);
      }
    }
  }
}



void usercontrol() // A function named "usercontrol", in this case, any code in the brackets will run once (unless in a loop) when its driver control
{
  task a(DriveTrainControls); // creates a Thread Named "a" that runs the function "DriveTrainControls", This thread controls the drivetrain
  task b(DrivePTOcontrols);
  task c(IntakeControls); // creates a Thread Named "b" that runs the function "IntakeControls", This thread controls the intake
  task d(OutakeControls);
  task e(Parkcontrols);
  task f(Lliftercontrols);
  task g(IntakerevControls);
  task h(OutakerevControls);
  task i(slowOutakeControls); 
  task j(Loadercontrols);
}

/*    ___           ___           ___           ___           ___           ___
     /\  \         /\__\         /\  \         /\  \         /\__\         /\  \
    /::\  \       /:/  /         \:\  \       /::\  \       /::|  |       /::\  \
   /:/\:\  \     /:/  /           \:\  \     /:/\:\  \     /:|:|  |      /:/\ \  \
  /::\~\:\  \   /:/  /  ___       /::\  \   /:/  \:\  \   /:/|:|  |__   _\:\~\ \  \
 /:/\:\ \:\__\ /:/__/  /\__\     /:/\:\__\ /:/__/ \:\__\ /:/ |:| /\__\ /\ \:\ \ \__\
 \/__\:\/:/  / \:\  \ /:/  /    /:/  \/__/ \:\  \ /:/  / \/__|:|/:/  / \:\ \:\ \/__/
      \::/  /   \:\  /:/  /    /:/  /       \:\  /:/  /      |:/:/  /   \:\ \:\__\
      /:/  /     \:\/:/  /     \/__/         \:\/:/  /       |::/  /     \:\/:/  /
     /:/  /       \::/  /                     \::/  /        /:/  /       \::/  /
     \/__/         \/__/                       \/__/         \/__/         \/__/ */

///////////////////////////////////////////////////////////////////////////

// Your autonomous functions

void pre_auton(void)
{
  vexcodeInit();
  inertial19.calibrate();
  while (inertial19.isCalibrating())
    wait(100, msec);

  // Start odometry with all drivetrain motors
    // Odom::Pose initialPose;
    // initialPose.x = 0.0; 
    // initialPose.y = 0.0;
    // initialPose.theta = 0.0; // 0=+Y, 90=+X

    // Odom::setPose(initialPose);
    // Odom::start({&Left}, {&Right}, &inertial19);

}

//////////////////////////////////////////////////////////////////////////
void auton() // A function named "auton", in this case, any code in the brackets will run once (unless in a loop) when its autonomous
{


  // ////////////SWP///////////////
  set();
  Intake2.spin(reverse);
  drive(20, 3000);
  wait(0.1, sec);
  drive(5,2000);
  turn(100);
  wait(0.1,sec);
  drive(28,2000);
  turn(156);
  Intake2.stop();
  drive(-22,1000);
  pto.setDriveMode(DRIVE_4_MOTOR);
  Funnel.set(true);
  Intake4.spin(reverse);
  wait(1, sec);
  Intake4.stop();
  pto.setDriveMode(DRIVE_6_MOTOR);
  Funnel.set(false);
  drive(15,2000);
  turn(256);
  Lifter.set(true);
  Intake2.spin(reverse);
  drive(70,3000);
  wait(0.1, sec);
  drive(11,2000);
  wait(0.3,sec);
  turn(191);
  drive(-17,2000);
  Funnel.set(true);
  wait(5,sec);
  drive(57,2000);
  /*Funnel.set(false);
  turn(156);
  Loader.set(true);
  wait(0.3,sec);
  drive(15,800);
  wait(3,sec);
  Lifter.set(false);
  drive(-33, 2000);
  Intake2.stop();
  Funnel.set(true);
  pto.setDriveMode(DRIVE_4_MOTOR);
  Intake4.spin(reverse);
  wait(1,sec);
  Intake4.stop();
  pto.setDriveMode(DRIVE_6_MOTOR);
  drive(10, 2000);
  drive(-10, 2000);*/




  /////////////////////////////////////////////////////////////////////////////////////////////////


  ////////ELIMS AUTONOMOUS 10 BLOCKS///////
  // set();
  // Intake2.spin(reverse);
  // drive(20, 3000);
  // wait(0.1, sec);
  // drive(5,2000);////first 3 blocks
  // turn(10);
  // Lifter.set(true);
  // drive(27,2000);
  // wait(0.3,sec);/// 2 block under goal
  // drive(-27,2000);
  // Intake2.stop();
  // turn(100);
  // drive(28.5,2000);
  // turn(156);//// facing match loader
  // Lifter.set(false);
  // drive(-22,1000);
  // Funnel.set(true);
  // pto.setDriveMode(DRIVE_4_MOTOR);
  // Intake4.spin(reverse);// score 6 blocks
  // Funnel.set(true);
  // Loader.set(true);
  // wait(1, sec);
  // drive(30, 2000);
  // Funnel.set(false);
  // wait(3,sec);/// Match load 3 blocks
  // drive(-30, 2000);
  // Funnel.set(true);
  // wait(1,sec);// score 3 blocks
  // Intake4.stop();
  // pto.setDriveMode(DRIVE_6_MOTOR);
  // drive(10, 2000);
  // drive(-10, 2000);// push blocks into goal further
  // Funnel.set(false);

//////////////////////////////////////////////////////////////////////////////
  
}

int main()
{
  vexcodeInit();
  pre_auton();
  Competition.autonomous(auton);          // what function to run when autonomous begins, in this case it would run the function "auton"
  Competition.drivercontrol(usercontrol); // what function to run when driver control begins, in this case it would run the function "usercontrol"

  while (true)
  {

    // Get computed position from your odometry
    // Odom::Pose currentPose = Odom::getPose();

    // Brain.Screen.setCursor(3, 1);
    // Brain.Screen.setCursor(4, 1);
    // Brain.Screen.print("Pose X: %.2f", currentPose.x);
    // Brain.Screen.setCursor(5, 1);
    // Brain.Screen.print("Pose Y: %.2f", currentPose.y);
    // Brain.Screen.setCursor(6, 1);
    // Brain.Screen.print("Pose Theta: %.2f", currentPose.theta);

    // Controller1.Screen.clearLine(0);
    // Controller1.Screen.setCursor(1, 1);
    // Controller1.Screen.print("X: %.2f", currentPose.x);
    // Controller1.Screen.setCursor(2, 1);
    // Controller1.Screen.print("Y: %.2f", currentPose.y);
    // Controller1.Screen.setCursor(3, 1);
    // Controller1.Screen.print("T: %.2f", currentPose.theta);
    // printf("(%.2f, %.2f),", currentPose.x, currentPose.y);
    // fflush(stdout);

    wait(50, msec); // Delay to avoid screen spam
  }
}
