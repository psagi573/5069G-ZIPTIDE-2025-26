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

// Ensure 'autons' and 'selectedAuton' are declared as extern if defined elsewhere

using namespace vex; // you need it so vex stuff works

competition Competition; // you need it so it works at a competition
// Global auton selector

float tovolt(float percentage)
{
  return (percentage * 12.0 / 100.0);
}

int DriveTrainControls() // we create a integer function named "DriveTrainControls", later in the code we plan to turnpid this into a Thread that controls the drivetrain
{
  Roller.stop();
  Intake.stop();
  L1.setStopping(brake);
  L2.setStopping(brake);
  L3.setStopping(brake);
  R6.setStopping(brake);
  R7.setStopping(brake);
  R8.setStopping(brake);
  Intake.setStopping(brake);

  L1.setVelocity(600, rpm);
  L2.setVelocity(600, rpm);
  L3.setVelocity(600, rpm);
  R6.setVelocity(600, rpm);
  R7.setVelocity(600, rpm);
  R8.setVelocity(600, rpm);
  Intake.setVelocity(300, rpm);

  while (true)
  {
    // Read joystick values
    int forwards = Controller1.Axis3.position(percent);
    int turning = Controller1.Axis1.position(percent);

    int leftVolt = tovolt(forwards + turning);
    int rightVolt = tovolt(forwards - turning);

    // // Spin motors
    L.spin(forward, leftVolt, volt);
    R.spin(forward, rightVolt, volt);
    wait(10, msec);
  }
}

int IntakeControls()
{
  Intake.stop();
  while (true)
  {

    if (Controller1.ButtonR1.pressing())
    {
      Trapdoor.set(false);
      Intake.spin(forward);
      waitUntil(!Controller1.ButtonR1.pressing()); // keeps it spinning until the user let go of R1
      Intake.stop();
    }
    wait(10, msec);
  }
}

int OutakeControls()
{
  while (true)
  {
    if (Controller1.ButtonR2.pressing())
    {
      Trapdoor.set(true);
      Intake.spin(forward);
      waitUntil(!Controller1.ButtonR2.pressing()); // keeps it spinning until the user let go of R2
      Intake.stop();
    }
    wait(10, msec);
  }
}

int ReverseControls()
{
  while (true)
  {
    if (Controller1.ButtonL1.pressing())
    {
      Intake.spin(reverse);
      waitUntil(!Controller1.ButtonL1.pressing()); // keeps it spinning until the user let go of L1
      Intake.stop();
    }
    wait(10, msec);
  }
}

int storeControls()
{
  while (true)
  {
    if (Controller1.ButtonL2.pressing())
    {
      Intake.spin(forward, 100, rpm);
      waitUntil(!Controller1.ButtonL2.pressing()); // keeps it spinning until the user let go of L1
      Intake.stop();
      wait(10, msec);
    }
  }
}

int Liftercontrols()
{

  bool Lifter1 = true;
  while (true)
  {
    if (Controller1.ButtonY.pressing())
    {
      if (Lifter)
      {
        Lifter1 = false;
      }
      else if (!Lifter)
      {
        Lifter1 = true;
      }
      while (Controller1.ButtonY.pressing())
      {

        wait(5, msec);
      }

      if (Lifter1)
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
  task b(IntakeControls);
  task c(OutakeControls);
  task d(ReverseControls);
  task e(storeControls);
  task f(Liftercontrols);

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
// void leftAutonQuals() {
// outake.setStopping(coast);
//   RollerIntake.setStopping(coast);

//   colorsort = task(Colorcontrols);
//   jamtask = task(jamcontrols);
//   targetColor = vex::color::blue;
//   jam = true;
//   Trap = false;
//   L1.setVelocity(600, rpm);
//   L2.setVelocity(600, rpm);
//   L3.setVelocity(600, rpm);
//   R6.setVelocity(600, rpm);
//   R7.setVelocity(600, rpm);
//   R8.setVelocity(600, rpm);
//   Out.setVelocity(200, rpm);
//   Take.setVelocity(200, rpm);
//   outake.setVelocity(200, rpm);
//   RollerIntake.setVelocity(600, rpm);

//   RollerIntake.spin(forward);
//   drive(19, 1500);
//   drive(8.5, 1500);
//   wait(0.2, sec);
//   turn(245);
//   wait(0.2, sec);
//   drive(-12, 1500);
//   outake.spin(forward);
//   wait(1, sec);
//   outake.stop();
//   drive(46, 1800);
//   turn(195);
//   Trap = true;
//   vel = true;
//   Loader.set(true);
//   Lifter.set(true);
//   wait(0.2, sec);
//   drive(13.5, 1100);
//   wait(0.1, sec);
//   drive(-10, 1500);
//   drive(-20, 1500);
//   outake.spin(forward);
// }

// void rightAutonElims() {
//   outake.setStopping(coast);
//   RollerIntake.setStopping(coast);

//   colorsort = task(Colorcontrols);
//   jamtask = task(jamcontrols);
//   targetColor = vex::color::blue;
//   jam = false;
//   Trap = false;
//   L1.setVelocity(600, rpm);
//   L2.setVelocity(600, rpm);
//   L3.setVelocity(600, rpm);
//   R6.setVelocity(600, rpm);
//   R7.setVelocity(600, rpm);
//   R8.setVelocity(600, rpm);
//   Out.setVelocity(200, rpm);
//   Take.setVelocity(200, rpm);
//   outake.setVelocity(200, rpm);
//   RollerIntake.setVelocity(600, rpm);

//   bool jam = false;
//   drive(19, 1500);
//   vel = true;
//   drive(7.5, 1500);
//   turn(120);
//   drive(38, 1300);
//   Loader.set(true);
//   Lifter.set(true);
//   wait(0.3, sec);
//   turn(165);
//   wait(0.2, sec);
//   drive(13, 1000);
//   drive(-10, 1100);
//   wait(0.2, sec);
//   drive(-20, 1000);
//   outake.spin(forward);
// }

// void leftAutonElims() {
//   outake.setStopping(coast);
//   RollerIntake.setStopping(coast);

//   colorsort = task(Colorcontrols);
//   jamtask = task(jamcontrols);
//   targetColor = vex::color::blue;
//   jam = false;
//   Trap = false;
//   L1.setVelocity(600, rpm);
//   L2.setVelocity(600, rpm);
//   L3.setVelocity(600, rpm);
//   R6.setVelocity(600, rpm);
//   R7.setVelocity(600, rpm);
//   R8.setVelocity(600, rpm);
//   Out.setVelocity(200, rpm);
//   Take.setVelocity(200, rpm);
//   outake.setVelocity(200, rpm);
//   RollerIntake.setVelocity(600, rpm);

//   drive(19, 1500);
//   vel = true;
//   drive(7.5, 1500);
//   turn(240);
//   drive(38, 1300);
//   Loader.set(true);
//   Lifter.set(true);
//   wait(0.3, sec);
//   turn(195);
//   wait(0.2, sec);
//   drive(13, 1000);
//   drive(-10, 1100);
//   wait(0.2, sec);
//   drive(-20, 1000);
//   outake.spin(forward);
// }

void pre_auton(void)
{
  vexcodeInit();

  // autonSelector.initialize();
  //   while (!Competition.isAutonomous() && !Competition.isDriverControl()) {
  //   autonSelector.update();   // <- keep checking for inputs
  //   wait(20, msec);
}

//////////////////////////////////////////////////////////////////////////
void auton() // A function named "auton", in this case, any code in the brackets will run once (unless in a loop) when its autonomous
{

  // autonSelector.runSelectedAuton();

  Intake.setStopping(coast);
  L1.setVelocity(600, rpm);
  L2.setVelocity(600, rpm);
  L3.setVelocity(600, rpm);
  R6.setVelocity(600, rpm);
  R7.setVelocity(600, rpm);
  R8.setVelocity(600, rpm);
  Intake.setVelocity(600, rpm);

  Intake.spin(forward);
  // drive(19, 1500);
  // vel = false;
  // drive(7.5, 1500);
  // turn(250);
  // Loader.set(true);
  // wait(0.3,sec);
  // drive(32.5, 1500);
  // wait(0.3, sec);
  // turn(200);
  // Lifter.set(true);
  // wait(0.2, sec);
  // drive(17.5, 1300);
  // wait(3,sec);
  // drive(-7,1500);
  // drive(7, 1500);
  // wait(3, sec);
  // drive(-16, 1100);
  // //wait(0.1, sec);
  // //turn(195);
  // drive(-15, 1000);
  // outake.spin(forward);
  // wait(9,sec);
  // Loader.set(false);
  // drive(10,1500);
  // turn(155);
  // drive(29,1500);
  // turn(110);
  // drive(50,1500);
  // wait(0.5, sec);
  // drive(0, 1500);

  // autonSelector.stopSelection();
  // autonSelector.executeSelectedAuton();
  ///////////////////////////////////////////////////////////////////////
  ////LEFT QUALS AUTON//////////
  // outake.setStopping(coast);
  // RollerIntake.setStopping(coast);

  // colorsort = task(Colorcontrols);
  // jamtask = task(jamcontrols);
  // targetColor = vex::color::blue;
  // L1.setVelocity(600, rpm);
  // L2.setVelocity(600, rpm);
  // L3.setVelocity(600, rpm);
  // R6.setVelocity(600, rpm);
  // R7.setVelocity(600, rpm);
  // R8.setVelocity(600, rpm);
  // Out.setVelocity(200, rpm);
  // Take.setVelocity(200, rpm);
  // outake.setVelocity(200, rpm);
  // RollerIntake.setVelocity(600, rpm);

  // RollerIntake.spin(forward);
  // drive(19, 1500);
  // drive(8.5, 1500);
  // wait(0.2, sec);
  // turn(240);
  // wait(0.2, sec);
  // drive(-12.5, 1500);
  // outake.spin(forward);
  // wait(1, sec);
  // outake.stop();
  // drive(46.5, 1800);
  // turn(195);
  // Loader.set(true);
  // Lifter.set(true);
  // wait(0.2, sec);
  // drive(16, 1700);
  // wait(0.4, sec);
  // drive(-10, 1500);
  // drive(-20, 1500);
  // outake.spin(forward);

  //////////////////////////////////////////////////////////////////

  // good elims left auton
  // outake.setStopping(coast);
  // RollerIntake.setStopping(coast);

  // colorsort = task(Colorcontrols);
  // jamtask = task(jamcontrols);
  // targetColor = vex::color::blue;
  // jam = true;
  // Trap = false;
  // L1.setVelocity(600, rpm);
  // L2.setVelocity(600, rpm);
  // L3.setVelocity(600, rpm);
  // R6.setVelocity(600, rpm);
  // R7.setVelocity(600, rpm);
  // R8.setVelocity(600, rpm);
  // Out.setVelocity(200, rpm);
  // Take.setVelocity(200, rpm);
  // outake.setVelocity(200, rpm);
  // RollerIntake.setVelocity(600, rpm);

  // RollerIntake.spin(forward);
  // drive(19, 1500);
  // vel = false;
  // drive(7.5, 1500);
  // turn(250);
  // Loader.set(true);
  // wait(0.3,sec);
  // drive(32.5, 1500);
  // wait(0.3, sec);
  // turn(200);
  // Lifter.set(true);
  // wait(0.2, sec);
  // drive(17.5, 1300);
  // wait(0.4,sec);
  // drive(-16, 1100);
  // //wait(0.1, sec);
  // //turn(195);
  // drive(-15, 1000);
  // outake.spin(forward);
  // wait(3,sec);
  // drive(27,1500);

  ////////////////////////////////////////////////

  // good elims left auton
  // outake.setStopping(coast);
  // RollerIntake.setStopping(coast);

  // colorsort = task(Colorcontrols);
  // jamtask = task(jamcontrols);
  // targetColor = vex::color::blue;
  // jam = true;
  // Trap = false;
  // L1.setVelocity(600, rpm);
  // L2.setVelocity(600, rpm);
  // L3.setVelocity(600, rpm);
  // R6.setVelocity(600, rpm);
  // R7.setVelocity(600, rpm);
  // R8.setVelocity(600, rpm);
  // Out.setVelocity(200, rpm);
  // Take.setVelocity(200, rpm);
  // outake.setVelocity(200, rpm);
  // RollerIntake.setVelocity(600, rpm);

  // RollerIntake.spin(forward);
  // drive(19, 1500);
  // vel = false;
  // drive(7.5, 1500);
  // turn(110);
  // Loader.set(true);
  // wait(0.3,sec);
  // drive(32.5, 1500);
  // wait(0.3, sec);
  // turn(160);
  // Lifter.set(true);
  // wait(0.2, sec);
  // drive(18, 1300);
  // wait(0.3,sec);
  // drive(-16, 1100);
  // //wait(0.1, sec);
  // //turn(195);
  // drive(-15, 1000);
  // outake.spin(forward);
  // wait(3,sec);
  // outake.stop();
  // RollerIntake.stop();
  // drive(27,1500);

  //////////////////////////////////////////////////////////////////

  // good elims Right auton

  //  outake.setStopping(coast);
  // RollerIntake.setStopping(coast);

  // colorsort = task(Colorcontrols);
  // jamtask = task(jamcontrols);
  // targetColor = vex::color::blue;
  // jam = true;
  // Trap = false;
  // L1.setVelocity(600, rpm);
  // L2.setVelocity(600, rpm);
  // L3.setVelocity(600, rpm);
  // R6.setVelocity(600, rpm);
  // R7.setVelocity(600, rpm);
  // R8.setVelocity(600, rpm);
  // Out.setVelocity(200, rpm);
  // Take.setVelocity(200, rpm);
  // outake.setVelocity(200, rpm);
  // RollerIntake.setVelocity(600, rpm);

  // RollerIntake.spin(forward);
  // drive(19, 1500);
  // vel = false;
  // drive(7.5, 1500);
  // turn(120);
  // wait(0.3,sec);
  // drive(39, 1500);
  // Loader.set(true);
  // Lifter.set(true);
  // wait(0.3, sec);
  // turn(165);
  // wait(0.2, sec);
  // drive(13, 1000);
  // drive(-15, 1100);
  // wait(0.2, sec);
  // turn(160);
  // drive(-15, 1000);
  // outake.spin(forward);
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
    Pose currentPose = Odom::getPose();

    Brain.Screen.setCursor(3, 1);
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("Pose X: %.2f", currentPose.x);
    Brain.Screen.setCursor(5, 1);
    Brain.Screen.print("Pose Y: %.2f", currentPose.y);
    Brain.Screen.setCursor(6, 1);
    Brain.Screen.print("Pose Theta: %.2f", currentPose.theta);

    Controller1.Screen.clearLine(0);
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("X: %.2f", currentPose.x);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("Y: %.2f", currentPose.y);
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("T: %.2f", currentPose.theta);
    printf("(%.2f, %.2f),", currentPose.x, currentPose.y);
    fflush(stdout);

    wait(50, msec); // Delay to avoid screen spam
  }
}
