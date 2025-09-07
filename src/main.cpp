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
#include "odom.h"
#include "motion.h"
#include "autonSelector.h"
#include "profile.h"

using namespace vex; // you need it so vex stuff works

motor_group R = motor_group(R6, R7, R8);
motor_group L = motor_group(L1, L2, L3);
drivetrain Drivetrain = drivetrain(R, L);
// motor_group outake = motor_group(Out, Take);
// motor_group scorer = motor_group(Out, Take, RollerIntake);

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
  // outake.setStopping(coast);
  RollerIntake.setStopping(coast);

  L1.setVelocity(600, rpm);
  L2.setVelocity(600, rpm);
  L3.setVelocity(600, rpm);
  R6.setVelocity(600, rpm);
  R7.setVelocity(600, rpm);
  R8.setVelocity(600, rpm);
  // outake.setVelocity(600, rpm);
  RollerIntake.setVelocity(600, rpm);

  while (true)
  {
    // Arcade Control
    R.spin(forward, tovolt(Controller1.Axis3.position(percent) - (Controller1.Axis1.position(pct) * 6)), volt);     // controlls any motors on the right side of the drivetrain
    L.spin(forward, tovolt(Controller1.Axis3.position(percent) + (Controller1.Axis1.position(percent) * 6)), volt); // controlls any motors on the left side of the drivetrain
    wait(10, msec);
  }
} /*
 int ArmControls()
 {
   while (true)
   {
     if (Controller1.ButtonR1.pressing())
     {
       scorer.spin(forward);                        // Move arm forward when R1 is pressed again
       waitUntil(!Controller1.ButtonR1.pressing()); // keeps it spining until the user let go of R1
       scorer.stop();
     }
     wait(10, msec);
   }
 }

 int AControls()
 {
   while (true)
   {
     if (Controller1.ButtonR2.pressing())
     {
       outake.spin(reverse);                        // Change direction to reverse when R2 is pressed
       waitUntil(!Controller1.ButtonR2.pressing()); // keeps it spining until the user let go of R1
       outake.stop();
     }
     wait(10, msec);
   }
 }

 int rControls()
 {
   while (true)
   {

     if (Controller1.ButtonL1.pressing())
     {
       RollerIntake.spin(forward);
       waitUntil(!Controller1.ButtonL1.pressing()); // keeps it spinning until the user let go of R1
       RollerIntake.stop();
     }
     wait(10, msec);
   }
 }

 int mControls()
 {
   while (true)
   {
     if (Controller1.ButtonL2.pressing())
     {
       RollerIntake.spin(reverse);
       waitUntil(!Controller1.ButtonL2.pressing()); // keeps it spinning until the user let go of R1
       RollerIntake.stop();
     }
     wait(10, msec);
   }
 }

 int LifterControls()
 {
   bool Lifter1 = false;
   while (true)
   {
     if (Controller1.ButtonB.pressing())
     {
       if (Lifter1)
       {
         Lifter1 = false;
       }
       else if (!Lifter1)
       {
         Lifter1 = true;
       }
       while (Controller1.ButtonB.pressing())
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
 */
// task colorsort;
void usercontrol() // A function named "usercontrol", in this case, any code in the brackets will run once (unless in a loop) when its driver control
{
  task a(DriveTrainControls); // creates a Thread Named "a" that runs the function "DriveTrainControls", This thread controls the drivetrain
  // task b(ArmControls);        // same as drivetrain controls but for the lifter
  // task c(LifterControls);
  // task d(AControls);
  // task e(rControls);
  // task f(mControls);
}

void pre_auton(void)
{
  vexcodeInit();
  autonSelector();
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

//////////////////////////////////////////////////////////////////////////
void auton() // A function named "auton", in this case, any code in the brackets will run once (unless in a loop) when its autonomous
{
  runAuton();
}

int main()
{

  vexcodeInit();
  startOdom(Xaxis, Yaxis, inertial19);
  pre_auton();
  Competition.autonomous(auton);          // what function to run when autonomous begins, in this case it would run the function "auton"
  Competition.drivercontrol(usercontrol); // what function to run when driver control begins, in this case it would run the function "usercontrol"

  while (true)
  {
    // Get raw encoder values
    double xEnc = Xaxis.position(turns);
    double yEnc = Yaxis.position(turns);
    double heading = inertial19.rotation();

    // Get computed position from your odometry
    Pose currentPose = getPose();

    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("X Encoder: %.2f", xEnc);
    Brain.Screen.setCursor(2, 1);
    Brain.Screen.print("Y Encoder: %.2f", yEnc);
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("Heading: %.2f", heading);
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
    printf("(%.2f, %.2f),", currentPose.x, currentPose.y);
    fflush(stdout);

    wait(50, msec); // Delay to avoid screen spam
  }
}
