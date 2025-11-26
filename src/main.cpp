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
#include "PTO.h"
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

  while (true)
  {
    // Read joystick values
    int forwards = Controller1.Axis3.position(percent);
    int turning = Controller1.Axis1.position(percent);

    int leftVolt = tovolt(forwards + turning);
    int rightVolt = tovolt(forwards - turning);
     
    
    // // Spin motors
    if (getCurrentDriveMode() == DRIVE_4_MOTOR) {
      L1.spin(forward, leftVolt, volt);
      L2.spin(forward, leftVolt, volt);
      R6.spin(forward, rightVolt, volt);
      R7.spin(forward, rightVolt, volt);
    } if (getCurrentDriveMode() == DRIVE_6_MOTOR) {
      L1.spin(forward, leftVolt, volt);
      L2.spin(forward, leftVolt, volt);
      PTOL3.spin(forward, leftVolt, volt);
      R6.spin(forward, rightVolt, volt);
      R7.spin(forward, rightVolt, volt);
      PTOR8.spin(forward, rightVolt, volt);
    } if (getCurrentDriveMode() == DRIVE_8_MOTOR) {
      L1.spin(forward, leftVolt, volt);
      L2.spin(forward, leftVolt, volt);
      PTOL3.spin(forward, leftVolt, volt);
      LIntake.spin(forward, leftVolt, volt);
      R6.spin(forward, rightVolt, volt);
      R7.spin(forward, rightVolt, volt);
      PTOR8.spin(forward, rightVolt, volt);
      RIntake.spin(forward, rightVolt, volt);
}
  
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
      Intake2.spin(forward);
      waitUntil(!Controller1.ButtonR1.pressing()); // keeps it spinning until the user let go of R1
      Intake2.stop();
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
      Intake2.spin(reverse);
      waitUntil(!Controller1.ButtonR2.pressing()); // keeps it spinning until the user let go of R2
      Intake2.stop();
    }
    wait(10, msec);
  }
}

// int ReverseControls()
// {
//   while (true)
//   {
//     if (Controller1.ButtonL1.pressing())
//     {
//       Intake.spin(reverse);
//       waitUntil(!Controller1.ButtonL1.pressing()); // keeps it spinning until the user let go of L1
//       Intake.stop();
//     }
//     wait(10, msec);
//   }
// }

// int storeControls()
// {
//   while (true)
//   {
//     if (Controller1.ButtonL2.pressing())
//     {
//       Intake.spin(forward);
//       waitUntil(!Controller1.ButtonL2.pressing()); // keeps it spinning until the user let go of L1
//       Intake.stop();
//       wait(10, msec);
//     }
//   }
// }

// int Liftercontrols()
// {

//   bool Lifter1 = true;
//   while (true)
//   {
//     if (Controller1.ButtonB.pressing())
//     {
//       if (Lifter)
//       {
//         Lifter1 = false;
//       }
//       else if (!Lifter)
//       {
//         Lifter1 = true;
//       }
//       while (Controller1.ButtonB.pressing())
//       {

//         wait(5, msec);
//       }

//       if (Lifter1)
//       {
//         Lifter.set(true);
//       }
//       else
//       {
//         Lifter.set(false);
//       }
//     }
//   }
// }




// int Allignercontrols()
// {

//   bool alligner1 = true;
//   while (true)
//   {
//     if (Controller1.ButtonRight.pressing())
//     {
//       if (alligner1)
//       {
//         alligner1 = false;
//       }
//       else if (!alligner1)
//       {
//         alligner1 = true;
//       }
//       while (Controller1.ButtonRight.pressing())
//       {

//         wait(5, msec);
//       }

//       if (alligner1)
//       {
//         Alligner.set(true);
//       }
//       else
//       {
//         Alligner.set(false);
//       }
//     }
//   }
// }



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
        setDriveMode(DRIVE_8_MOTOR);
      }
      else
      {
        setDriveMode(DRIVE_6_MOTOR);
      }
    }
  }
}

int IntakePTOcontrols()
{

  bool IntakePTO = false;
  while (true)
  {
    if (Controller1.ButtonLeft.pressing())
    {
      if (IntakePTO)
      {
        IntakePTO = false;
      }
      else if (!IntakePTO)
      {
        IntakePTO = true;
      }
      while (Controller1.ButtonLeft.pressing())
      {

        wait(5, msec);
      }

      if (IntakePTO)
      {
        setDriveMode(DRIVE_4_MOTOR);
      }
      else
      {
        setDriveMode(DRIVE_6_MOTOR);
      }
    }
  }
}


// int Hookcontrols()
// {

//   bool hook1 = true;
//   while (true)
//   {
//     if (Controller1.ButtonY.pressing())
//     {
//       if (hook1)
//       {
//         hook1 = false;
//       }
//       else if (!hook1)
//       {
//         hook1=true;
//       }
//       while (Controller1.ButtonY.pressing())
//       {

//         wait(5, msec);
//       }

//       if (hook1)
//       {
//         Hook.set(true);
//       }
//       else
//       {
//         Hook.set(false);
//       }
//     }
//   }
// }




// int Loadercontrols()
// {

//   bool loader1 = true;
//   while (true)
//   {
//     if (Controller1.ButtonDown.pressing())
//     {
//       if (loader1)
//       {
//         loader1 = false;
//       }
//       else if (!loader1)
//       {
//         loader1 = true;
//       }
//       while (Controller1.ButtonDown.pressing())
//       {

//         wait(5, msec);
//       }

//       if (loader1)
//       {
//         Alligner.set(false);
//         Loader.set(true);
//       }
//       else
//       {
//         Alligner.set(true);
//         Loader.set(false);
//       }
//     }
//   }
// }

void usercontrol() // A function named "usercontrol", in this case, any code in the brackets will run once (unless in a loop) when its driver control
{
  task a(DriveTrainControls); // creates a Thread Named "a" that runs the function "DriveTrainControls", This thread controls the drivetrain
  task b(IntakePTOcontrols);
  task c(DrivePTOcontrols);
  task d(IntakeControls); // creates a Thread Named "b" that runs the function "IntakeControls", This thread controls the intake
  task e(OutakeControls);

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
  // inertial19.calibrate();
  // while (inertial19.isCalibrating())
  // {
  //   wait(100, msec);
  // }


  // autonSelector.initialize();
  //   while (!Competition.isAutonomous() && !Competition.isDriverControl()) {
  //   autonSelector.update();   // <- keep checking for inputs
  //   wait(20, msec);
}

//////////////////////////////////////////////////////////////////////////
void auton() // A function named "auton", in this case, any code in the brackets will run once (unless in a loop) when its autonomous
{

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
    // Pose currentPose = Odom::getPose();

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
