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
#include "autons.h"
#include <string>

// Ensure 'autons' and 'selectedAuton' are declared as extern if defined elsewhere

using namespace vex; // you need it so vex stuff works
motor_group R = motor_group(R6, R7, R8);
motor_group L = motor_group(L1, L2, L3);
drivetrain Drivetrain = drivetrain(R, L);
motor_group outake = motor_group(Out, Take);
motor_group scorer = motor_group(Out, Take, RollerIntake);

competition Competition; // you need it so it works at a competition

float tovolt(float percentage)
{
  return (percentage * 12.0 / 100.0);
}

const int JOYSTICK_DEADZONE = 5; // tweak this if needed

int getExpoValue(int joystickValue)
{
  int output = 0;

  if (abs(joystickValue) > JOYSTICK_DEADZONE)
  {
    int direction = (joystickValue > 0) ? 1 : -1; // forward or backward

    // Exponential curve + linear tweak
    output = direction * (1.2 * pow(1.05, abs(joystickValue)) - 1.2 + 0.2 * abs(joystickValue));

    // Clamp output to -100 to 100
    if (output > 100)
      output = 100;
    if (output < -100)
      output = -100;
  }

  return output;
}

int DriveTrainControls() // we create a integer function named "DriveTrainControls", later in the code we plan to turnpid this into a Thread that controls the drivetrain
{
  // IN ORDER FOR THIS TO WORK, ALL MOTORS ON THE RIGHT SIDE OF THE DRIVETRAIN MUST BE SET TO "REVERSE"
  L1.setStopping(brake);
  L2.setStopping(brake);
  L3.setStopping(brake);
  R6.setStopping(brake);
  R7.setStopping(brake);
  R8.setStopping(brake);
  outake.setStopping(brake);
  RollerIntake.setStopping(brake);

  L1.setVelocity(600, rpm);
  L2.setVelocity(600, rpm);
  L3.setVelocity(600, rpm);
  R6.setVelocity(600, rpm);
  R7.setVelocity(600, rpm);
  R8.setVelocity(600, rpm);
  outake.setVelocity(200, rpm);
  RollerIntake.setVelocity(600, rpm);
  // Makes the motors set to "coast" when they arent being used aka the joystick isnt being moved

  while (true)
  {
    // Read joystick values
    int four = Controller1.Axis3.position(percent);
    int tur = Controller1.Axis1.position(percent);
    // int ture = Controller1.Axis1.position(percent);
    // // Determine left and right motor voltages
    int leftVolt = tovolt(four + tur);
    int rightVolt = tovolt(four - tur);

    // // Detect turning in place
    // brakeType driveBrake;

    // if (abs(four) < 5 && abs(tur) > 10)
    // {
    //   driveBrake = brakeType::brake;
    // }
    // else
    // {
    //   driveBrake = brakeType::coast;
    // }

    // // Apply brake mode to all drivetrain motors
    // L.setStopping(driveBrake);
    // R.setStopping(driveBrake);

    // // Spin motors
    L.spin(forward, leftVolt, volt);
    R.spin(forward, rightVolt, volt);
    wait(10, msec);
  }
}

int SystemControls()
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

int OutakeControls()
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

int IntakeControls()
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

int Intake2Controls()
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

int LoaderControls()
{
  bool Loader1 = false;
  while (true)
  {
    if (Controller1.ButtonDown.pressing())
    {
      if (Loader1)
      {
        Loader1 = false;
      }
      else if (!Loader1)
      {
        Loader1 = true;
      }
      while (Controller1.ButtonDown.pressing())
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

int Colorsortcontrols()
{

  bool Trapdoor1 = false;
  while (true)
  {
    if (Controller1.ButtonY.pressing())
    {
      if (Trapdoor1)
      {
        Trapdoor1 = false;
      }
      else if (!Trapdoor1)
      {
        Trapdoor1 = true;
      }
      while (Controller1.ButtonY.pressing())
      {

        wait(5, msec);
      }

      if (Trapdoor1)
      {
        Trapdoor.set(true);
      }
      else
      {
        Trapdoor.set(false);
      }
    }
  }
}
task colorsort;
void usercontrol() // A function named "usercontrol", in this case, any code in the brackets will run once (unless in a loop) when its driver control
{
  colorsort.stop();
  task a(DriveTrainControls); // creates a Thread Named "a" that runs the function "DriveTrainControls", This thread controls the drivetrain
  task b(SystemControls);     // same as drivetrain controls but for the lifter
  task c(LifterControls);
  task d(OutakeControls);
  task e(IntakeControls);
  task f(Intake2Controls);
  task g(LoaderControls);
  task h(Colorsortcontrols);
}

int autonSelected = 0;
bool autonConfirmed = false;

// List of auton names
const char *autonNames[4] = {
    "Left Rush",
    "Right Rush",
    "Skills",
    "Safe Mid"};

void updateScreens()
{
  // Clear and redraw Brain
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  if (!autonConfirmed)
  {
    Brain.Screen.print("Select Auton: %s", autonNames[autonSelected]);
  }
  else
  {
    Brain.Screen.print("Confirmed: %s", autonNames[autonSelected]);
  }

  // Clear and redraw Controller
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(0, 0);
  if (!autonConfirmed)
  {
    Controller1.Screen.print("Auton: %s", autonNames[autonSelected]);
  }
  else
  {
    Controller1.Screen.print("Confirmed: %s", autonNames[autonSelected]);
  }
}

void autonSelector()
{
  // Initial draw
  updateScreens();

  while (!autonConfirmed)
  {
    if (Controller1.ButtonR1.pressing())
    {
      autonSelected = (autonSelected + 1) % 4; // cycle forward
      updateScreens();
      wait(300, msec); // debounce
    }
    if (Controller1.ButtonL1.pressing())
    {
      autonSelected = (autonSelected - 1 + 4) % 4; // cycle backward
      updateScreens();
      wait(300, msec); // debounce
    }
    if (Controller1.ButtonA.pressing())
    {
      // First press shows confirm message
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(1, 1);
      Brain.Screen.print("Confirm: %s", autonNames[autonSelected]);

      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(0, 0);
      Controller1.Screen.print("Confirm: %s", autonNames[autonSelected]);

      // Wait for release
      while (Controller1.ButtonA.pressing())
      {
        wait(20, msec);
      }
      // Wait for second press
      while (!Controller1.ButtonA.pressing())
      {
        wait(20, msec);
      }
      while (Controller1.ButtonA.pressing())
      {
        wait(20, msec);
      }

      autonConfirmed = true;
      Brain.Screen.clearScreen(); // clear after confirm
      Controller1.Screen.clearScreen();
      break;
    }
    wait(20, msec);
  }
}

void pre_auton(void)
{
  vexcodeInit();
  // autonSelector();
  inertial19.calibrate();
  while (inertial19.isCalibrating())
  {
    wait(50, msec);
  }
  // autonSelectorLoop();
}

bool Trap = false;                        // whether to enable sorting
vex::color targetColor = vex::color::red; // set target color

int Colorcontrols()
{
  Color.setLightPower(100, percent);

  while (true)
  {
    if (!Trap)
    {
      Trapdoor.set(false); // keep closed if Trap disabled
      continue;
    }

    // Only check if object is nearby
    if (Color.isNearObject())
    {
      // Check if the color matches
      if (Color.color() == targetColor)
      {
        Trapdoor.set(true);  // open trapdoor
        task::sleep(1000);   // wait 1000 milliseconds
        Trapdoor.set(false); // close trapdoor
      }
      else
      {
        Trapdoor.set(false); // keep it closed
      }
    }
    else
    {
      Trapdoor.set(false); // nothing near -> close
    }

    wait(20, msec); // small delay to avoid CPU hog
  }
  return 0;
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
  outake.setStopping(coast);
  RollerIntake.setStopping(coast);

  timer autonTimer;
  autonTimer.reset();

  colorsort = task(Colorcontrols);
  targetColor = vex::color::blue;
  Trap = true;
  L1.setVelocity(600, rpm);
  L2.setVelocity(600, rpm);
  L3.setVelocity(600, rpm);
  R6.setVelocity(600, rpm);
  R7.setVelocity(600, rpm);
  R8.setVelocity(600, rpm);
  Out.setVelocity(200, rpm);
  Take.setVelocity(200, rpm);
  outake.setVelocity(200, rpm);
  RollerIntake.setVelocity(600, rpm);

  ///////////////////////////////////////////////////////////////////////
  ////LEFT QUALS AUTON//////////

  // RollerIntake.spin(forward);
  // drive(27);
  // //drive(8);
  // wait(0.2, sec);
  // turn(245);
  // wait(0.2, sec);
  // drive(-12);
  // outake.spin(forward);
  // wait(1.3, sec);
  // outake.stop();
  // drive(47);
  // turn(200);
  // Loader.set(true);
  // Lifter.set(true);
  // wait(0.2, sec);
  // drive(13);
  // wait(0.2, sec);
  // drive(-30);
  // outake.spin(forward);

  //////////////////////////////////////////////////////////////////

  // good elims left auton
  Brain.Timer.reset();
  RollerIntake.spin(forward);
  drive(19);
  drive(7.5);
  turn(240);
  drive(37.5);
  Loader.set(true);
  Lifter.set(true);
  wait(0.3, sec);
  turn(195);
  drive(13);
  drive(-5);
  turn(205);
  drive(-25);
  outake.spin(forward);

  double elapsedTime = autonTimer.time(timeUnits::msec);
  Brain.Screen.setCursor(8, 1);
  Brain.Screen.print("Auton Time: %.2f seconds", elapsedTime);
  /////////////////////////////////////////////////////////////////////////
  // Brain.Screen.setCursor(7, 1);
  // Brain.Screen.print("Time: %f", Brain.Timer.time(sec));

  //   switch (autonSelected) {
  //     case 0:
  //       outake.setStopping(coast);
  //       RollerIntake.setStopping(coast);

  //       colorsort = task(Colorcontrols);
  //       targetColor = vex::color::red;
  //       Trap = true;
  //       L1.setVelocity(600, rpm);
  //       L2.setVelocity(600, rpm);
  //       L3.setVelocity(600, rpm);
  //       R6.setVelocity(600, rpm);
  //       R7.setVelocity(600, rpm);
  //       R8.setVelocity(600, rpm);
  //       Out.setVelocity(200, rpm);
  //       Take.setVelocity(200, rpm);
  //       outake.setVelocity(200, rpm);
  //       RollerIntake.setVelocity(600, rpm);

  //       RollerIntake.spin(forward);

  //       break;
  // //////////////////////////////////////////////////////////

  //     case 1:
  //             outake.setStopping(coast);
  //       RollerIntake.setStopping(coast);

  //       colorsort = task(Colorcontrols);
  //       targetColor = vex::color::red;
  //       Trap = true;
  //       L1.setVelocity(600, rpm);
  //       L2.setVelocity(600, rpm);
  //       L3.setVelocity(600, rpm);
  //       R6.setVelocity(600, rpm);
  //       R7.setVelocity(600, rpm);
  //       R8.setVelocity(600, rpm);
  //       Out.setVelocity(200, rpm);
  //       Take.setVelocity(200, rpm);
  //       outake.setVelocity(200, rpm);
  //       RollerIntake.setVelocity(600, rpm);

  //       RollerIntake.spin(reverse);

  //       break;
  // ///////////////////////////////////////////////////////////

  //     case 2:
  //       outake.setStopping(coast);
  //       RollerIntake.setStopping(coast);

  //       colorsort = task(Colorcontrols);
  //       targetColor = vex::color::red;
  //       Trap = true;
  //       L1.setVelocity(600, rpm);
  //       L2.setVelocity(600, rpm);
  //       L3.setVelocity(600, rpm);
  //       R6.setVelocity(600, rpm);
  //       R7.setVelocity(600, rpm);
  //       R8.setVelocity(600, rpm);
  //       Out.setVelocity(200, rpm);
  //       Take.setVelocity(200, rpm);
  //       outake.setVelocity(200, rpm);
  //       RollerIntake.setVelocity(600, rpm);

  //       outake.spin(forward);

  //       break;

  // //////////////////////////////////////////////////////
  //     case 3:
  //             outake.setStopping(coast);
  //       RollerIntake.setStopping(coast);

  //       colorsort = task(Colorcontrols);
  //       targetColor = vex::color::red;
  //       Trap = true;
  //       L1.setVelocity(600, rpm);
  //       L2.setVelocity(600, rpm);
  //       L3.setVelocity(600, rpm);
  //       R6.setVelocity(600, rpm);
  //       R7.setVelocity(600, rpm);
  //       R8.setVelocity(600, rpm);
  //       Out.setVelocity(200, rpm);
  //       Take.setVelocity(200, rpm);
  //       outake.setVelocity(200, rpm);
  //       RollerIntake.setVelocity(600, rpm);

  //       outake.spin(reverse);

  //       break;
  //   }

  //////////////right quals auton//////////
  // RollerIntake.spin(forward);
  // drive(18.5);
  // drive(9);
  // turn(243);
  // drive(-15);
  // outake.spin(forward);
  // wait(0.8, sec);
  // outake.stop();
  // drive(50);

  // turn(195);
  // Loader.set(true);
  // Lifter.set(true);
  // wait(0.2, sec);
  // drive(13);
  // wait(0.7, sec);
  // Trap = true;
  // wait(0.3, sec);
  // drive(-10);
  // turn(203);
  // drive(-22);
  // outake.spin(forward);

  //////left side elims auton//////

  // RollerIntake.spin(forward);
  // drive(18.5);
  // drive(8);
  // turn(240);
  // drive(39.5);
  // turn(195);
  // Loader.set(true);
  // Lifter.set(true);
  // wait(0.2, sec);
  // drive(13);
  // wait(0.7, sec);
  // Trap = true;
  // wait(0.3, sec);
  // drive(-10);
  // turn(203);
  // drive(-22);
  // outake.spin(forward);
}

int main()
{

  vexcodeInit();
  pre_auton();
  Competition.autonomous(auton);          // what function to run when autonomous begins, in this case it would run the function "auton"
  Competition.drivercontrol(usercontrol); // what function to run when driver control begins, in this case it would run the function "usercontrol"
  startOdom(Xaxis, Yaxis, inertial19);

  // startOdomAt(Xaxis, Yaxis, inertial19,
  //             autons[selectedAuton].startX,
  //             autons[selectedAuton].startY,
  //             autons[selectedAuton].startTheta);
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
    Brain.Screen.print("Y inch: %.2f", yEnc * 2);
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
    Controller1.Screen.print("T: %.2f", currentPose.theta);
    printf("(%.2f, %.2f),", currentPose.x, currentPose.y);
    fflush(stdout);

    wait(50, msec); // Delay to avoid screen spam
  }
}
