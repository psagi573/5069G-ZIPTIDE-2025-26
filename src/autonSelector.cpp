#include "vex.h"
#include "odom.h"
#include "autonselector.h"
#include "autons.h"
#include <string>
#include <cmath>

using namespace vex;

// -------------------------
// Configuration
// -------------------------
rotation *xRot;
rotation *yRot;
inertial *imuSensor;

// Replace these with your converted .c images
extern const char *pathRightImage;
extern const char *pathLeftImage;
extern const char *ziptideLogo;
std::string autonRoutine;
// -------------------------
// Auton Definition
// -------------------------


Auton autons[] = {
    {"Red Left", 20, 10.0, 10.0, 10.0},
    {"Red Right", 15, 20.0, 20.0, 20.0},
    // Add more autons here
};
const int AUTON_COUNT = sizeof(autons) / sizeof(Auton);
int selectedAuton = 0;


void runAuton()
{
    if (autonRoutine == "Red Left")
    {
        redLeftAuton();
    }
    else if (autonRoutine == "Red Right")
    {
        redRightAuton();
    }
}
// -------------------------
// Selector States
// -------------------------
enum SelectorState
{
    AUTON_SELECT,
    CALIBRATE,
    CALIBRATING,
    READY
};
SelectorState state = AUTON_SELECT;

// -------------------------
// Display Functions
// -------------------------
void displayAutonSelection()
{
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(vex::mono20);
    Brain.Screen.printAt(10, 20, autons[selectedAuton].name.c_str());
    Brain.Screen.printAt(10, 50, "Points: %d", autons[selectedAuton].points);
    // Brain.Screen.drawImage(10, 80, autons[selectedAuton].pathImage);

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print(autons[selectedAuton].name.c_str());
}

void displayCalibrationScreen()
{
    Brain.Screen.setFillColor(green);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(vex::mono20);
    Brain.Screen.printAt(20, 50, "CALIBRATE");

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("CALIBRATE");
}

void displayCalibrating()
{
    Brain.Screen.clearScreen();
    Brain.Screen.setFillColor(green);
    Brain.Screen.printAt(20, 50, "CALIBRATING...");

    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("CALIBRATING...");
}


void displayReady()
{
    Brain.Screen.clearScreen();
    Controller1.Screen.clearScreen();
}



// -------------------------
// Calibration
// -------------------------
void startCalibration()
{
    state = CALIBRATING;
    displayCalibrating();

    // Calibrate IMU
    inertial19.calibrate();
    while (inertial19.isCalibrating())
        wait(50, msec);

    // Start odometry at selected auton start position

    // Rumble controller
    Controller1.rumble("..");

    state = READY;
}

// -------------------------
// Selector Loop
// -------------------------
void autonSelectorLoop()
{
    displayAutonSelection();

    while (true)
    {
        // Navigate autons
        if (Controller1.ButtonLeft.pressing() && state == AUTON_SELECT)
        {
            selectedAuton = (selectedAuton - 1 + AUTON_COUNT) % AUTON_COUNT;
            displayAutonSelection();
            wait(200, msec);
        }
        if (Controller1.ButtonRight.pressing() && state == AUTON_SELECT)
        {
            selectedAuton = (selectedAuton + 1) % AUTON_COUNT;
            displayAutonSelection();
            wait(200, msec);
        }

        // Confirm / Calibrate
        if (Controller1.ButtonA.pressing())
        {
            if (state == AUTON_SELECT)
            {
                state = CALIBRATE;
                displayCalibrationScreen();
            }
            else if (state == CALIBRATE)
            {
                startCalibration();
            }
        }

        // Cancel calibration back to selection
        if (Controller1.ButtonB.pressing() && state == CALIBRATE)
        {
            state = AUTON_SELECT;
            displayAutonSelection();
        }

        // Update live pose if ready
        if (state == READY)
        {
            displayReady();
            autonRoutine = autons[selectedAuton].name;
        }

        wait(50, msec);
    }
}

// -------------------------
// Initialization
// -------------------------
// void PRSPro(rotation &xSensor, rotation &ySensor, inertial &imu)
// {
//     xRot = &xSensor;
//     yRot = &ySensor;
//     imuSensor = &imu;

//     startOdomAt(*xRot, *yRot, *imuSensor,
//                 autons[selectedAuton].startX,
//                 autons[selectedAuton].startY,
//                 autons[selectedAuton].startTheta);

//     // Start the selector loop
//     autonSelectorLoop();
// }
