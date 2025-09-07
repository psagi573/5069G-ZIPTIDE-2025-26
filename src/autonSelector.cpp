#include "vex.h"
#include "odom.h"
#include "autonselector.h"
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

// -------------------------
// Auton Definition
// -------------------------
struct Auton
{
    std::string name;
    int points;
    const char *pathImage;
    double startX;
    double startY;
    double startTheta;
};

Auton autons[] = {
    {"Right 20pt", 20, pathRightImage, 0.0, 0.0, 0.0},
    {"Left 15pt", 15, pathLeftImage, 0.0, 0.0, 0.0},
    // Add more autons here
};
const int AUTON_COUNT = sizeof(autons) / sizeof(Auton);
int selectedAuton = 0;

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
    Controller1.Screen.print(autons[selectedAuton].name.c_str());
}

void displayCalibrationScreen()
{
    Brain.Screen.setFillColor(green);
    Brain.Screen.clearScreen();
    Brain.Screen.setFont(vex::mono20);
    Brain.Screen.printAt(20, 50, "CALIBRATE");

    Controller1.Screen.clearScreen();
    Controller1.Screen.print("CALIBRATE");
}

void displayCalibrating()
{
    Brain.Screen.clearScreen();
    Brain.Screen.setFillColor(green);
    Brain.Screen.printAt(20, 50, "CALIBRATING...");

    Controller1.Screen.clearScreen();
    Controller1.Screen.print("CALIBRATING...");
}

void displayReady()
{
    Brain.Screen.clearScreen();
    // Brain.Screen.drawImage(10, 80, ziptideLogo);
    Brain.Screen.setFont(vex::mono20);
    Brain.Screen.setPenColor(white);

    thread readyTask(readyScreenTask);
}

int readyScreenTask()
{
    while (state == READY)
    {
        Pose p = getPose();

        // Only update the numbers
        Brain.Screen.setFont(mono20);
        Brain.Screen.setPenColor(white);
        Brain.Screen.printAt(10, 10, "X: %.2f  ", p.x); // extra spaces to overwrite old numbers
        Brain.Screen.printAt(10, 30, "Y: %.2f  ", p.y);
        Brain.Screen.printAt(10, 50, "Theta: %.2f  ", p.theta);

        Controller1.Screen.clearLine();
        Controller1.Screen.print("X: %.2f\nY: %.2f\nTheta: %.2f", p.x, p.y, p.theta);

        wait(50, msec);
    }
    return 0;
}

// -------------------------
// Calibration
// -------------------------
void startCalibration()
{
    state = CALIBRATING;
    displayCalibrating();

    // Calibrate IMU
    imuSensor->calibrate();
    while (imuSensor->isCalibrating())
        wait(50, msec);

    // Start odometry at selected auton start position
    startOdomAt(*xRot, *yRot, *imuSensor,
                autons[selectedAuton].startX,
                autons[selectedAuton].startY,
                autons[selectedAuton].startTheta);

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
        }

        wait(50, msec);
    }
}

// -------------------------
// Initialization
// -------------------------
void initPRSPro(rotation &xSensor, rotation &ySensor, inertial &imu)
{
    xRot = &xSensor;
    yRot = &ySensor;
    imuSensor = &imu;

    // Start the selector loop
    autonSelectorLoop();
}
