#include "autonSelector.h"
#include "motion.h"
#include "vex.h"

using namespace vex;

extern competition Competition;

// Globals
std::string autonRoutine = "none";
int autonIndex = 0;

std::vector<std::string> autonList = {
    "Red Left",
    "Red Right",
    "Blue Left",
    "Blue Right",
    "Skills"};

// Forward declare auton functions (they live in main.cpp)
void redLeftAuton();
void redRightAuton();
void blueLeftAuton();
void blueRightAuton();
void skillsAuton();
void redLeftAuton()
{
    drive(10);
}

//////////////////////////////////////////////////////////////////////////
void redRightAuton()
{
    RollerIntake.setVelocity(600, rpm);
    // Out.setVelocity(200, rpm);
    // Take.setVelocity(200, rpm);

    RollerIntake.spin(forward);
    drive(48.0);
    wait(1.0, sec);
    turn(15);
    drive(20.0);
    wait(0.5, sec);
    drive(-20);
    turn(90);
    drive(40);
    turn(110);
    // put in loader thingy
    drive(10);
    wait(1, sec);
    RollerIntake.stop();
    drive(-55);
    // outake.spin(reverse);
}

//////////////////////////////////////////////////////////////////////////
void blueLeftAuton()
{
    drive(20);
}

//////////////////////////////////////////////////////////////////////////
void blueRightAuton()
{
    turn(20);
}

//////////////////////////////////////////////////////////////////////////
void skillsAuton()
{
    turn(180);
}
// Run auton based on selection
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
    else if (autonRoutine == "Blue Left")
    {
        blueLeftAuton();
    }
    else if (autonRoutine == "Blue Right")
    {
        blueRightAuton();
    }
    else if (autonRoutine == "Skills")
    {
        skillsAuton();
    }
}

// ----------------- UI FUNCTIONS -----------------
void updateBrainScreen()
{
    Brain.Screen.clearScreen();

    // Title
    Brain.Screen.setFont(propXL);
    Brain.Screen.setCursor(1, 9);
    Brain.Screen.setPenColor(white);
    Brain.Screen.print("5069G");

    // Auton name center
    Brain.Screen.setFont(propL);
    Brain.Screen.setCursor(4, 5);
    Brain.Screen.print("Auton: %s", autonList[autonIndex].c_str());

    // Buttons
    Brain.Screen.setFillColor(color(128, 128, 128));
    Brain.Screen.drawRectangle(40, 180, 60, 40);   // Left
    Brain.Screen.drawRectangle(320, 180, 60, 40);  // Right
    Brain.Screen.drawRectangle(170, 180, 100, 40); // Confirm

    Brain.Screen.setFont(monoM);
    Brain.Screen.setCursor(11, 4);
    Brain.Screen.print("<");
    Brain.Screen.setCursor(11, 23);
    Brain.Screen.print(">");
    Brain.Screen.setCursor(10, 7);
    Brain.Screen.print("CONFIRM");
}

void updateControllerScreen()
{
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("5069G");
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print(autonList[autonIndex].c_str());
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("L/R: Change A: Confirm");
}

void showFinalScreen()
{
    Brain.Screen.clearScreen();
    Brain.Screen.setFillColor(black);
    Brain.Screen.setFont(propXXL);

    int screenWidth = 480;
    int w1 = Brain.Screen.getStringWidth("5069G");
    int w2 = Brain.Screen.getStringWidth("ZIPTIDE");

    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt((screenWidth - w1) / 2, 100, "5069G");

    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt((screenWidth - w2) / 2, 200, "ZIPTIDE");
    inertial19.calibrate();
    while (inertial19.isCalibrating())
        wait(100, msec);
    runAuton();
}

// ----------------- AUTON SELECTOR -----------------
void autonSelector()
{
    updateBrainScreen();
    updateControllerScreen();

    bool confirmed = false;
    while (!confirmed)
    {
        // Brain screen press
        if (Brain.Screen.pressing())
        {
            int x = Brain.Screen.xPosition();
            int y = Brain.Screen.yPosition();

            if (y >= 180 && y <= 220)
            {
                if (x >= 40 && x <= 100) // Left button
                {
                    autonIndex = (autonIndex - 1 + autonList.size()) % autonList.size();
                    updateBrainScreen();
                    updateControllerScreen();
                    while (Brain.Screen.pressing())
                        wait(10, msec); // debounce
                }
                else if (x >= 320 && x <= 380) // Right button
                {
                    autonIndex = (autonIndex + 1) % autonList.size();
                    updateBrainScreen();
                    updateControllerScreen();
                    while (Brain.Screen.pressing())
                        wait(10, msec);
                }
                else if (x >= 170 && x <= 270) // Confirm
                {
                    confirmed = true;
                    while (Brain.Screen.pressing())
                        wait(10, msec);
                }
            }
        }

        // Controller input
        if (Controller1.ButtonLeft.pressing())
        {
            autonIndex = (autonIndex - 1 + autonList.size()) % autonList.size();
            updateBrainScreen();
            updateControllerScreen();
            while (Controller1.ButtonLeft.pressing())
                wait(10, msec); // debounce
        }
        else if (Controller1.ButtonRight.pressing())
        {
            autonIndex = (autonIndex + 1) % autonList.size();
            updateBrainScreen();
            updateControllerScreen();
            while (Controller1.ButtonRight.pressing())
                wait(10, msec);
        }
        else if (Controller1.ButtonA.pressing())
        {
            confirmed = true;
            while (Controller1.ButtonA.pressing())
                wait(10, msec);
        }

        wait(10, msec);
    }

    // Final screen + confirm
    showFinalScreen();

    autonRoutine = autonList[autonIndex];
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("Selected: %s", autonRoutine.c_str());
}
