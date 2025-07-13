#include "autonSelector.h"
#include "vex.h"
using namespace vex; // Ensure VEX objects are in scope

competition Competition; // Define the Competition object

int selectedAuton = 0; // This is the actual definition

const int totalAutons = 3;

std::string autonNames[] = {
    "Left Side Win Point",
    "Right Side Rush",
    "Skills Routine"};

void displayAuton()
{
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("Auton Selected:");
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("%d: %s", selectedAuton + 1, autonNames[selectedAuton].c_str());
}

void autonSelectorUI()
{
    displayAuton();
    Brain.Screen.print("\n\nTouch left/right to choose.");

    while (!Brain.Screen.pressing())
        wait(10, msec); // Wait for initial press

    while (true)
    {
        if (Brain.Screen.pressing())
        {
            int x = Brain.Screen.xPosition();
            if (x < 120)
            {
                selectedAuton = (selectedAuton - 1 + totalAutons) % totalAutons;
            }
            else
            {
                selectedAuton = (selectedAuton + 1) % totalAutons;
            }

            displayAuton();
            wait(300, msec); // debounce
            while (Brain.Screen.pressing())
                wait(10, msec); // wait for release
        }

        // Exit if competition mode starts
        if (Competition.isAutonomous() || Competition.isDriverControl())
            break;
    }
}