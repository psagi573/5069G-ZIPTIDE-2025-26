#include "autonSelector.h"
#include "vex.h"

using namespace vex;

extern competition Competition;

int selectedAuton = 0;
const int totalAutons = 3;

std::string autonNames[] = {
    "Left Side Win Point",
    "Right Side Rush",
    "Skills Routine"};

void drawSelector()
{
    Brain.Screen.clearScreen();
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFillColor(black);
    Brain.Screen.print("Select Autonomous:");

    // Draw left arrow box
    Brain.Screen.setFillColor(blue);
    Brain.Screen.drawRectangle(10, 100, 60, 60);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(30, 140, "<");

    // Draw right arrow box
    Brain.Screen.setFillColor(blue);
    Brain.Screen.drawRectangle(250, 100, 60, 60);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(275, 140, ">");

    // Display auton name in middle box
    Brain.Screen.setFillColor(black);
    Brain.Screen.drawRectangle(80, 100, 160, 60);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(90, 140, "%d: %s", selectedAuton + 1, autonNames[selectedAuton].c_str());
}

void autonSelectorUI()
{
    drawSelector();

    while (!Competition.isAutonomous() && !Competition.isDriverControl())
    {
        if (Brain.Screen.pressing())
        {
            int x = Brain.Screen.xPosition();
            int y = Brain.Screen.yPosition();

            if (y >= 100 && y <= 160)
            {
                if (x >= 10 && x <= 70)
                {
                    selectedAuton = (selectedAuton - 1 + totalAutons) % totalAutons;
                }
                else if (x >= 250 && x <= 310)
                {
                    selectedAuton = (selectedAuton + 1) % totalAutons;
                }

                drawSelector();
                wait(300, msec); // Debounce delay
                while (Brain.Screen.pressing())
                {
                    wait(10, msec); // Wait for release
                }
            }
        }

        wait(10, msec);
    }
}
