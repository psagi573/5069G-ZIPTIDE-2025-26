#include "autonSelector.h"
#include "vex.h"

using namespace vex;

extern competition Competition;

std::string autonRoutine = "none";
int autonIndex = 0;

std::vector<std::string> autonList = {
    "Red Left",
    "Red Right",
    "Blue Left",
    "Blue Right",
    "Skills"};

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
    Brain.Screen.setPenColor(color(128, 128, 128));
    Brain.Screen.setFont(propXXL);

    int screenWidth = 480;
    int w1 = Brain.Screen.getStringWidth("5069G");
    int w2 = Brain.Screen.getStringWidth("ZIPTIDE");

    Brain.Screen.printAt((screenWidth - w1) / 2, 100, "5069G");
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt((screenWidth - w2) / 2, 200, "ZIPTIDE");
}

void autonSelector()
{
    updateBrainScreen();
    updateControllerScreen();

    bool confirmed = false;
    while (!confirmed)
    {
        if (Brain.Screen.pressing())
        {
            int x = Brain.Screen.xPosition();
            int y = Brain.Screen.yPosition();

            if (y >= 180 && y <= 220)
            {
                if (x >= 40 && x <= 100)
                {
                    autonIndex = (autonIndex - 1 + autonList.size()) % autonList.size();
                }
                else if (x >= 320 && x <= 380)
                {
                    autonIndex = (autonIndex + 1) % autonList.size();
                }
                else if (x >= 170 && x <= 270)
                {
                    confirmed = true;
                    break;
                }
                updateBrainScreen();
                updateControllerScreen();
                wait(250, msec);
            }
        }

        if (Controller1.ButtonLeft.pressing())
        {
            autonIndex = (autonIndex - 1 + autonList.size()) % autonList.size();
            updateBrainScreen();
            updateControllerScreen();
            wait(300, msec);
        }
        else if (Controller1.ButtonRight.pressing())
        {
            autonIndex = (autonIndex + 1) % autonList.size();
            updateBrainScreen();
            updateControllerScreen();
            wait(300, msec);
        }
        else if (Controller1.ButtonA.pressing())
        {
            confirmed = true;
            break;
        }

        wait(10, msec);
    }

    showFinalScreen();

    autonRoutine = autonList[autonIndex];
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("Selected: %s", autonRoutine.c_str());
}
