#include "autonSelector.h"
#include "vex.h"

using namespace vex;

AutonSelector::AutonSelector() {
    currentAutonIndex = 0;
    selecting = true;
    confirmed = false;
}

void AutonSelector::addAuton(const char* name, void (*autonFunction)()) {
    autonOptions.push_back({name, autonFunction});
}

void AutonSelector::displayAutonSelection() {
    Controller1.Screen.clearScreen();
    
    if (!confirmed) {
        // Main selection screen
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("A: Select  B: Back");
        
        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("L1/R1: Navigate");
        
        Controller1.Screen.setCursor(3, 1);
        Controller1.Screen.print("-> %s", autonOptions[currentAutonIndex].name);
    } else {
        // Confirmation screen
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("SELECTED AUTON:");
        
        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("%s", autonOptions[currentAutonIndex].name);
        
        Controller1.Screen.setCursor(3, 1);
        Controller1.Screen.print("A: Confirm  B: Back");
    }
}

void AutonSelector::runSelection() {
    selecting = true;
    confirmed = false;
    
    while (selecting) {
        // Display current selection
        displayAutonSelection();
        
        // Wait for controller input
        while (true) {
            if (!confirmed) {
                // Main selection mode
                if (Controller1.ButtonL1.pressing()) {
                    // Move to previous auton
                    currentAutonIndex--;
                    if (currentAutonIndex < 0) {
                        currentAutonIndex = autonOptions.size() - 1;
                    }
                    displayAutonSelection();
                    wait(200, msec); // Debounce
                }
                else if (Controller1.ButtonR1.pressing()) {
                    // Move to next auton
                    currentAutonIndex++;
                    if (currentAutonIndex >= autonOptions.size()) {
                        currentAutonIndex = 0;
                    }
                    displayAutonSelection();
                    wait(200, msec); // Debounce
                }
                else if (Controller1.ButtonA.pressing()) {
                    // Enter confirmation screen
                    confirmed = true;
                    displayAutonSelection();
                    wait(200, msec);
                    break;
                }
            } else {
                // Confirmation screen mode
                if (Controller1.ButtonA.pressing()) {
                    // Final confirmation - exit selection
                    selecting = false;
                    wait(200, msec);
                    break;
                }
                else if (Controller1.ButtonB.pressing()) {
                    // Go back to selection
                    confirmed = false;
                    displayAutonSelection();
                    wait(200, msec);
                    break;
                }
            }
            
            // Small delay to prevent CPU overload
            wait(20, msec);
        }
    }
    
    // Display final selection
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Auton Ready:");
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("%s", autonOptions[currentAutonIndex].name);
}

void AutonSelector::executeSelectedAuton() {
    if (autonOptions.size() > 0 && currentAutonIndex < autonOptions.size()) {
        autonOptions[currentAutonIndex].autonFunction();
    }
}