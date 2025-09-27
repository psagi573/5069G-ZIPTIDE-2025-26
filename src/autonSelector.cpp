#include "autonSelector.h"
#include "vex.h"

using namespace vex;

// Add this extern declaration to access the Competition object from main.cpp
extern competition Competition;

std::vector<AutonSelector::AutonOption> autonOptions;

AutonSelector::AutonSelector() {
    currentAutonIndex = 0;
    selecting = false;
    confirmed = false;
    selectionTask = nullptr;
}

void AutonSelector::addAuton(const char* name, void (*autonFunction)()) {
    autonOptions.push_back({name, autonFunction});
}

void AutonSelector::updateSelection() {
    if (!selecting || Competition.isEnabled()) {
        stopSelection();
        return;
    }
    
    Controller1.Screen.clearScreen();
    
    if (!confirmed) {
        // Main selection screen
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("A: Select  B: Back");
        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("L1/R1: Navigate");
        Controller1.Screen.setCursor(3, 1);
        Controller1.Screen.print("-> %s", autonOptions[currentAutonIndex].name);
        
        // Handle navigation
        if (Controller1.ButtonL1.pressing()) {
            currentAutonIndex--;
            if (currentAutonIndex < 0) {
                currentAutonIndex = autonOptions.size() - 1;
            }
            wait(200, msec); // Debounce
        }
        else if (Controller1.ButtonR1.pressing()) {
            currentAutonIndex++;
            if (currentAutonIndex >= autonOptions.size()) {
                currentAutonIndex = 0;
            }
            wait(200, msec); // Debounce
        }
        else if (Controller1.ButtonA.pressing()) {
            confirmed = true;
            wait(200, msec);
        }
    } else {
        // Confirmation screen
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("SELECTED AUTON:");
        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("%s", autonOptions[currentAutonIndex].name);
        Controller1.Screen.setCursor(3, 1);
        Controller1.Screen.print("A: Confirm  B: Back");
        
        if (Controller1.ButtonA.pressing()) {
            selecting = false;
            Controller1.Screen.clearScreen();
            Controller1.Screen.setCursor(1, 1);
            Controller1.Screen.print("Auton Ready:");
            Controller1.Screen.setCursor(2, 1);
            Controller1.Screen.print("%s", autonOptions[currentAutonIndex].name);
        }
        else if (Controller1.ButtonB.pressing()) {
            confirmed = false;
            wait(200, msec);
        }
    }
}

int AutonSelector::selectionTaskFunc(void* selector) {
    AutonSelector* autonSelector = (AutonSelector*)selector;
    
    while (autonSelector->isSelecting() && !Competition.isEnabled()) {
        autonSelector->updateSelection();
        wait(50, msec);
    }
    
    return 0;
}

void AutonSelector::startSelection() {
    if (selecting || Competition.isEnabled()) return;
    
    selecting = true;
    confirmed = false;
    
    if (selectionTask) {
        selectionTask->stop();
        delete selectionTask;
    }
    
    selectionTask = new task(selectionTaskFunc, this);
}

void AutonSelector::stopSelection() {
    selecting = false;
    if (selectionTask) {
        selectionTask->stop();
        delete selectionTask;
        selectionTask = nullptr;
    }
}

void AutonSelector::executeSelectedAuton() {
    if (autonOptions.size() > 0 && currentAutonIndex < autonOptions.size()) {
        autonOptions[currentAutonIndex].autonFunction();
    }
}