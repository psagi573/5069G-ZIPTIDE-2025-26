#include "vex.h"
#include "autonSelector.h"
#include "Auton.h"
#include "odometry.h"

using namespace vex;

// Global pointer for callback functions
CompetitionAutonSelector* g_autonSelector = nullptr;

// C-style callback functions
void cycleForwardCallback() {
    if (g_autonSelector) {
        g_autonSelector->cycleForward();
    }
}

void cycleBackwardCallback() {
    if (g_autonSelector) {
        g_autonSelector->cycleBackward();
    }
}

CompetitionAutonSelector::CompetitionAutonSelector() {
    selectedMode = CompetitionAutonMode::LEFT_AWP;
    currentIndex = 1;
    competitionMode = false;
    g_autonSelector = this;  // Set global pointer
}

void CompetitionAutonSelector::initialize() {
    Brain.Screen.clearScreen();
    Controller1.Screen.clearScreen();
    
    // Only set up controller callbacks if NOT in competition mode
    if (!competitionMode) {
        Controller1.ButtonR1.pressed(cycleForwardCallback);
        Controller1.ButtonL1.pressed(cycleBackwardCallback);
    }
    
    updateDisplays();
}

void CompetitionAutonSelector::setCompetitionMode(bool isCompetition) {
    competitionMode = isCompetition;
    
    // Update button callbacks based on mode
    if (competitionMode) {
        // In competition mode, disable the callbacks
        Controller1.ButtonR1.pressed(NULL);
        Controller1.ButtonL1.pressed(NULL);
    } else {
        // In practice mode, enable the callbacks
        Controller1.ButtonR1.pressed(cycleForwardCallback);
        Controller1.ButtonL1.pressed(cycleBackwardCallback);
    }
    
    updateDisplays();
}

void CompetitionAutonSelector::updateDisplays() {
    if (competitionMode) {
        updateCompetitionDisplay();
    } else {
        updatePreMatchDisplay();
    }
    updateControllerDisplay();
}

void CompetitionAutonSelector::updatePreMatchDisplay() {
    Brain.Screen.clearScreen();
    
    // Title - Pre-Match Mode
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFont(propL);
    Brain.Screen.printAt(160, 30, "PRE-MATCH SELECTOR");
    Brain.Screen.setFont(propM);
    Brain.Screen.setPenColor(cyan);
    Brain.Screen.printAt(160, 55, "Practice Mode - Use Controller");
    
    // Current selection
    Brain.Screen.setFillColor(modeColors[currentIndex]);
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawRectangle(40, 80, 400, 60);
    
    // Direct string printing - no temporary buffers
    Brain.Screen.setFont(propXL);
    Brain.Screen.printAt(140, 120, modeNames[currentIndex]);
    
    // Instructions for practice mode
    Brain.Screen.setFont(propM);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(50, 160, "L1: Previous  |  R1: Next");
    Brain.Screen.printAt(50, 185, "Selection will be saved for competition");
    
    // All options
    Brain.Screen.setPenColor(yellow);
    Brain.Screen.printAt(50, 220, "Available Options:");
    
    for (int i = 0; i < 7; i++) {
        int yPos = 240 + (i * 18);
        if (i == currentIndex) {
            Brain.Screen.setPenColor(yellow);
            Brain.Screen.printAt(50, yPos, modeNames[i]);
        } else {
            Brain.Screen.setPenColor(white);
            Brain.Screen.printAt(50, yPos, modeNames[i]);
        }
    }
    
    // Competition reminder
    Brain.Screen.setPenColor(green);
    Brain.Screen.printAt(50, 380, "At competition: Set switch to AUTO");
}

void CompetitionAutonSelector::updateCompetitionDisplay() {
    Brain.Screen.clearScreen();
    
    // Title - Competition Mode
    Brain.Screen.setPenColor(white);
    Brain.Screen.setFont(propL);
    Brain.Screen.printAt(160, 30, "COMPETITION MODE");
    Brain.Screen.setFont(propM);
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(160, 55, "Field Controlled - Switch to AUTO");
    
    // Current selection (read-only in competition)
    Brain.Screen.setFillColor(modeColors[currentIndex]);
    Brain.Screen.setPenColor(white);
    Brain.Screen.drawRectangle(40, 80, 400, 60);
    
    // Direct string printing
    Brain.Screen.setFont(propXL);
    Brain.Screen.printAt(140, 120, modeNames[currentIndex]);
    
    // Competition instructions
    Brain.Screen.setFont(propM);
    Brain.Screen.setPenColor(white);
    Brain.Screen.printAt(50, 160, "Selection: %d/%d", currentIndex + 1, 7);
    Brain.Screen.printAt(50, 185, "Chosen during pre-match practice");
    
    // Status
    Brain.Screen.setPenColor(green);
    Brain.Screen.printAt(50, 220, "READY FOR MATCH");
    Brain.Screen.printAt(50, 245, "Field switch controls autonomous start");
    
    // Cannot change reminder
    Brain.Screen.setPenColor(red);
    Brain.Screen.printAt(50, 280, "Cannot change during competition");
    Brain.Screen.printAt(50, 305, "Use pre-match to set selection");
}

void CompetitionAutonSelector::updateControllerDisplay() {
    Controller1.Screen.clearScreen();
    
    if (competitionMode) {
        // Competition mode - minimal display
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("COMP: %s", modeNames[currentIndex]);
        
        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("Field Controlled");
        
        Controller1.Screen.setCursor(3, 1);
        Controller1.Screen.print("Switch to AUTO");
    } else {
        // Practice mode - full controls
        Controller1.Screen.setCursor(1, 1);
        Controller1.Screen.print("PRACT: %s", modeNames[currentIndex]);
        
        Controller1.Screen.setCursor(2, 1);
        Controller1.Screen.print("L1<  R1> to Change");
        
        Controller1.Screen.setCursor(3, 1);
        Controller1.Screen.print("Selection: %d/7", currentIndex + 1);
    }
}

void CompetitionAutonSelector::cycleForward() {
    // Only allow changes in practice mode
    if (!competitionMode) {
        currentIndex = (currentIndex + 1) % 7;
        selectedMode = static_cast<CompetitionAutonMode>(currentIndex);
        Controller1.rumble(".");
        updateDisplays();
    }
}

void CompetitionAutonSelector::cycleBackward() {
    // Only allow changes in practice mode
    if (!competitionMode) {
        currentIndex = (currentIndex - 1 + 7) % 7;
        selectedMode = static_cast<CompetitionAutonMode>(currentIndex);
        Controller1.rumble(".");
        updateDisplays();
    }
}

CompetitionAutonMode CompetitionAutonSelector::getSelectedMode() {
    return selectedMode;
}

const char* CompetitionAutonSelector::getModeName() {
    return modeNames[currentIndex];
}

void CompetitionAutonSelector::update() {
  // Controller buttons
  if (Controller1.ButtonR1.pressing()) {
    cycleForward();
    wait(250, msec); // debounce
  }

  if (Controller1.ButtonL1.pressing()) {
    cycleBackward();
    wait(250, msec); // debounce
  }
}

void CompetitionAutonSelector::runSelectedAuton() {
    // Display running message
    Brain.Screen.clearScreen();
    Brain.Screen.setPenColor(green);
    Brain.Screen.setFont(propXL);
    Brain.Screen.printAt(120, 100, "RUNNING");
    Brain.Screen.setFont(propL);
    Brain.Screen.printAt(120, 140, getModeName());
    
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("RUN: %s", getModeName());
    
    wait(500, msec);
    
    // Run autonomous based on selection
    switch (selectedMode) {
        case CompetitionAutonMode::SKILLS:
            startOdom(Xaxis, Yaxis, inertial19);
            autonomous_skills();
            break;
        case CompetitionAutonMode::LEFT_AWP:
            startOdom(Xaxis, Yaxis, inertial19);
            autonomous_left_awp();
            break;
        case CompetitionAutonMode::RIGHT_AWP:
            startOdom(Xaxis, Yaxis, inertial19);
            autonomous_right_awp();
            break;
        case CompetitionAutonMode::LEFT_ELIM:
            startOdom(Xaxis, Yaxis, inertial19);    
            autonomous_left_elim();
            break;
        case CompetitionAutonMode::RIGHT_ELIM:
            startOdom(Xaxis, Yaxis, inertial19);
            autonomous_right_elim();
            break;
        case CompetitionAutonMode::SAFE:
            startOdom(Xaxis, Yaxis, inertial19);
            autonomous_safe();
            break;
        case CompetitionAutonMode::DISABLED:
            // No autonomous - just wait out the time
            wait(15, sec);
            break;
    }
}