#pragma once
#include "vex.h"
#include <string>
#include <vector>

// Globals
extern std::string autonRoutine;
extern int autonIndex;
extern std::vector<std::string> autonList;


struct Auton {
    std::string name;
    int points;
    double startX;
    double startY;
    double startTheta;
};

extern Auton autons[];
extern int selectedAuton;


// Selector
void autonSelector();

// Run the chosen auton
void runAuton();
void autonSelectorLoop();
