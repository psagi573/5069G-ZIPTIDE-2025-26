#pragma once
#include "vex.h"

using namespace vex;

// Selected auton index and routine string
extern int autonIndex;
extern std::string autonRoutine;

// Starts the auton selector UI on Brain and Controller
void autonSelector();

// Internal helpers (optional to expose)
void updateBrainScreen();
void updateControllerScreen();
void showFinalScreen();