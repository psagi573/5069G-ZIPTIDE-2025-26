#pragma once
#include "vex.h"
#include <string>
#include <vector>

// Globals
extern std::string autonRoutine;
extern int autonIndex;
extern std::vector<std::string> autonList;

// Selector
void autonSelector();

// Run the chosen auton
void runAuton();
void initPRSPro(rotation &xSensor, rotation &ySensor, inertial &imu);
