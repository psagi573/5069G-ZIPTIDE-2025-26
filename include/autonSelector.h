#pragma once
#include "vex.h"
#include <string>
#include <vector>

// Globals
extern std::string autonRoutine;
extern int autonIndex;
extern std::vector<std::string> autonList;

extern rotation *xRot;
extern rotation *yRot;
extern inertial *imuSensor;

// Selector
void autonSelector();

// Run the chosen auton
void runAuton();
void PRSPro(rotation &xSensor, rotation &ySensor, inertial &imu);
