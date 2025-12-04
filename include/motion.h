#pragma once
#include "odometry.h"

// existing functions
void drive(double distInches, double timeout);
void turn(double targetHeading);
void arc(double radiusInches, double angleDeg);
void Sweep(double targetAngleDeg, bool left);

