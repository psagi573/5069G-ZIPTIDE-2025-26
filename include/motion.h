#pragma once
#include "odometry.h"

// existing functions
void drive(double distInches, double timeout);
void turn(double targetHeading);
void arc(double radiusInches, double angleDeg);
void Sweep(double targetAngleDeg, bool left);

// ---- New Boomerang motion functions ----
// Blocking: returns true if reached target before timeout, false on timeout.
bool moveToPointBoomerang(double tx, double ty, int timeoutMs = 3000);
bool moveToPoseBoomerang(double tx, double ty, double thetaDeg, int timeoutMs = 4500);
