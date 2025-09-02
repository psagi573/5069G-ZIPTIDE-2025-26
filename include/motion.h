#pragma once
#include "odom.h"
void drive(double distInches);
void turn(double targetHeading);
void arc(double radiusInches, double angleDeg, bool forward, bool right);
void Sweep(double targetAngleDeg, bool left, bool forward);