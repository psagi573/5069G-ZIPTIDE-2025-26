#pragma once
#include "odometry.h"
void drive(double distInches, double timeout);
void turn(double targetHeading);
void arc(double radiusInches, double angleDeg);
void Sweep(double targetAngleDeg, bool left);
void driveTo(double targetX, double targetY);
void moveTo(double targetX, double targetY, double finalHeading, double maxVelocity, double acceleration);
void moveTo(double targetX, double targetY, double maxVelocity, double acceleration);
void moveTo(double targetX, double targetY);