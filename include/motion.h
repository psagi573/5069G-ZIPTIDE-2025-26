#include "odom.h"

void drive(double distInches, double headingDeg = 0);
void turn(double targetHeading);
void arc(double radiusInches, double angleDeg);
void sweep(double power, double angleDeg);