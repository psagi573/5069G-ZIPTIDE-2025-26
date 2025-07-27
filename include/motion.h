#include "odom.h"
void longdrive(double distInches, double headingDeg);
void fastdrive(double distInches);
void turn(double targetHeading);
void sturn(double stargetHeading);
void arc(double radiusInches, double angleDeg);
void moveTo(double targetX, double targetY, double targetTheta, bool turnAtEnd);
// oid sweep(double power, double angleDeg);