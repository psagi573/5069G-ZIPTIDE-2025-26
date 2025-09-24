#include "autons.h"
#include "motion.h"
#include "odom.h"

void redLeftAuton()
{
    // Example autonomous routine for "Red Left"
    drive(24,1000);   // Drive forward 24 inches
    turn(90);  // Turn to 90 degrees
    drive(40,1000);   // Drive forward 40 inches
    // Add more actions as needed
}


void redRightAuton()
{
    // Example autonomous routine for "Red Right"
    drive(24,1000);   // Drive forward 24 inches
    turn(90);  // Turn to 90 degrees
    drive(12,1000);   // Drive forward 12 inches
    // Add more actions as needed
}