#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
motor L1 = motor(PORT20, ratio6_1, true);
motor L2 = motor(PORT9, ratio6_1, true);
motor PTOL3 = motor(PORT10, ratio6_1, false);
motor LIntake = motor(PORT5, ratio6_1, false);

motor R6 = motor(PORT2, ratio6_1, false);
motor R7 = motor(PORT7, ratio6_1, false);
motor PTOR8 = motor(PORT3, ratio6_1, true);
motor RIntake = motor(PORT6, ratio6_1, true);
inertial inertial19 = inertial(PORT12);
digital_out IntakePTOPiston = digital_out(Brain.ThreeWirePort.G);
digital_out DrivePTOPiston = digital_out(Brain.ThreeWirePort.H);
motor_group DrivePTO = motor_group(PTOL3, PTOR8);
motor_group IntakePTO = motor_group(LIntake, RIntake);
motor_group drivetrain = motor_group(L1, L2, R6, R7);
motor_group Intake = motor_group(LIntake, RIntake);
motor_group Intake2 = motor_group(LIntake, RIntake, PTOR8, PTOL3);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void)
{
  // nothing to initialize
}