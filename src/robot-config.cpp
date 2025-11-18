#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
motor L1 = motor(PORT1, ratio6_1, true);
motor L2 = motor(PORT10, ratio6_1, true);
motor PTOL6 = motor(PORT11, ratio6_1, false);
motor R6 = motor(PORT12, ratio6_1, false);
motor R7 = motor(PORT13, ratio6_1, true);
motor PTOR8 = motor(PORT15, ratio6_1, false);
motor LIntake = motor(PORT9, ratio18_1, false);
motor RIntake = motor(PORT5, ratio18_1, true);
inertial inertial19 = inertial(PORT14);
digital_out IntakePTOPiston = digital_out(Brain.ThreeWirePort.G);
digital_out DrivePTOPiston = digital_out(Brain.ThreeWirePort.H);
digital_out Lifter = digital_out(Brain.ThreeWirePort.C);
digital_out Hook = digital_out(Brain.ThreeWirePort.D);
digital_out Loader = digital_out(Brain.ThreeWirePort.F);
motor_group DrivePTO = motor_group(PTOL6, PTOR8);
motor_group IntakePTO = motor_group(LIntake, RIntake);
motor_group drivetrain = motor_group(L1, L2, R6, R7);
motor_group Intake = motor_group(LIntake, RIntake);

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