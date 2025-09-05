#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

controller Controller1 = controller(primary);
motor L1 = motor(PORT1, ratio6_1, true);
motor R6 = motor(PORT6, ratio6_1, false);
rotation Yaxis = rotation(PORT17, true);
rotation Xaxis = rotation(PORT12, true);
inertial inertial19 = inertial(PORT11);
motor L2 = motor(PORT2, ratio6_1, true);
motor L3 = motor(PORT3, ratio6_1, true);
motor R7 = motor(PORT7, ratio6_1, false);
motor R8 = motor(PORT8, ratio6_1, false);
controller Controller2 = controller(partner);
digital_out mogo = digital_out(Brain.ThreeWirePort.C);
digital_out WallStake = digital_out(Brain.ThreeWirePort.H);
digital_out Doinker = digital_out(Brain.ThreeWirePort.G);
digital_out Hood = digital_out(Brain.ThreeWirePort.A);
motor Hook = motor(PORT5, ratio6_1, true);
motor RollerIntake = motor(PORT10, ratio6_1, false);

// VEXcode device constructors
/*controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
motor L1 = motor(PORT19, ratio6_1, true);
motor L2 = motor(PORT17, ratio6_1, true);
motor L3 = motor(PORT7, ratio6_1, true);
motor R6 = motor(PORT12, ratio6_1, false);
motor R7 = motor(PORT13, ratio6_1, false);
motor R8 = motor(PORT20, ratio6_1, false);
motor Out = motor(PORT10, ratio18_1, false);
motor Take = motor(PORT9, ratio18_1, true);
motor RollerIntake = motor(PORT21, ratio6_1, false);
rotation Yaxis = rotation(PORT3, false);
rotation Xaxis = rotation(PORT5, true);
inertial inertial19 = inertial(PORT11);
digital_out Lifter = digital_out(Brain.ThreeWirePort.A);*/

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