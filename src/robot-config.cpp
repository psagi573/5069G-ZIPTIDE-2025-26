#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
controller Controller2 = controller(partner);
motor L1 = motor(PORT12, ratio6_1, true);
motor L2 = motor(PORT13, ratio6_1, true);
motor daddy = motor(PORT5, ratio36_1, false);
motor L3 = motor(PORT14, ratio6_1, true);
motor R6 = motor(PORT18, ratio6_1, false);
motor R7 = motor(PORT19, ratio6_1, false);
motor R8 = motor(PORT20, ratio6_1, false);
motor Out = motor(PORT1, ratio18_1, false);
motor Take = motor(PORT3, ratio18_1, true);
motor RollerIntake = motor(PORT10, ratio6_1, false);
rotation Yaxis = rotation(PORT15, false);
rotation Xaxis = rotation(PORT16, true);
inertial inertial19 = inertial(PORT11);
digital_out Lifter = digital_out(Brain.ThreeWirePort.H);
digital_out Trapdoor = digital_out(Brain.ThreeWirePort.G);
digital_out Loader = digital_out(Brain.ThreeWirePort.F);
optical Color = optical(PORT17);

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