using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor L1;
extern motor R6;
extern rotation Yaxis;
extern rotation Xaxis;
extern inertial inertial19;
extern motor L2;
extern motor L3;
extern motor R7;
extern motor R8;
extern controller Controller2;
extern digital_out Lifter;
extern digital_out Loader;
extern digital_out Trapdoor;
extern motor Out;
extern motor Take;
extern motor jj;
extern motor jk;
extern motor RollerIntake;
extern optical Color;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);