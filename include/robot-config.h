using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor L1;
extern motor R6;
/*extern rotation Yaxis;
extern rotation Xaxis;
extern inertial inertial19;*/
extern motor L2;
extern motor L3;
extern motor R7;
extern motor R8;
extern controller Controller2;
extern digital_out Lifter;
extern motor Out;
extern motor Take;
extern motor RollerIntake;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);