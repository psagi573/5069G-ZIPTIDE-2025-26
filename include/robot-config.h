using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor L1;
extern motor R6;
extern inertial inertial19;
extern motor L2;
extern motor L3;
extern motor R7;
extern motor R8;
extern controller Controller2;
extern digital_out Lifter;
extern digital_out Trapdoor;
extern motor Roller;
extern motor Intake;
extern motor_group R;
extern motor_group L;
extern drivetrain Drivetrain;
extern motor_group Intake;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);