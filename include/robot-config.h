using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor L1;
extern motor R6;
extern inertial inertial19;
extern motor L2;
extern motor PTOL3;
extern motor R7;
extern motor PTOR8;
extern controller Controller2;
extern digital_out DrivePTOPiston;
extern digital_out IntakePTOPiston;
extern motor RIntake;
extern motor LIntake;
extern motor_group DrivePTO;
extern motor_group IntakePTO;
extern motor_group drivetrain;
extern motor_group Intake;
extern motor_group Intake4;
extern motor_group L;
extern motor_group R;



/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);