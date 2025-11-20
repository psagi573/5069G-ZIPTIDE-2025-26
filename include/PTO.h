// pto.h

enum DriveMode {
    DRIVE_4_MOTOR = 4, // L1, L2, R6, R7 on drive
    DRIVE_6_MOTOR = 6, // + PTOL3, PTOR8 on drive
    DRIVE_8_MOTOR = 8  // + LIntake, RIntake on drive
};

// Function declaration
DriveMode setDriveMode(DriveMode targetMode);
DriveMode getCurrentDriveMode();