#include "vex.h"
#include "PTO.h" 



static DriveMode current_drive_mode = DRIVE_6_MOTOR; 


DriveMode setDriveMode(DriveMode targetMode) {
    

    // 2. Control the Pistons based on the target mode (using your confirmed logic)
    switch (targetMode) {
        
        case DRIVE_4_MOTOR:
            // 4 Motors: Both sets of PTO motors are retracted (Off the drive)
            DrivePTOPiston.set(true); // Retract
            IntakePTOPiston.set(false); // Retract
            break;

        case DRIVE_6_MOTOR:
            // 6 Motors: DrivePTO ON, IntakePTO OFF
            DrivePTOPiston.set(false); // Extend
            IntakePTOPiston.set(false); // Retract
            break;

        case DRIVE_8_MOTOR:
            // 8 Motors: Both DrivePTO ON and IntakePTO ON
            DrivePTOPiston.set(false); // Extend
            IntakePTOPiston.set(true); // Extend
            break;
            
    }

    // 3. Wait for the pistons to physically shift (CRITICAL for reliability)
    // TUNE THIS VALUE: It must be long enough for the air to move the cylinder!
    vex::wait(100, vex::msec); 

    // 4. Update the state tracker and return the new state
    current_drive_mode = targetMode;
}


/**
 * @brief Utility function to get the current drive mode.
 * @return The current DriveMode enum.
 */
DriveMode getCurrentDriveMode() {
    return current_drive_mode;
}