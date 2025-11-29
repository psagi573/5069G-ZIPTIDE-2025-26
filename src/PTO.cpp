#include "PTOManager.h"
#include "vex.h"

PTOManager::PTOManager(std::vector<vex::motor*> leftMotors,
                       std::vector<vex::motor*> rightMotors,
                       vex::digital_out& drivePTO,
                       vex::digital_out& intakePTO)
    : leftAll(leftMotors), rightAll(rightMotors),
      drivePiston(drivePTO), intakePiston(intakePTO),
      currentDriveMode(DRIVE_6_MOTOR) {}

// --- Set drive mode ---
DriveMode PTOManager::setDriveMode(DriveMode targetMode) {
    if(targetMode == currentDriveMode) return currentDriveMode;

    switch(targetMode) {
        case DRIVE_4_MOTOR:
            drivePiston.set(true);   // retract drive PTO
            intakePiston.set(false); // retract intake PTO
            break;
        case DRIVE_6_MOTOR:
            drivePiston.set(false);  // extend drive PTO
            intakePiston.set(false);
            break;
        case DRIVE_8_MOTOR:
            drivePiston.set(false);
            intakePiston.set(true);  // extend intake PTO
            break;
    }
    vex::wait(100, vex::msec);
    currentDriveMode = targetMode;
    return currentDriveMode;
}

DriveMode PTOManager::getCurrentDriveMode() const {
    return currentDriveMode;
}

// --- Return only the motors currently engaged in drivetrain ---
std::vector<vex::motor*> PTOManager::getActiveLeftMotors() const {
    switch(currentDriveMode) {
        case DRIVE_4_MOTOR:
            return {leftAll[0], leftAll[1]};
        case DRIVE_6_MOTOR:
            return {leftAll[0], leftAll[1], leftAll[2]};
        case DRIVE_8_MOTOR:
            return {leftAll[0], leftAll[1], leftAll[2]}; // 4th motor is intake
    }
    return {leftAll[0], leftAll[1]}; // fallback
}

std::vector<vex::motor*> PTOManager::getActiveRightMotors() const {
    switch(currentDriveMode) {
        case DRIVE_4_MOTOR:
            return {rightAll[0], rightAll[1]};
        case DRIVE_6_MOTOR:
            return {rightAll[0], rightAll[1], rightAll[2]};
        case DRIVE_8_MOTOR:
            return {rightAll[0], rightAll[1], rightAll[2]}; // 4th motor is intake
    }
    return {rightAll[0], rightAll[1]}; // fallback
}