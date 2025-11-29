#pragma once
#include "vex.h"
#include <vector>

enum DriveMode {
    DRIVE_4_MOTOR,
    DRIVE_6_MOTOR,
    DRIVE_8_MOTOR
};

class PTOManager {
public:
    PTOManager(std::vector<vex::motor*> leftMotors,
               std::vector<vex::motor*> rightMotors,
               vex::digital_out& drivePTO,
               vex::digital_out& intakePTO);

    DriveMode setDriveMode(DriveMode targetMode);
    DriveMode getCurrentDriveMode() const;

    // NEW: getters for currently active motors
    std::vector<vex::motor*> getActiveLeftMotors() const;
    std::vector<vex::motor*> getActiveRightMotors() const;

private:
    std::vector<vex::motor*> leftAll;
    std::vector<vex::motor*> rightAll;
    vex::digital_out& drivePiston;
    vex::digital_out& intakePiston;

    DriveMode currentDriveMode;
};