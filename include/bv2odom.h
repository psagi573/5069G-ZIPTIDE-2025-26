#pragma once

#include "vex.h"

struct OdomPose
{
    double x;     // inches
    double y;     // inches
    double theta; // degrees [0..360)
    double vx;    // inches/sec (world frame)
    double vy;    // inches/sec (world frame)
    double omega; // deg/sec
};

/// Start odometry background thread.
/// xSensor: rotation sensor for lateral tracker (X axis tracker) -- measures lateral movement (left/right)
/// ySensor: rotation sensor for longitudinal tracker (Y axis tracker) -- measures forward/back
/// imu: inertial sensor used for heading
/// waitForCalib: if true, this call will block until imu finishes calibrating (safe to call during 3s pre-match)
/// updateHz: odom loop frequency (default 100Hz)
void startOdom(vex::rotation &xSensor, vex::rotation &ySensor, vex::inertial &imu,
               bool waitForCalib = true, int updateHz = 100);

/// Stop the odom thread cleanly.
void stopOdom();

/// Get a copy of the current pose (thread-safe).
OdomPose getPose();

/// Reset pose (thread-safe). Useful to zero odom at match start.
void resetPose(double x = 0.0, double y = 0.0, double thetaDeg = 0.0);

/// Set odometry wheel geometry (inches). Call before startOdom for correctness.
void setOdomOffsets(double xTrackerOffsetInches, double yTrackerOffsetInches, double wheelDiameterInches = 2.0);

/// Set low-pass smoothing for velocities: 0.0 = no smoothing (raw), 1.0 = max smoothing (very slow)
void setVelocitySmoothing(double alphaV, double alphaOmega);

/// Set IMU fusion weight for heading blending [0..1].
/// imuWeight = 1.0 => trust IMU entirely; 0.0 => ignore IMU.
/// Typical: 0.8..0.98 (trust IMU mostly, but still smooth).
void setIMUWeight(double imuWeight);

/// Set maximum allowed IMU jump (deg) between loops. If IMU reading jumps more than this, it will be ignored that loop.
/// Useful to reject spikes from the IMU when physically hit.
void setIMUJumpThreshold(double maxDeg);

/// Set update frequency (Hz) while running (affects dt used to compute velocities)
void setUpdateHz(int hz);
