// odom.h
#pragma once

#include "vex.h"

/**
 * @brief Pose structure representing robot position and orientation
 */
struct Pose {
    double x;           // X position in inches (field coordinates)
    double y;           // Y position in inches (field coordinates)  
    double theta;       // Heading in degrees (0° = positive X-axis, counterclockwise positive)
    
    // Optional: Raw sensor readings for debugging
    double xSensor;     // Raw X encoder reading in inches
    double ySensor;     // Raw Y encoder reading in inches
    
    // Default constructor
    Pose() : x(0.0), y(0.0), theta(0.0), xSensor(0.0), ySensor(0.0) {}
    
    // Parameterized constructor
    Pose(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_), xSensor(0.0), ySensor(0.0) {}
};

/**
 * @brief Odometer class for tracking robot position using two tracking wheels and IMU
 */
class OdomSystem {
private:
    // Sensor pointers
    vex::rotation* xRot;
    vex::rotation* yRot;
    vex::inertial* imuSensor;
    
    // Thread control
    vex::thread odomThread;
    bool odomRunning;
    vex::mutex poseMutex;
    
    // Robot configuration
    double xOffset;     // Forward offset from center to tracking wheel (inches)
    double yOffset;     // Lateral offset from center to tracking wheel (inches)
    double wheelDiameter;
    double wheelCircumference;
    
    // Previous readings
    double prevXEncoder;
    double prevYEncoder;
    double prevThetaRad;
    
    // Current pose
    Pose currentPose;
 
    
public:
    /**
     * @brief Constructor with custom offsets
     * @param xOffset Forward offset from robot center to tracking wheel (inches)
     * @param yOffset Lateral offset from robot center to tracking wheel (inches)
     * @param wheelDiameter Diameter of tracking wheels (inches)
     */
    OdomSystem(double xOffset = 5.25, double yOffset = 0.75, double wheelDiameter = 2.0);
    

    int odomTask();


    /**
     * @brief Destructor - automatically stops odometry thread
     */
    ~OdomSystem();
    
    /**
     * @brief Initialize odometry system with sensors
     * @param xSensor Reference to X (lateral) rotation sensor
     * @param ySensor Reference to Y (forward) rotation sensor  
     * @param imu Reference to inertial sensor
     */
    void start(vex::rotation& xSensor, vex::rotation& ySensor, vex::inertial& imu);
    
    /**
     * @brief Initialize odometry system with sensors at specific starting pose
     * @param xSensor Reference to X (lateral) rotation sensor
     * @param ySensor Reference to Y (forward) rotation sensor
     * @param imu Reference to inertial sensor
     * @param startX Starting X position (inches)
     * @param startY Starting Y position (inches)
     * @param startTheta Starting heading (degrees)
     */
    void startAt(vex::rotation& xSensor, vex::rotation& ySensor, vex::inertial& imu,
                 double startX, double startY, double startTheta);
    
    /**
     * @brief Stop the odometry system and thread
     */
    void stop();
    
    /**
     * @brief Get the current robot pose (thread-safe)
     * @return Current Pose structure
     */
    Pose getPose();
    
    /**
     * @brief Set the robot pose manually (for resetting or sensor fusion)
     * @param newPose New pose to set
     */
    void setPose(const Pose& newPose);
    
    /**
     * @brief Set the robot pose manually with individual parameters
     * @param x New X position (inches)
     * @param y New Y position (inches)
     * @param theta New heading (degrees)
     */
    void setPose(double x, double y, double theta);
    
    /**
     * @brief Reset the odometry system to zero pose
     */
    void reset();
    
    /**
     * @brief Reset the odometry system to a specific pose
     * @param x X position to reset to (inches)
     * @param y Y position to reset to (inches)
     * @param theta Heading to reset to (degrees)
     */
    void resetTo(double x, double y, double theta);
    
    /**
     * @brief Get the raw encoder readings in inches
     * @param xInches Output parameter for X encoder reading
     * @param yInches Output parameter for Y encoder reading
     */
    void getRawEncoders(double& xInches, double& yInches);
    
    /**
     * @brief Get the current IMU heading in degrees
     * @return Heading in degrees
     */
    double getIMUHeading();
    
    /**
     * @brief Print current pose to brain screen for debugging
     */
    void printPose();
    
    /**
     * @brief Check if odometry system is running
     * @return True if odometry thread is active
     */
    bool isRunning() const { return odomRunning; }
    
    /**
     * @brief Get the current wheel circumference
     * @return Wheel circumference in inches
     */
    double getWheelCircumference() const { return wheelCircumference; }
    
    /**
     * @brief Update wheel diameter (recalculates circumference)
     * @param newDiameter New wheel diameter in inches
     */
    void setWheelDiameter(double newDiameter);
    
    /**
     * @brief Update tracker offsets
     * @param newXOffset New forward offset in inches
     * @param newYOffset New lateral offset in inches
     */
    void setOffsets(double newXOffset, double newYOffset);
};

// Legacy C-style functions for backward compatibility
/**
 * @brief Initialize odometry system (legacy function)
 */
void startOdom(vex::rotation& xSensor, vex::rotation& ySensor, vex::inertial& imu);

/**
 * @brief Initialize odometry at specific starting pose (legacy function)
 */
void startOdomAt(vex::rotation& xSensor, vex::rotation& ySensor, vex::inertial& imu,
                 double startX, double startY, double startTheta);

/**
 * @brief Stop odometry system (legacy function)
 */
void stopOdom();

/**
 * @brief Get current pose (legacy function)
 */
Pose getPose();

/**
 * @brief Print pose for debugging (legacy function)
 */
void printPose();