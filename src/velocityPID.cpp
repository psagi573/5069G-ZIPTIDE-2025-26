#include "velocityPID.h"
#include <algorithm>
#include <cmath>

template <typename T>
T clamp(T value, T minVal, T maxVal)
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}

VelocityPID::VelocityPID(double p, double i, double d, double cap)
{
    kP = p;
    kI = i;
    kD = d;
    outputCap = cap;
    integral = 0;
    prevError = 0;
}

void VelocityPID::reset()
{
    integral = 0;
    prevError = 0;
}

double VelocityPID::compute(double targetVelocity, double currentVelocity)
{
    double error = targetVelocity - currentVelocity;
    integral += error;

    // Limit integral windup
    integral = clamp(integral, -50.0, 50.0);

    double derivative = error - prevError;
    prevError = error;

    double output = (kP * error) + (kI * integral) + (kD * derivative);
    output = clamp(output, -outputCap, outputCap);
    return output;
}
