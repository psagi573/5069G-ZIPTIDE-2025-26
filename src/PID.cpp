#include "PID.h"
#include "vex.h"
#include <cmath>
#include <algorithm>

template <typename T>
T clamp(T value, T minVal, T maxVal)
{
    if (value < minVal)
        return minVal;
    if (value > maxVal)
        return maxVal;
    return value;
}

PID::PID(double p, double i, double d, double cap)
{
    kP = p;
    kI = i;
    kD = d;
    outputCap = cap;
    integral = 0;
    prevError = 0;
}

void PID::reset()
{
    integral = 0;
    prevError = 0;
}

double PID::compute(double target, double current)
{
    double error = target - current;
    integral += error;

    double integralCap = 50.0;

    integral = clamp(integral, -integralCap, integralCap);

    double derivative = error - prevError;
    prevError = error;

    double output = (error * kP) + (integral * kI) + (derivative * kD);
    // Clamp output manually since std::clamp may not be available

    output = clamp(output, -outputCap, outputCap);

    return output;
}
