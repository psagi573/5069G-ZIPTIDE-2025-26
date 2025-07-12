#include "PID.h"
#include "vex.h"
#include <cmath>
#include <algorithm>
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
    double derivative = error - prevError;
    prevError = error;

    double output = (error * kP) + (integral * kI) + (derivative * kD);
    // Clamp output manually since std::clamp may not be available
    if (output > outputCap)
        output = outputCap;
    if (output < -outputCap)
        output = -outputCap;
    return output;
}
