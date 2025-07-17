#pragma once

class VelocityPID
{
public:
    VelocityPID(double p, double i, double d, double cap = 11.0);
    void reset();
    double compute(double targetVelocity, double currentVelocity);

private:
    double kP;
    double kI;
    double kD;
    double outputCap;

    double integral;
    double prevError;
};
