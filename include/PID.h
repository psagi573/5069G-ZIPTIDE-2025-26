#include "vex.h"

class PID
{
private:
    double kP, kI, kD;
    double integral;
    double prevError;
    double outputCap;

public:
    PID(double p, double i, double d, double cap = 11.0);
    void reset();
    double compute(double target, double current);
};
