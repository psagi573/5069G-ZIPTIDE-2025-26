#include "vex.h"

class PID
{
private:
public:
    double kP, kI, kD;
    double integral;
    double prevError;
    double outputCap;
    PID(double p, double i, double d, double cap = 11.0);
    void reset();
    double compute(double target, double current);
};
