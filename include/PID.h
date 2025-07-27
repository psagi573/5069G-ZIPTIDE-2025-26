#include "vex.h"

class PID
{
private:
public:
    double kP, kI, kD;
    double integral;
    double prevError;
    double error;
    PID(double p, double i, double d);
    void reset();
    double compute(double target, double current, bool turn = false);
};
