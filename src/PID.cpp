#include "PID.h"
#include "vex.h"
#include <cmath>




class PID {
private:
    const float kP, kI, kD;
    float integral = 0;
    float prevError = 0;
    float integralLimit = 1000;

public:
    PID(float p, float i, float d) : kP(p), kI(i), kD(d) {}

    float update(float Error) {
        integral = fmax(-integralLimit, fmin(integralLimit, integral + Error));
        float derivative = Error - prevError;
        prevError = Error;
        return kP * Error + kI * integral + kD * derivative;
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }
};
/*
class linearPID {
private:
    const float kP, kI, kD;
    float integral = 0;
    float prevError = 0;
    float integralLimit = 1000;

public:
    linearPID(float p, float i, float d) : kP(p), kI(i), kD(d) {}

    float update(float linearError) {
        integral = fmax(-integralLimit, fmin(integralLimit, integral + linearError));
        float derivative = linearError - prevError;
        prevError = linearError;
        return kP * linearError + kI * integral + kD * derivative;
    }

    void reset() {
        integral = 0;
        prevError = 0;
    }
};

class angularPID {
private:
    const float tkP, tkI, tkD;
    float integral = 0;
    float tprevError = 0;
    float integralLimit = 1000;

public:
    angularPID(float tp, float ti, float td) : tkP(tp), tkI(ti), tkD(td) {}

    float update(float angularError) {
        integral = fmax(-integralLimit, fmin(integralLimit, integral + angularError));
        float derivative = angularError - tprevError;
        tprevError = angularError;
        return tkP * angularError + tkI * integral + tkD * derivative;
    }

    void reset() {
        integral = 0;
        tprevError = 0;
    }
}; */