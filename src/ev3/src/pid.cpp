#include "pid.h"

namespace ev3ros {

pid::pid()
{
    reset();
}

pid::pid(double min, double max, double kp, double ki, double kd)
{
    reset();
    config(min, max, kp, ki, kd);
}

void pid::reset()
{
    iTerm = 0;
    lastInput = 0;
    output = 0;
}

void pid::setPID(double kp, double ki, double kd)
{
    tKp = kp;
    tKi = ki;
    tKd = kd;
}

void pid::setLimits(double min, double max)
{
    outMin = min;
    outMax = max;
}


void pid::config(double min, double max, double kp, double ki, double kd)
{
    setPID(kp, ki, kd);
    setLimits(min, max);
}


double pid::calculate(double target, double input, double dt)
{
    double error = target - input;

    iTerm+= (tKi * dt * error);

    if(iTerm > outMax) iTerm= outMax;
    if(iTerm < -outMax) iTerm= -outMax;

    double dInput = (input - lastInput);
    lastInput = input;

    /*PID Output*/
    double output = tKp * error + iTerm - (tKd / dt) * dInput;

    if(output > outMax) output = outMax;
    if(output < -outMax) output = -outMax;

    return output;
}

}
