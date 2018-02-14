#ifndef PID_H
#define PID_H

namespace ev3ros {

class pid
{
public:
    pid();
    pid(double min, double max, double kp, double ki, double kd);
    void config(double min, double max, double kp, double ki, double kd);
    void setPID(double kp, double ki, double kd);
    void setLimits(double min, double max);
    void reset();
    double calculate(double target, double input, double dt);
private:
    double tKp, tKi, tKd;
    double iTerm, lastInput, output;
    double outMin, outMax;

};

}
#endif // PID_H
