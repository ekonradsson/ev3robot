#pragma once

#include <thread>
#include <mutex>
#include <atomic>

#include "motor.h"

namespace ev3ros {

class driveCtrl
{
public:
    driveCtrl();
    ~driveCtrl();

    bool	init();
    void    configure(double _width, double _wheelDaim, double _gearRatio, double _velAccel, double _rotAccel, double _maxOutRatio);
    void    stop();
    void    stop(int stopAction);
    void    setStopAction(int stopAction);

    void	setVel(double vel, double _rot, bool ramp = true);
    bool    updateMotors(double yaw, double rotAct, double dt);

    void    setVelPID(int kp, int ki, int kd);
    void    setHoldPID(int kp, int ki, int kd);
    void    setRotPID(double kp, double ki, double kd);

    void getPose(double &lPos, double &rPos, double &rot);

private:
    void updateScale();

    // member variables
    double  wheelDiam;
    double  encoderRes;
    double  gearRatio;
    double	trackWidth;
    double  maxOutRatio;
    double  linAccel;

    double  rotAccel;

    double  rotMax;
    double	linMax;
    double  outMax;
    double  motorMax;

    double  tKp, tKi, tKd;

    double  scale;
    bool    resetPID;
    int     stopAction;

    std::atomic<double> rotSet;
    std::atomic<double> linSet;

    std::atomic<double>	linTarget;
    std::atomic<double>	rotTarget;

    motor		*left;
    motor		*right;

    std::mutex motorMutex;
};


}

