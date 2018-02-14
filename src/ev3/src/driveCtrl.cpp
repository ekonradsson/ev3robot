#include <iostream>
#include <cassert>
#include <cmath>
#include <chrono>

#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"

#include "driveCtrl.h"
#include "utils.h"

namespace ev3ros {

//#define DEBUG_DRIVE
//#define DEBUG_DRIVE_PID

driveCtrl::driveCtrl()
{
    wheelDiam = 0.0385022;
    encoderRes = 360;
    gearRatio = -1;
    trackWidth = 0.18;
    maxOutRatio = 0.9;

    tKp = 0.4;
    tKi = 6.0;
    tKd = 0.005;

    outMax = 0;

    linAccel = 0.8;
    linTarget.store(0);
    linSet.store(0);

    rotAccel = 6.0;
    rotTarget.store(0);
    rotSet.store(0);

    scale = 0;
    linMax = 0.0;
    rotMax = 0.0;

    stopAction = TACHO_BRAKE;

    left = new motor(OUTPUT_B);
    right = new motor(OUTPUT_C);
}

driveCtrl::~driveCtrl()
{
    left->stop();
    right->stop();
    delete left;
    delete right;
}

void driveCtrl::setStopAction(int _stopAction)
{
    std::lock_guard<std::mutex> lock(motorMutex);
    stopAction = _stopAction;
    left->setStopAction(stopAction);
    right->setStopAction(stopAction);
}

void driveCtrl::setVelPID(int kp, int ki, int kd)
{
    std::lock_guard<std::mutex> lock(motorMutex);
    left->setVelPID(kp, ki, kd);
    right->setVelPID(kp, ki, kd);
}

void driveCtrl::setHoldPID(int kp, int ki, int kd)
{
    std::lock_guard<std::mutex> lock(motorMutex);
    left->setHoldPID(kp, ki, kd);
    right->setHoldPID(kp, ki, kd);
}

void driveCtrl::setRotPID(double kp, double ki, double kd)
{
    tKp = kp;
    tKi = ki;
    tKd = kd;
}

void driveCtrl::configure(double _width, double _wheelDaim, double _gearRatio, double _velAccel, double _rotAccel, double _maxOutRatio)
{
    std::lock_guard<std::mutex> lock(motorMutex);

    trackWidth = _width;
    wheelDiam = _wheelDaim;
    gearRatio = _gearRatio;
    linAccel = _velAccel;
    rotAccel = _rotAccel;
    maxOutRatio = _maxOutRatio;

    updateScale();

    resetPID = true;
}

bool driveCtrl::init()
{
    std::lock_guard<std::mutex> lock(motorMutex);

    if (right->init(stopAction) && left->init(stopAction)) //TACHO_HOLD
    {
        if (left->getEncoderRes() == right->getEncoderRes())
        {
            encoderRes = left->getEncoderRes();
        } else
        {
            debug_print("driveCtrl : Motor encoder resolution  does not match\n");
            return false;
        }

        updateScale();

        linTarget.store(0);
        rotTarget.store(0);
        linSet.store(0);
        rotSet.store(0);
        resetPID = true;

        return true;
    } else
    {
        return false;
    }
}

void driveCtrl::updateScale()
{
    if ((encoderRes!=0) && (gearRatio!=0))
    {
        scale = encoderRes / gearRatio;
    } else
    {
        scale = 0;
    }

    left->configure(scale);
    right->configure(scale);
    motorMax = std::min(left->getMaxVel(), right->getMaxVel());
    outMax = motorMax * maxOutRatio; //revs per second

    double wheelDist=M_PI*wheelDiam; //dist traveled for one motor revolution
    linMax = wheelDist*outMax; // maximum linear velocity

    double turnDist = trackWidth*M_PI; // dist traveled to turn full circle
    double motorRevs = wheelDist!=0 ? turnDist/wheelDist : 0; // motor revolutions to turn full circle
    double turnTime = outMax!=0 ? motorRevs/outMax : 0; // time to turn full circle

    rotMax = turnTime != 0 ? 1/turnTime * M_PI * 2 : 0;
}

void driveCtrl::stop()
{
    stop(stopAction);
}

void driveCtrl::stop(int _stopAction)
{
    std::lock_guard<std::mutex> lock(motorMutex);

    linTarget.store(0);
    rotTarget.store(0);

    linSet.store(0);
    rotSet.store(0);

    left->stop(_stopAction);
    right->stop(_stopAction);
}

bool driveCtrl::updateMotors(double yaw, double rotAct, double dtSet)
{
    static double iTerm, lastInput;
    static std::chrono::steady_clock::time_point lastTime = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(motorMutex);

    double linTarget = this->linTarget.load();
    double rotTarget = this->rotTarget.load();
    double dtAct = dtSet;

    if (resetPID)
    {
        resetPID = false;
        iTerm = 0;
        linSet.store(0);
        rotSet.store(0);
        lastInput = rotAct;
    }

    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    auto secs = std::chrono::duration_cast<std::chrono::duration<float>>(now-lastTime);

    dtAct = secs.count();
    lastTime = now;

    double input = rotAct;

    if ((linTarget==0) && (rotTarget==0) && (stopAction!=TACHO_HOLD))
    {
        input = 0;
        iTerm = 0;
    }

    double _linSet = this->linSet.load();
    double _rotSet = this->rotSet.load();

    double error = _rotSet - input;

    iTerm+= (tKi * dtAct * error);

    if(iTerm > motorMax) iTerm= motorMax;
    if(iTerm < -motorMax) iTerm= -motorMax;

    double dInput = (input - lastInput);
    lastInput = input;

    /*PID Output*/
    double output = tKp * error + iTerm - (dtAct!=0 ? (tKd / dtAct) : 0) * dInput;

    if(output > motorMax) output = motorMax;
    if(output < -motorMax) output = -motorMax;

    if (linAccel!=0)
    {
        if (linTarget>_linSet)
        {
            _linSet += (linAccel*dtSet);

            if (_linSet>linTarget)
            {
                _linSet = linTarget;
            }
        } else if (linTarget<_linSet)
        {
            _linSet -= (linAccel*dtSet);

            if (_linSet<linTarget)
            {
                _linSet = linTarget;
            }
        }
    } else
    {
        _linSet = linTarget;
    }

    if (rotAccel!=0)
    {
        if (rotTarget>_rotSet)
        {
            _rotSet += (rotAccel*dtSet);

            if (_rotSet>rotTarget)
            {
                _rotSet = rotTarget;
            }
        } else if (rotTarget<_rotSet)
        {
            _rotSet -= (rotAccel*dtSet);
            if (_rotSet<rotTarget)
            {
                _rotSet = rotTarget;
            }
        }
    } else
    {
        _rotSet = rotTarget;
    }

    linSet.store(_linSet);
    rotSet.store(_rotSet);

    double lVel = ((_linSet - _rotSet*trackWidth/2.0) / wheelDiam/2.0)-output;
    double rVel = ((_linSet + _rotSet*trackWidth/2.0) / wheelDiam/2.0)+output;

    double ratio = std::max(fabs(lVel/motorMax), fabs(rVel/motorMax));

    double linAct = ((left->getVel() + right->getVel())/2) * (M_PI*wheelDiam);

    if (ratio>1) {
         lVel /= ratio;
         rVel /= ratio;
    }

    //printf("driveCtrl : linSet=%.2f, rotSet=%.2f, rotAct=%.2f, lVel=%.2f, rVel=%.2f, ratio=%.2f\n", _linSet, _rotSet, rotAct, lVel, rVel, ratio);
    debug_print("driveCtrl : linSet=%.2f, linAct=%.2f, rotSet=%.2f, rotAct=%.2f, error=%.2f, output=%.2f, iterm=%.2f\n", _linSet, linAct, _rotSet, rotAct, error, output, iTerm);


    if ((linTarget==0) && (rotTarget==0) && (fabs(lVel)<0.001) && (fabs(rVel<0.001)))
    {
        lVel = 0;
        rVel = 0;
    }

    left->setVel(lVel);
    right->setVel(rVel);

    return true;
}

void driveCtrl::setVel(double _vel, double _rot, bool ramp)
{
    #ifdef DEBUG_DRIVE
        printf("driveCtrl::setVel(%.2f,  %.2f, %d)\n", _vel, _rot, ramp);
    #endif

    double velAct = (left->getVel() + right->getVel())/2;
    double linAct = velAct * M_PI * wheelDiam;

    double lVel = (_vel - _rot*trackWidth/2.0) / wheelDiam/2.0;
    double rVel = (_vel + _rot*trackWidth/2.0) / wheelDiam/2.0;
    double ratio = std::max(fabs(lVel/outMax), fabs(rVel/outMax));

    if (ratio>1)
    {
        _vel /= ratio;
        _rot /= ratio;
    }

    //printf("driveCtrl : reqVel=%.2f, limVel=%.2f, linAct=%.2f, reqRot=%.2f, ratio=%.2f\n", reqVel, _vel, linAct, _rot, ratio);

    linTarget.store(_vel);
    rotTarget.store(_rot);

    if (!ramp)
    {
        linSet.store(_vel);
        rotSet.store(_rot);
    }
}

void driveCtrl::getPose(double &lPos, double &rPos, double &rot)
{
    lPos = left->getPos()*(M_PI*wheelDiam);
    rPos = right->getPos()*(M_PI*wheelDiam);
    rot = ((lPos - rPos) / (-trackWidth));
}

} //namespace


