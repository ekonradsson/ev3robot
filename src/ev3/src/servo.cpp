#include <cassert>
#include <chrono>
#include <math.h>

#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"
#include "ev3_sensor.h"

#include "servo.h"
#include "motor.h"
#include "pid.h"
#include "utils.h"

#include <ros/ros.h>

namespace ev3ros {

servo::servo()
{
    this->updateThread = NULL;
    this->accel = 5000;
    this->vel = 1000;
    this->margin = 10;
    this->posTarget = 0;
    this->stopRequest.store(false);
    this->posPid = new ev3ros::pid();
    this->posPid->setPID(0.5, 0.0, 0.0);
    this->motor = new ev3ros::motor(OUTPUT_A);
    assert(this->motor!=NULL);
}

servo::~servo()
{
    if (this->updateThread!=NULL)
    {
        this->stopRequest.store(true);
        this->updateThread->join();
        delete this->updateThread;
        this->stopRequest.store(false);
    }
    motor->stop();
    delete motor;
    delete posPid;
}

bool servo::init(double scale)
{
    if (motor->init(TACHO_BRAKE))
    {
        motor->configure(scale);
        motor->setVelPID(600,15,0);

        if ( ev3_search_sensor( LEGO_EV3_TOUCH, &limitSensor, 0 ))
        {
            int value;
            get_sensor_value(0, limitSensor, &value);
            while (value==0)
            {
                motor->setVel(500);
                Sleep(1);
                get_sensor_value(0, limitSensor, &value);
            }
            motor->stop();
            get_sensor_value(0, limitSensor, &value);
            while (value==1)
            {
                motor->setVel(-200);
                Sleep(1);
                get_sensor_value(0, limitSensor, &value);
            }
            motor->stop();
        }

        motor->reference();
        return setRate(updateRate);
    }
    return false;
}

void servo::setPosPID(double kp, double ki, double kd)
{
    pidMutex.lock();
    posPid->setPID(kp, ki, kd);
    pidMutex.unlock();
}

void servo::setVelPID(int kp, int ki, int kd)
{
    motorMutex.lock();
    motor->setVelPID(kp, ki, kd);
    motorMutex.unlock();
}

void servo::setPolarity(const char *value)
{
    motorMutex.lock();
    motor->setPolarity(value);
    motorMutex.unlock();
}

bool servo::setRate(int rate)
{
    updateRate = abs(rate);

    if (updateThread!=NULL)
    {
        stopRequest.store(true);
        updateThread->join();
        delete updateThread;
        stopRequest.store(false);
    }
    updateThread = new std::thread(&servo::threadTask, this);
    assert(updateThread!=NULL);
    return true;
}

void servo::setPos(double _pos)
{
    this->posTarget = _pos;
}

double servo::getPos()
{
    motorMutex.lock();
    double pos = motor->getPos();
    motorMutex.unlock();
    return pos;
}

void servo::setParams(double _accel, double _vel, double _margin, double _scale)
{
    this->accel = fabs(_accel);
    this->vel = fabs(_vel);
    this->margin = fabs(_margin);
    motorMutex.lock();
    motor->configure(_scale);
    motorMutex.unlock();
}

void servo::threadTask() // position b
{
    double velMax = this->vel;
    double velAct = motor->getVel();
    double velSet = 0;
    double velTarget = this->vel;
    double velCorr = 0;
    double accelSet = 0;

    double dtSet = (double)1.0/updateRate;
    double dtAct = dtSet;

    double timeAccel = 0.0;
    double timeDecel = 0.0;
    double timeCruise = 0.0;
    double timeTotal = 0.0;
    double timeCurrent = 0.0;

    double distAccel = 0;
    double distDecel = 0;
    double distTotal = 0;
    double distCurrent = 0;

    int8_t direction = 0;
    int8_t state = 0;

    double posTarget = 0;
    double posSet = posTarget;
    double posTargetLast = posTarget;
    double posAct = motor->getPos();
    double posError = posAct - posSet;
    double posStart = posAct;

    posPid->setLimits(-velMax, velMax);
    ros::Rate loop(updateRate);

    static std::chrono::steady_clock::time_point lastTime = std::chrono::steady_clock::now();

    debug_print("servo : task starting...\n");

    while(!stopRequest.load())
    {
        motorMutex.lock();
        velAct = motor->getVel();
        posAct = motor->getPos();
        motorMutex.unlock();

        std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
        auto secs = std::chrono::duration_cast<std::chrono::duration<float>>(now-lastTime);

        dtAct = secs.count();
        lastTime = now;

        posSet += velSet * dtAct;
        posError = posAct - posSet;

        posTarget = this->posTarget;
/*
        printf("servo : state %d, dir %d, pos [target %.0f, set %.0f, act %.0f, error %.0f], vel [set %.0f, act, %.0f, corr %.0f]\n",
           state, direction, posTarget, posSet, posAct, posError,
           velSet, velAct, velCorr);
*/
        if (fabs(posTarget - posTargetLast) > this->margin)
        {
            posTargetLast = posTarget;
            if (posTarget >= posAct)
            {
                accelSet = this->accel;
                direction = 1;
            } else
            {
                accelSet = -this->accel;
                direction = -1;
            }

            velMax = this->vel * direction;

            posSet = posAct;

            distTotal = fabs(posTarget - posSet);

            timeAccel = ((velMax - velSet) / accelSet);
            timeDecel = ((0 - velMax) / (-accelSet));

            distAccel = fabs(0.5 * (velSet + velMax) * timeAccel);
            distDecel = fabs(0.5 * (velMax) * timeDecel);

            if (distTotal > distAccel + distDecel)
            {
                velTarget = velMax ;
                timeCruise = fabs((distTotal - distAccel - distDecel) / velTarget);
            } else
            {
                velTarget = sqrt(fabs((accelSet * velSet * velSet + 2.0 * accelSet * accelSet * distTotal) / (accelSet + accelSet)));
                velTarget *= direction;

                timeAccel = (velTarget - velSet) / accelSet;
                timeDecel = (0 - velTarget) / (-accelSet);
                timeCruise = 0.0;

                distAccel = fabs(0.5 * (velSet + velTarget) * timeAccel);
                distDecel = fabs(0.5 * (velTarget) * timeDecel);

            }

            timeTotal = timeAccel + timeCruise + timeDecel;
            timeCurrent = 0.0;
            posStart = posAct;
            velCorr = 0;

/*
            printf("servo : state %d, dir %d, time [total %.2f, curr %.2f, accel %.2f, decel %.2f], dist [total %.0f, curr %.0f, accel %.0f, decel %.0f]\n",
               state, direction,
               timeTotal, timeCurrent, timeAccel, timeDecel,
               distTotal, distCurrent, distAccel, distDecel
               );
*/

        } else
        {
            if (state!=0)
            {
                timeCurrent += dtAct;
                pidMutex.lock();
                velCorr = posPid->calculate(posSet, posAct, dtAct);
                pidMutex.unlock();
            } else
            {
                velSet = 0;
            }
        }

        distCurrent = fabs(posAct - posStart);

        if (timeCurrent < timeAccel)
        {
            // accelerate
            state = 1;
            velSet += accelSet * dtAct;
            if (fabs(velSet) > fabs(velTarget))
            {
                velSet = velTarget;
                state = 2;
            }
        } else if ((timeCurrent > timeTotal - timeDecel) && (timeCurrent<timeTotal))
        {
            // decelerate
            state = 3;
            velSet -= accelSet * dtAct;

        } else if (timeCurrent>timeTotal)
        {
            // stop
            if (state!=0)
            {
                state = 4;
                velSet = 0;
                posSet = posTarget;

                if (fabs(posTarget - posAct) < this->margin)
                {
                    state = 0;
                    velCorr = 0;
                    //printf("servo : at position %.2f, error %.2f\n", posAct, posTarget - posAct);
                }
            }
        } else
        {
            // constant velocity
            state = 2;
            velSet = velTarget;
        }

        int16_t motorVel = (int16_t)(velSet + velCorr);

        if (motorVel > fabs(velMax))
            motorVel = fabs(velMax);

        if (motorVel < -fabs(velMax))
            motorVel = -fabs(velMax);

        motorMutex.lock();
        motor->setVel(motorVel);
        motorMutex.unlock();

        loop.sleep();
    }
    motor->stop();
    debug_print("servo : task stopped\n");
}

}
