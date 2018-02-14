//#include <ros/ros.h>
#include <iostream>
#include <cassert>
#include <math.h>

#include "ev3.h"
#include "ev3_port.h"
#include "ev3_tacho.h"

#include "motor.h"
#include "utils.h"

namespace ev3ros {

motor::motor(uint8_t _port)
{
    port = _port;
    handle = DESC_LIMIT;

    maxOut = 0;
    maxVel = 0;
    scale = 1;
    targetVel = 0;
    absPos = 0;
    stopAction = TACHO_COAST;
}

motor::~motor()
{
    stop();
}

void  motor::configure(double _scale)
{
    if (_scale!=0)
    {
        scale = _scale;
        maxVel = fabs(maxOut/scale);
        reset();
    }
}

void motor::setPolarity(const char *value)
{
    set_tacho_polarity(handle, (char*)value);
}

void motor::setStopAction(int _stopAction)
{
    if (stopAction != _stopAction)
    {
        stopAction = _stopAction;
        set_tacho_stop_action_inx(handle, _stopAction);
        if (!isRuning())
        {
            stop();
        }
    }
}

bool motor::init(int _stopAction)
{
    ev3_port_name( port, EXT_PORT__NONE_, 0, portName);

    stopAction = _stopAction;

    if (ev3_search_tacho_plugged_in(port, EXT_PORT__NONE_, &handle, 0 ))
    {
        get_tacho_max_speed(handle, &maxOut);
        setStopAction(stopAction);

        int velKp=0, velKi=0, velKd=0;
        getVelPID(velKp, velKi, velKd);

        int holdKp=0, holdKi=0, holdKd=0;
        getHoldPID(holdKp, holdKi, holdKd);

        debug_print("Motor dedected on %s ,max out %d, vel PID %d, %d, %d, hold PID %d, %d, %d\n", portName, maxOut, velKp, velKi, velKd, holdKp, holdKi, holdKd);

        return true;
    } else
    {
        printf("Motor not dedected on %s\n", portName);
        return false;
    }
}

void motor::reference()
{
    absPos = 0;
    set_tacho_command_inx(handle, TACHO_RESET);
    targetVel = 0;
}

void motor::reset()
{
    int pos = 0;
    get_tacho_position(handle, &pos);
    absPos += pos;

    set_tacho_command_inx(handle, TACHO_RESET);
    set_tacho_stop_action_inx(handle, stopAction);
    targetVel = 0;
}

char* motor::getPortName()
{
    return portName;
}

void motor::stop()
{
    set_tacho_command_inx(handle, TACHO_STOP);
}

void motor::stop(int _stopAction)
{
    set_tacho_stop_action_inx(handle, _stopAction);
    set_tacho_command_inx(handle, TACHO_STOP);
}

int motor::getEncoderRes()
{
    int count;
    get_tacho_count_per_rot(handle, &count);
    return count;
}

void motor::getVelPID(int &_kp, int &_ki, int &_kd)
{
    get_tacho_speed_pid_Kp(handle, &_kp);
    get_tacho_speed_pid_Ki(handle, &_ki);
    get_tacho_speed_pid_Kd(handle, &_kd);
}

void motor::setVelPID(int _kp, int _ki, int _kd)
{
    set_tacho_speed_pid_Kp(handle, _kp);
    set_tacho_speed_pid_Ki(handle, _ki);
    set_tacho_speed_pid_Kd(handle, _kd);
}


void motor::getHoldPID(int &_kp, int &_ki, int &_kd)
{
    get_tacho_hold_pid_Kp(handle, &_kp);
    get_tacho_hold_pid_Ki(handle, &_ki);
    get_tacho_hold_pid_Kd(handle, &_kd);
}

void motor::setHoldPID(int _kp, int _ki, int _kd)
{
    set_tacho_hold_pid_Kp(handle, _kp);
    set_tacho_hold_pid_Ki(handle, _ki);
    set_tacho_hold_pid_Kd(handle, _kd);
}

void motor::setPos(int16_t pos)
{
    set_tacho_position_sp(handle, pos*scale);
    set_tacho_command_inx(handle, TACHO_RUN_TO_ABS_POS );
}

void motor::setPos(int16_t pos, int16_t speed)
{
    set_tacho_speed_sp(handle, speed*scale);
    set_tacho_position_sp(handle, pos*scale);
    set_tacho_command_inx(handle, TACHO_RUN_TO_ABS_POS );
}

void motor::setPos(int16_t pos, int16_t speed, int16_t ramp)
{
    set_tacho_ramp_up_sp(handle, ramp);
    set_tacho_ramp_down_sp(handle, ramp);
    set_tacho_speed_sp(handle, speed*scale);
    set_tacho_position_sp(handle, pos*scale);
    set_tacho_command_inx(handle, TACHO_RUN_TO_ABS_POS );
}

double motor::getPos()
{
    //todo store current pos each time scale or ratio is changed, and reset motor pos
    int pos = 0;
    get_tacho_position(handle, &pos);
    return ((absPos+pos) / scale);
}

double motor::getMaxVel()
{
    return maxVel;
}

double motor::getScale()
{
    return scale; //convert to seconds
}

void motor::setDuty(int value)
{
    set_tacho_duty_cycle_sp(handle, value);
    set_tacho_command_inx(handle, TACHO_RUN_DIRECT );
}

int motor::getDuty()
{
    int duty = 0;
    get_tacho_duty_cycle(handle, &duty);
    return duty;
}

void motor::setVel(double _vel)
{
    if (_vel!=targetVel)
    {
        targetVel = _vel;

        if (targetVel > maxVel)
        {
            targetVel = maxVel;
        } else if (targetVel < -maxVel)
        {
            targetVel = -maxVel;
        }

        if (fabs(targetVel*scale)<5) targetVel = 0;

        if (((targetVel!=0) || (stopAction==TACHO_HOLD)))
        {
            set_tacho_speed_sp(handle, targetVel*scale);
            set_tacho_command_inx(handle, TACHO_RUN_FOREVER );
        } else
        {
            set_tacho_command_inx(handle, TACHO_STOP);
        }
    }
}

double motor::getVel()
{
    int speed;
    get_tacho_speed( handle, &speed );

    return speed / scale;
}

bool  motor::isMoving()
{
    int speed;
    get_tacho_speed( handle, &speed );
    return (abs(speed)>0);
}

int motor::getState()
{
    get_tacho_state_flags(handle, &state);
    return (int)state;
}

bool motor::isStalled()
{
    get_tacho_state_flags(handle, &state);
    return state & TACHO_STALLED;
}

bool motor::isOverLoaded()
{
    get_tacho_state_flags(handle, &state);
    return state & TACHO_OVERLOADED;
}

bool motor::isRuning()
{
    get_tacho_state_flags(handle, &state);
    return state & TACHO_RUNNING;
}

bool motor::isRamping()
{
    get_tacho_state_flags(handle, &state);
    return state & TACHO_RAMPING;
}

bool motor::isHolding()
{
    get_tacho_state_flags(handle, &state);
    return state & TACHO_HOLDING;
}

} //namespace


