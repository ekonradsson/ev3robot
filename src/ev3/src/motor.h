#pragma once

#include "ev3_tacho.h"

namespace ev3ros {

class motor
{
public:
    motor(uint8_t port);
    ~motor();

    bool	init(int stopAction);
    void    configure(double scale);
    void	reset();
    void    reference();
    void    stop();
    void    stop(int stopAction);
    void    setStopAction(int stopAction);
    void    setPolarity(const char *value);

    void	setVel(double vel);
    double  getVel();

    void    setDuty(int value);
    int     getDuty();

    double  getScale();

    double	getPos();
    void    setPos(int16_t pos);
    void    setPos(int16_t pos, int16_t speed);
    void    setPos(int16_t pos, int16_t speed, int16_t ramp);

    double	getMaxVel();

    bool    isMoving();

    bool    isStalled();
    bool    isOverLoaded();
    bool    isRuning();
    bool    isRamping();
    bool    isHolding();
    
    int     getState();

    void    getVelPID(int &kp, int &ki, int &kd);
    void    setVelPID(int kp, int ki, int kd);

    void    getHoldPID(int &kp, int &ki, int &kd);
    void    setHoldPID(int kp, int ki, int kd);

    int     getEncoderRes();

    char*	getPortName();

private:

    char portName[255];
    FLAGS_T state;
    int     stopAction;

    double	scale;
    double  targetVel;
    double  maxVel;
    int     maxOut;
    int32_t absPos;

    uint8_t	port;
    uint8_t	handle;
};

}

