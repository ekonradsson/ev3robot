#pragma once

#include <thread>
#include <mutex>
#include <atomic>

#include "motor.h"
#include "pid.h"

namespace ev3ros {

class servo
{
public:
    servo();
    ~servo();
    bool init(double scale);
    bool setRate(int rate);
    void setPos(double pos);
    void setParams(double accel, double vel, double margin, double scale);
    void setPosPID(double kp, double ki, double kd);
    void setVelPID(int kp, int ki, int kd);
    void setPolarity(const char *value);
    double getPos();
private:
    void threadTask();
    ev3ros::motor *motor=NULL;
    ev3ros::pid *posPid=NULL;

    std::mutex pidMutex;
    std::mutex motorMutex;

    std::thread *updateThread=NULL;

    std::atomic<bool> stopRequest;
    std::atomic<double> posTarget;

    std::atomic<double> accel;
    std::atomic<double> margin;
    std::atomic<double> vel;
    uint8_t updateRate = 10;
    uint8_t limitSensor;
};

}
