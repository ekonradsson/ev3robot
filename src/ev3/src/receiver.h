#pragma once

#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

#include <serial/serial.h>

namespace ev3ros {

static const uint8_t channelCount = 7;

static const int rcStickAil = 0;
static const int rcSwitchFlap = 1;
static const int rcStickEle = 2;
static const int rcSwitchGear = 3;
static const int rcStickRud = 4;
static const int rcStickThr = 6;

static const int rcVelocity = rcStickEle;
static const int rcRotation = rcStickAil;
static const int rcTilt = rcStickThr;

class receiver
{
public:
    receiver();
    ~receiver();

    bool init(const std::string port, const uint32_t baudrate);

    bool getValues(const uint8_t id, const uint8_t count, std::vector<uint16_t> &array);
private:
    std::mutex mutex;
    serial::Serial ser;

    void threadTask();
    std::thread *serialThread;
    std::atomic<bool> stopRequest;

    std::string port;
    uint32_t baud;
    uint16_t timeout = 50;
    std::vector<uint16_t> values;
};

} //namespace
