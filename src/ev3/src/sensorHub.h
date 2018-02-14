#pragma once

#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include <serial/serial.h>

namespace ev3ros {

// 1 digital, 8 bits give 8 different inputs.
// 4 analog

class sensorHub
{
public:
    sensorHub(const uint16_t timeout);
    ~sensorHub();

    bool init(const std::string port, const uint32_t baudrate);
    bool start();
    void config(const uint16_t timeout, uint16_t retryCount);
    void reqestReading();
    bool getReading(int timeout, std::vector<uint16_t> &values);
private:
    serial::Serial ser;

    void notify();
    void threadTask();
    void stopThread();
    void startThread();

    bool readHub(std::vector<uint16_t> *data);

    std::thread *updateThread;
    std::atomic<bool> stopRequest;

    std::string port;
    uint32_t baud;
    uint16_t timeout;
    uint16_t retryCount;
    uint16_t sensorCount;

    std::vector<uint16_t> hubReading;

    bool dataRequest;
    bool dataReady;

    std::mutex requestMutex;
    std::mutex dataMutex;
    std::condition_variable signal;
};

} //namespace

