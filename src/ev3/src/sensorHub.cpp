#include <iostream>
#include <cassert>
#include <cmath>
#include <stdio.h>

#include "ev3.h"
#include "ev3_port.h"
#include "sensorHub.h"
#include "utils.h"

namespace ev3ros {

sensorHub::sensorHub(const uint16_t _timeout)
    :ser()
{
    updateThread = NULL;
    timeout = _timeout;
    stopRequest.store(false);
    retryCount = 3;
    sensorCount = 9;
}

sensorHub::~sensorHub()
{
    stopThread();

    if(ser.isOpen())
    {
        ser.close();
    }
    debug_print("sensorHub : terminated!\n");
}

void sensorHub::config(const uint16_t _timeout, uint16_t _retryCount)
{
    timeout = _timeout;
    retryCount = _retryCount;

    ser.setTimeout(serial::Timeout::max(), timeout, 0, timeout, 0);
}

bool sensorHub::init(const std::string _port, const uint32_t _baud)
{
    port = _port;
    baud = _baud;

    ser.setBaudrate(baud);
    ser.setPort(port);
    ser.setBytesize(serial::eightbits);
    ser.setFlowcontrol(serial::flowcontrol_none);
    ser.setStopbits(serial::stopbits_one);
    ser.setTimeout(serial::Timeout::max(), timeout, 0, timeout, 0);

    ser.open();

    if(!ser.isOpen())
    {
        printf("sensorHub : serialport not open!\n");
        return false;
    }

    ser.flush(); //flush io buffers
    startThread();

    return true;
}

bool sensorHub::start()
{
    startThread();
    debug_print("sensorHub : started!\n");
    return true;
}

bool sensorHub::readHub(std::vector<uint16_t> *data)
{
    static std::vector<uint8_t>out_data;
    static std::vector<uint8_t>in_data;

    int bytes;
    bool valid = false;

    int retries = retryCount;

    while (retries>=0)
    {
        out_data.clear();
        out_data.push_back(0x23);
        out_data.push_back(0x0);
        out_data.push_back(sensorCount);

        ser.flushInput();
        ser.write(out_data);
        ser.flushOutput();

        in_data.clear();

        bytes = ser.read(in_data,(sensorCount*2)+2);

        data->clear();

        if (bytes>1)
        {
            if ((in_data[0]==0x23) && (in_data[1]==(sensorCount*2)))
            {
                for (int i=0; i<sensorCount; ++i)
                {
                    data->push_back((in_data[(i*2)+2]<<8|in_data[(i*2)+3]));
                }
                return true;
            } else
            {
                debug_print("SensorHub : read error, bytes %d, header %d, size %d\n", bytes, in_data[0], in_data[1]);
            }
        } else
        {
            debug_print("SensorHub : read timeout\n");
        }
        retries--;
    }
    printf("SensorHub : Failed to read data from hub after %d retries\n", retryCount);
    return false;
}

void sensorHub::stopThread()
{
    if (updateThread!=NULL)
    {
        debug_print("SensorHub : stoping...\n");
        stopRequest.store(true);
        reqestReading();
        updateThread->join();
        delete updateThread;
        updateThread = NULL;
        stopRequest.store(false);
        debug_print("SensorHub : stoped\n");
    }
}

void sensorHub::startThread()
{
    if (updateThread==NULL)
    {
        updateThread = new std::thread(&sensorHub::threadTask, this);
        assert(updateThread!=NULL);
        debug_print("SensorHub : runing\n");
    }
}

void sensorHub::reqestReading()
{
    { // scope for lock
        std::lock_guard<std::mutex> lock(requestMutex);
        dataRequest = true;
    }
    signal.notify_one();
}

bool sensorHub::getReading(int timeout, std::vector<uint16_t> &values)
{
    std::unique_lock<std::mutex> lock(dataMutex);
    if (signal.wait_for(lock, std::chrono::milliseconds(timeout), [this]{return dataReady;}))
    {
        dataReady = false;

        values.clear();
        for (int i=0;i<hubReading.size();++i)
        {
            values.push_back(hubReading[i]);
        }
        return true;
    }
    return false;
}

void sensorHub::threadTask()
{
    bool newData = false;

    debug_print("SensorHub : task started\n");

    while(!stopRequest.load())
    {
        { // Wait for load request
            std::unique_lock<std::mutex> lock(requestMutex);
            signal.wait(lock, [this]{return dataRequest;});
        }
        dataRequest = false;

        if (!stopRequest.load())
        {
            debug_print("sensorHub : load reqest received");

            { // load and send signal
                std::lock_guard<std::mutex> lock(dataMutex);
                newData = readHub(&hubReading);
                if (newData)
                {
                    dataReady = true;
                } else
                {
                    dataReady = false;
                }
            }

            if (newData)
            {
                newData = false;
                signal.notify_one();
            }
        }
    }
    debug_print("SensorHub : task ended\n");
}

} //namespace


