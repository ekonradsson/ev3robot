#include <iostream>
#include <cassert>
#include <stdio.h>

#include "ev3.h"
#include "ev3_port.h"

#include "receiver.h"
#include "utils.h"

namespace ev3ros {

receiver::receiver()
    :ser()
{
    serialThread = NULL;
    timeout = 20;
    stopRequest.store(false);
}

receiver::~receiver()
{
    if (serialThread!=NULL)
    {
        stopRequest = true;
        serialThread->join();       
        delete serialThread;
        serialThread = NULL;
        stopRequest = false;
    }

    if(ser.isOpen())
    {
        ser.close();
    }

    debug_print("receiver : terminated\n");
}

bool receiver::init(const std::string _port, const uint32_t _baud)
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
        printf("receiver : Port not open!\n");
        return false;
    }

    ser.flush(); //flush io buffers

    serialThread = new std::thread(&receiver::threadTask, this);
    assert(serialThread!=NULL);

    debug_print("receiver : initialized");

    return true;
}

bool receiver::getValues(const uint8_t id, const uint8_t count, std::vector<uint16_t> &array)
{
    array.clear();
    mutex.lock();
    bool valid = false;
    if (values.size()==ev3ros::channelCount)
    {
        for (int i=0;i<count;++i)
        {
            assert(i+id>=0 && i+id<values.size());
            array.push_back(values[i+id]);
        }
        values.clear();
        valid = true;
    } else
    {
        for (int i=0;i<count;++i)
        {
            array.push_back(0);
        }
        valid = false;
    }
    mutex.unlock();

#ifdef DEBUG_RECEIVER
    debug_print("receiver::getValues(%d, %d), value size %d, array size %d, input count %d, valid %d\n", id, count, values.size(), array.size(), ev3ros::channelCount, valid);
#endif

    return (valid && array.size()==count);
}

void receiver::threadTask()
{
    int bytes;
    std::vector<uint8_t>in_data;

    debug_print( "receiver : Thread starting\n");

    ser.flushInput();

    while(!stopRequest.load())
    {
        in_data.clear();

        bytes = ser.read(in_data,1);

        if (bytes>=1)
        {
            if (in_data[0] == 0xA2)
            {
                bytes = ser.read(in_data,14);
                if (bytes>=14)
                {
                    if (mutex.try_lock())
                    {
                        values.clear();

                        for (int i = 0; i < channelCount; i++)
                        {
                            uint16_t value = (uint16_t)((in_data[(i * 2) + 2]) | ((uint16_t)(in_data[(i * 2) + 1] << 8)));
                            uint8_t id = value >> 11;
                            uint16_t pos = value & 0x07FF;
                            if (id > channelCount - 1)
                            {
                                debug_print("Receiver data error\n");
                            } else
                            {
                                values.push_back(pos);
                            }
                        }
                        mutex.unlock();

                        Sleep(15); // 50 hz, 20 ms between packets
                    }
                } else
                {
                    debug_print("receiver : read error, bytes %d, header %d, size %d\n", bytes, in_data[0], in_data[1]);
                }
            } else {
            }
        } else
        {
            mutex.lock();
            values.clear();
            mutex.unlock();
        }
    }
    debug_print( "receiver : thread exiting");
}

} //namespace


