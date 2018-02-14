#include <iostream>
#include <cassert>
#include <chrono>

#include <fstream>

#include "bno055.h"
#include "utils.h"

namespace ev3ros {

#define BNO055_ID (0xA0)
#define DEFAULT_TIMEOUT 6
#define INIT_TIMEOUT 1000
#define CYCLE_LIMIT 100
#define SET_OFFSETS_TIME 2000

#define INIT_ACKSLEEP 100

bno055::bno055()
    :serial_()
{
    updateThread = NULL;
    retryCount = 3;
    timeout = DEFAULT_TIMEOUT;
    regCount = 6;
    mode = OP_MODE_IMU;
    calSaved = false;
    stopRequest.store(false);
}

bno055::~bno055()
{
    stopThread();

    if(serial_.isOpen())
    {
        serial_.close();
    }
}

void bno055::config(uint16_t _regCount, uint16_t _timeout, uint16_t _retries)
{
    regCount = _regCount;
    timeout = _timeout;
    retryCount = _retries;

    serial_.setTimeout(serial::Timeout::max(), timeout, 0, timeout, 0);
}

bool bno055::init(const std::string port, uint32_t baudrate)
{
    serial_.setBaudrate(baudrate);
    serial_.setPort(port);
    serial_.setBytesize(serial::eightbits);
    serial_.setFlowcontrol(serial::flowcontrol_none);
    serial_.setStopbits(serial::stopbits_one);

    serial_.setTimeout(serial::Timeout::max(), INIT_TIMEOUT, 0, INIT_TIMEOUT, 0);

    serial_.open();

    if(!serial_.isOpen())
    {
        printf("bno055 : serialport not open!\n");
        return false;
    }

    serial_.flush(); //flush io buffers

    if (!reset())
    {
        return false;
    }

    serial_.flush(); //flush io buffers

    while (readByte(BNO055_CHIP_ID) != BNO055_ID)
    {
        Sleep(INIT_ACKSLEEP);
    }

    if (!setMode(OP_MODE_CONFIG))
    {
        return false;
    }

    if (!setPage(0))
    {
        return false;
    }

    calSaved = loadOffsets();

    if (calSaved)
    {
        setOffsets();
    }

    if(!writeByte(BNO055_PWR_MODE,PWR_MODE_NORMAL))
    {
        printf("bno055: Failed to set power mode\n");
        return false;
    }

    if(!writeByte(BNO055_UNIT_SEL,0x83))
    {
        printf("bno055 : Failed to set units\n");
        return false;
    }

    if(!writeByte(BNO055_AXIS_MAP_CONFIG,REMAP_CONFIG_P0))
    {
        printf("bno055 : Failed to remap axis\n");
        return false;
    }

    if(!writeByte(BNO055_AXIS_MAP_SIGN,REMAP_SIGN_P0))
    {
        printf("bno055 : Failed to set axis sign\n");
        return false;
    }

    if (!setMode(mode))
    {
        return false;
    }

    serial_.setTimeout(serial::Timeout::max(), timeout, 0, timeout, 0);

    startThread();

    return true;
}

bool bno055::reset()
{
    bool valid = writeByte(BNO055_SYS_TRIGGER, 0x20, false);
    Sleep(600);
    if (!valid)
    {
        printf("bno055 : failed to reset!\n");
    }
    return valid;
}

bool bno055::setMode(uint8_t mode)
{
    if(!writeByte(BNO055_OPR_MODE, mode))
    {
        printf("bno055 : Failed to set mode %d\n", (int)mode);
        return false;
    } else
    {
        return true;
    }
}

void bno055::getStatus(uint8_t &mag, uint8_t &acc, uint8_t &gyr, uint8_t &sys)
{
    uint8_t status = calStatus.load();
    mag = (status & 0x3);//MAG
    acc = (status & 0x3<<2)>>2;//ACC 0x0c
    gyr = (status & 0x3<<4)>>4;//GYR 0x30
    sys = (status & 0x3<<6)>>6;//SYS
}

uint8_t bno055::getStatus()
{
    return calStatus.load();
}

bool bno055::isCalibrated()
{
    uint8_t status = getStatus();
    return (status==0xff);
}

void bno055::printDiag()
{
    ImuDiagnostics diag_data;

    bool error = !read(BNO055_ST_RESULT,(uint8_t*)&diag_data,5);

    if (error)
    {
        printf("bno055 : Failed to read diagnostics");
    } else {
        printf("bno055 : test %d, status %d, error %d,  int %d, clock %d",
            diag_data.test_result, diag_data.status, diag_data.error, diag_data.interupt, diag_data.clock);
    }
}
void bno055::printOffsets()
{
    std::cout << "Offsets:\n";
    std::cout << "Accel ";
    for(int i=0;i<3;++i)
    {
        std::cout << offsets.Accel[i] << " ";
    }
    std::cout << "\nMag ";
    for(int i=0;i<3;++i)
    {
        std::cout << offsets.Mag[i] << " ";
    }
    std::cout << "\nGyro ";
    for(int i=0;i<3;++i)
    {
        std::cout << offsets.Gyro[i] << " ";
    }
    std::cout << "\nAccel Rad " << offsets.AccelRad;
    std::cout << "\nMag Rad " << offsets.MagRad;
    std::cout << "\n";
}

bool bno055::getOffsets()
{
    if (setPage(0))
    {
        bool error = !read(BNO055_ACC_OFFSET_X_LSB, (uint8_t*)&offsets, sizeof(OffsetData));
        if(error)
        {
            printf("bno055 : Failed to get offsets\n");
        }

        return !error;
    } else
    {
        printf("bno055 : Failed to get offsets\n");
        return false;
    }
}

bool bno055::setOffsets()
{
    if (setPage(0))
    {
        if (!write(BNO055_ACC_OFFSET_X_LSB, (uint8_t*)&offsets.Accel, 6))
        {
            printf("bno055 : Failed to write ACC_OFFSET\n");
        }

        if (!write(BNO055_MAG_OFFSET_X_LSB, (uint8_t*)&offsets.Mag, 6))
        {
            printf("bno055 : Failed to write MAG_OFFSET\n");
        }

        if (!write(BNO055_GYR_OFFSET_X_LSB, (uint8_t*)&offsets.Gyro, 6))
        {
            printf("bno055 : Failed to write GYR_OFFSET\n");
        }

        if (!write(BNO055_ACC_RADIUS_LSB, (uint8_t*)&offsets.AccelRad, 2))
        {
            printf("bno055 : Failed to write ACC_RADIUS\n");
        }

        if (!write(BNO055_MAG_RADIUS_LSB, (uint8_t*)&offsets.MagRad, 2))
        {
            printf("bno055 : Failed to write MAG_RADIUS\n");
        }
        return true;
    } else
    {
        printf("bno055 : Failed to set offsets\n");
        return false;
    }
}

bool bno055::clearOffsets()
{

    for (int i=0;i<3;++i)
    {
        offsets.Accel[i]=0;
        offsets.Gyro[i]=0;
        offsets.Mag[i]=0;
    }
    offsets.MagRad = 0;
    offsets.AccelRad = 0;

    bool valid = setOffsets();

    return valid;
}

bool bno055::saveOffsets()
{
    if (setMode(OP_MODE_CONFIG))
    {
        serial_.setTimeout(serial::Timeout::max(), INIT_TIMEOUT, 0, INIT_TIMEOUT, 0);

        if (getOffsets())
        {
            std::ofstream file("/home/robot/.bno055.cal", std::ios::out | std::ios::binary);
            if (file.is_open())
            {
                file.write(reinterpret_cast<char*>(&offsets), sizeof(OffsetData));
                file.close();
                return true;
            } else
            {
                printf("bno055 : Could not save offsets\n");
                return false;
            }
        }

        serial_.setTimeout(serial::Timeout::max(), timeout, 0, timeout, 0);

        setMode(mode);
    }
    return calSaved;
}

bool bno055::isCalSaved()
{
    return calSaved;
}

bool bno055::loadOffsets()
{
    std::ifstream file("/home/robot/.bno055.cal", std::ios::in | std::ios::binary);
    if (file.is_open())
    {
        file.read(reinterpret_cast<char*>(&offsets), sizeof(OffsetData));
        file.close();
        return true;
    } else
    {
        return false;
    }
}

bool bno055::setPage(uint8_t id)
{
    if(!writeByte(PAGE_ID, id, INIT_ACKSLEEP))
    {
        printf("bno055 : Failed to set register page %d\n", id);
        return false;
    } else
    {
        return true;
    }
}

bool bno055::writeByte(BNO055Register reg, uint8_t byte)
{
    return write(reg,&byte,1,true);
}

bool bno055::writeByte(BNO055Register reg, uint8_t byte, bool ack)
{
    return write(reg,&byte,1,ack);
}

bool bno055::write(BNO055Register reg, const void *data, unsigned length)
{
    return write(reg,data,length,true);
}

bool bno055::write(BNO055Register reg, const void *data, unsigned length, bool ack)
{
    int8_t retries = retryCount;
    uint8_t bytes;

    static uint8_t in_data[2]={0,0};
    static std::vector<uint8_t>out_data;

    while(retries>=0)
    {
        out_data.clear();

        out_data.push_back(0xAA);
        out_data.push_back(0); //WRITE
        out_data.push_back((uint8_t)reg);
        out_data.push_back(length);
        for (int b = 0; b < length; ++b)
        {
            out_data.push_back(*((uint8_t*)data+b));
        }

        serial_.flushInput();
        bytes = serial_.write(out_data);
        serial_.flushOutput();

        //    debug_print("IMU: wrote %d bytes, expected %d, data size %d", bytes, out_data.size(), length);

        if (!ack)
        {
            return true;
        } else
        {
            if(serial_.read(in_data,2)==2)
            {
                if ((in_data[0]==0xEE && in_data[1]==0x01))
                {
                    return true;
                } else
                {
                    if (in_data[1]==0x07)
                    {
                        retries--;
                    } else
                    {
                        debug_print("bno055 : write error %02x %02x\n", in_data[0], in_data[1]);
                        retries = 0;
                    }
                }
            } else
            {
                debug_print("bno055 : write timeout");
                retries = 0;
            }
        }
    }
    return false;
}

uint8_t bno055::readByte(BNO055Register reg)
{
    uint8_t in_data = 0;
    read(reg, &in_data, 1);
    return in_data;
}

bool bno055::read(BNO055Register reg, uint8_t *data, unsigned length)
{
    static std::vector<uint8_t>out_data;
    static std::vector<uint8_t>in_data;

    int retries = retryCount;
    size_t bytes;

    while(retries>=0)
    {
        out_data.clear();
        out_data.push_back(0xAA);
        out_data.push_back(1); //READ
        out_data.push_back((uint8_t)reg);
        out_data.push_back(length);
        serial_.write(out_data);
        serial_.flushOutput();

        serial_.flushInput();
        in_data.clear();

        bytes = serial_.read(in_data,2+length);

        if (bytes>1)
        {
            if (in_data[0]==0xBB && in_data[1]==length)
            {
                memcpy(data,in_data.data()+2,length);
                #ifdef DEBUG_IMU
                    if (retries!=retryCount)
                    {
                        debug_print("bno055 : Succeeded after %d retries", retryCount-retries);
                    }
                #endif
                return true;
            } else
            {
                #ifdef DEBUG_IMU
                    if (in_data[0]==0xee)
                    {
                        if (in_data[1]!=0x07)
                        {
                            debug_print("bno055 : read error. reg=%02x %02x code=%02x", reg, in_data[0], in_data[1]);
                        }
                    } else
                    {
                        debug_print("bno055 : unknown response. reg=%02x %02x code=%02x", reg, in_data[0], in_data[1]);
                    }
                #endif
            }
        } else
        {
            #ifdef DEBUG_IMU
                debug_print("bno055 : read timeout");
            #endif
        }

        retries--;

        if (retries<=0)
        {
            printf("bno055 : Failed to read data after %d retries\n", retryCount);
        }
    }
    return false;
}

bool bno055::readIMU(ImuData *data)
{
    return read(BNO055_GYR_DATA_X_LSB,(uint8_t*)data,sizeof(int16_t)*regCount);
}

void bno055::stopThread()
{
    if (updateThread!=NULL)
    {
        stopRequest.store(true);

        reqestReading();

        updateThread->join();
        delete updateThread;
        updateThread = NULL;
        stopRequest.store(false);
    }
}

void bno055::startThread()
{
    if (updateThread==NULL)
    {
        updateThread = new std::thread(&bno055::threadTask, this);
        assert(updateThread!=NULL);
    }
}

void bno055::reqestReading()
{
    { // scope for lock
        std::lock_guard<std::mutex> lock(requestMutex);
        dataRequest = true;
    }
    signal.notify_one();
}

bool bno055::getReading(int timeout, std::vector<double> &values)
{
    std::unique_lock<std::mutex> lock(dataMutex);
    if (signal.wait_for(lock, std::chrono::milliseconds(timeout), [this]{return dataReady;}))
    {
        dataReady = false;

        values.clear();
        values.push_back(imuReading.GyroX*gyroScale);
        values.push_back(imuReading.GyroY*gyroScale);
        values.push_back(imuReading.GyroZ*gyroScale);

        values.push_back(imuReading.Roll*rpyScaleRad);
        values.push_back(imuReading.Pitch*rpyScaleRad);
        values.push_back(imuReading.Yaw*rpyScaleRad);

        values.push_back(imuReading.QuatX*quatScale);
        values.push_back(imuReading.QuatY*quatScale);
        values.push_back(imuReading.QuatZ*quatScale);
        values.push_back(imuReading.QuatW*quatScale);

        values.push_back(imuReading.AccelX*accelScale);
        values.push_back(imuReading.AccelY*accelScale);
        values.push_back(imuReading.AccelZ*accelScale);

        return true;
    }
    return false;
}

void bno055::threadTask()
{
    bool newData = false;

    if(!serial_.isOpen())
    {
        printf("bno055 : serialport not open!\n");
        exit(1);
    }

    while(!stopRequest.load())
    {
        { // Wait for load request
            std::unique_lock<std::mutex> lock(requestMutex);
            signal.wait(lock, [this]{return dataRequest;});
        }
        dataRequest = false;

        if (!stopRequest.load())
        {
            #ifdef DEBUG_IMU
                printf("bno055 : load reqest received");
            #endif

            { // load and send signal
                std::lock_guard<std::mutex> lock(dataMutex);
                newData = readIMU(&imuReading);
                if (newData)
                {
                    calStatus.store(imuReading.calib);
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
}


}//namespace

