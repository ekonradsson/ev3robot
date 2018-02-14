/*
 * bno055.h
 *
 *  Based on BoschBno055Uart by Christian Holl
 *
 */

#pragma once

#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include <serial/serial.h>

namespace ev3ros
{ 

typedef enum
{

    PAGE_ID=0x07,

    //PAGE 0
    BNO055_CHIP_ID=0,
    BNO055_ACC_ID,
    BNO055_MAG_ID,
    BNO055_GYR_ID,
    BNO055_SW_REV_ID_LSB,
    BNO055_SW_REV_ID_MSB,
    BNO055_BL_Rev_ID,
    BNO055_Page_ID,
    BNO055_ACC_DATA_X_LSB,
    BNO055_ACC_DATA_X_MSB,
    BNO055_ACC_DATA_Y_LSB,
    BNO055_ACC_DATA_Y_MSB,
    BNO055_ACC_DATA_Z_LSB,
    BNO055_ACC_DATA_Z_MSB,
    BNO055_MAG_DATA_X_LSB,
    BNO055_MAG_DATA_X_MSB,
    BNO055_MAG_DATA_Y_LSB,
    BNO055_MAG_DATA_Y_MSB,
    BNO055_MAG_DATA_Z_LSB,
    BNO055_MAG_DATA_Z_MSB,
    BNO055_GYR_DATA_X_LSB=0x14,
    BNO055_GYR_DATA_X_MSB,
    BNO055_GYR_DATA_Y_LSB,
    BNO055_GYR_DATA_Y_MSB,
    BNO055_GYR_DATA_Z_LSB,
    BNO055_GYR_DATA_Z_MSB,
    BNO055_EUL_Heading_LSB,
    BNO055_EUL_Heading_MSB,
    BNO055_EUL_Roll_LSB,
    BNO055_EUL_Roll_MSB,
    BNO055_EUL_Pitch_LSB,
    BNO055_EUL_Pitch_MSB,
    BNO055_QUA_Data_w_LSB,
    BNO055_QUA_Data_w_MSB,
    BNO055_QUA_Data_x_LSB,
    BNO055_QUA_Data_x_MSB,
    BNO055_QUA_Data_y_LSB,
    BNO055_QUA_Data_y_MSB,
    BNO055_QUA_Data_z_LSB,
    BNO055_QUA_Data_z_MSB,
    BNO055_LIA_Data_X_LSB,
    BNO055_LIA_Data_X_MSB,
    BNO055_LIA_Data_Y_LSB,
    BNO055_LIA_Data_Y_MSB,
    BNO055_LIA_Data_Z_LSB,
    BNO055_LIA_Data_Z_MBS,
    BNO055_GRVData_X_LSB,
    BNO055_GRV_Data_X_MSB,
    BNO055_GRV_Data_Y_LSB,
    BNO055_GRV_Data_Y_MSB,
    BNO055_GRV_Data_Z_LSB,
    BNO055_GRV_Data_Z_MSB,
    BNO055_TEMP,
    BNO055_CALIB_STAT,
    BNO055_ST_RESULT,
    BNO055_INT_STA,
    BNO055_SYS_CLK_STATUS,
    BNO055_SYS_STATUS,
    BNO055_SYS_ERR,
    BNO055_UNIT_SEL,
    BNO055_Reserved,
    BNO055_OPR_MODE,
    BNO055_PWR_MODE,
    BNO055_SYS_TRIGGER,
    BNO055_TEMP_SOURCE,
    BNO055_AXIS_MAP_CONFIG=0x41,
    BNO055_AXIS_MAP_SIGN=0x42,
    //RESERVED
    BNO055_ACC_OFFSET_X_LSB=0x55,
    BNO055_ACC_OFFSET_X_MSB,
    BNO055_ACC_OFFSET_Y_LSB,
    BNO055_ACC_OFFSET_Y_MSB,
    BNO055_ACC_OFFSET_Z_LSB,
    BNO055_ACC_OFFSET_Z_MSB,
    BNO055_MAG_OFFSET_X_LSB,
    BNO055_MAG_OFFSET_X_MSB,
    BNO055_MAG_OFFSET_Y_LSB,
    BNO055_MAG_OFFSET_Y_MSB,
    BNO055_MAG_OFFSET_Z_LSB,
    BNO055_MAG_OFFSET_Z_MSB,
    BNO055_GYR_OFFSET_X_LSB,
    BNO055_GYR_OFFSET_X_MSB,
    BNO055_GYR_OFFSET_Y_LSB,
    BNO055_GYR_OFFSET_Y_MSB,
    BNO055_GYR_OFFSET_Z_LSB,
    BNO055_GYR_OFFSET_Z_MSB,
    BNO055_ACC_RADIUS_LSB,
    BNO055_ACC_RADIUS_MSB,
    BNO055_MAG_RADIUS_LSB,
    BNO055_MAG_RADIUS_MSB,

    BNO055_P1_Page_ID=0x07,
    BNO055_ACC_Config,
    BNO055_MAG_Config,
    BNO055_GYR_Config_0,
    BNO055_GYR_Config_1,
    BNO055_ACC_Sleep_Config,
    BNO055_GYR_Sleep_Config,
    BNO055_P1_Reserved,
    BNO055_INT_MSK,
    BNO055_INT_EN,
    BNO055_ACC_AM_THRES,
    BNO055_ACC_INT_SET,
    BNO055_ACC_HG_DURATION,
    BNO055_ACC_HG_THRES,
    BNO055_ACC_NM_THRES,
    BNO055_ACC_NM_SET,
    BNO055_GYR_INT_SET,
    BNO055_GYR_HR_X_SET,
    BNO055_GYR_DUR_X,
    BNO055_GYR_HR_Y_SET,
    BNO055_GYR_DUR_Y,
    BNO055_GYR_HR_Z_SET,
    BNO055_GYR_DUR_Z,
    BNO055_GYR_AM_THRES,
    BNO055_GYR_AM_SET,
    //RESERVED
    BNO055_UNIQE_ID_B0=0x50,
    BNO055_UNIQE_ID_B1=0x51,
    BNO055_UNIQE_ID_B2=0x52,
    BNO055_UNIQE_ID_B3=0x53,
    BNO055_UNIQE_ID_B4=0x54,
    BNO055_UNIQE_ID_B5=0x55,
    BNO055_UNIQE_ID_B6=0x56,
    BNO055_UNIQE_ID_B7=0x57,
    BNO055_UNIQE_ID_B8=0x58,
    BNO055_UNIQE_ID_B9=0x59,
    BNO055_UNIQE_ID_BA=0x5A,
    BNO055_UNIQE_ID_BB=0x5B,
    BNO055_UNIQE_ID_BC=0x5C,
    BNO055_UNIQE_ID_BD=0x5D,
    BNO055_UNIQE_ID_BE=0x5E,
    BNO055_UNIQE_ID_BF=0x5F,
    //RESERVED

}BNO055Register;

typedef enum
{
    OP_MODE_CONFIG,
    OP_MODE_ACCONLY,
    OP_MODE_MAGONLY,
    OP_MODE_GYROONLY,
    OP_MODE_ACCMAG,
    OP_MODE_ACCGYRO,
    OP_MODE_MAGGYRO,
    OP_MODE_AMG,
    OP_MODE_IMU,
    OP_MODE_COMPASS,
    OP_MODE_M4G,
    OP_MODE_NDOF_FMC_OFF,
    OP_MODE_NDOF
}BNO055OpMode;

typedef enum
{
    PWR_MODE_NORMAL,
    PWR_MODE_LOW,
    PWR_MODE_SUSPEND
}BNO055PwrMode;

typedef enum
{
    REMAP_CONFIG_P0 = 0x21,
    REMAP_CONFIG_P1 = 0x24, // default
    REMAP_CONFIG_P2 = 0x24,
    REMAP_CONFIG_P3 = 0x21,
    REMAP_CONFIG_P4 = 0x24,
    REMAP_CONFIG_P5 = 0x21,
    REMAP_CONFIG_P6 = 0x21,
    REMAP_CONFIG_P7 = 0x24
} BNO055OAxisRemap;

typedef enum
{
    REMAP_SIGN_P0 = 0x04,
    REMAP_SIGN_P1 = 0x00, // default
    REMAP_SIGN_P2 = 0x06,
    REMAP_SIGN_P3 = 0x02,
    REMAP_SIGN_P4 = 0x03,
    REMAP_SIGN_P5 = 0x01,
    REMAP_SIGN_P6 = 0x07,
    REMAP_SIGN_P7 = 0x05
} BNO055OAxisRemapSign;

typedef struct
{
    int16_t GyroX; //0x14
    int16_t GyroY;
    int16_t GyroZ;
    int16_t Yaw;
    int16_t Roll;
    int16_t Pitch;
    int16_t QuatW;
    int16_t QuatX;
    int16_t QuatY;
    int16_t QuatZ;
    int16_t AccelX;
    int16_t AccelY;
    int16_t AccelZ;
    int16_t GravX;
    int16_t GravY;
    int16_t GravZ;
    int8_t temp;
    uint8_t calib;
}ImuData;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}ImuVector;

typedef struct
{
    int8_t test_result;
    int8_t interupt;
    int8_t clock;
    int8_t status;
    int8_t error;
}ImuDiagnostics;

typedef struct
{
    int16_t Accel[3];
    int16_t Mag[3];
    int16_t Gyro[3];
    int16_t AccelRad;
    int16_t MagRad;
}OffsetData;


typedef enum
{
    imuGyroX = 0,
    imuGyroY,
    imuGyroZ,
    imuRoll,
    imuPitch,
    imuYaw,
    imuQuatX,
    imuQuatY,
    imuQuatZ,
    imuQuatW,
    imuAccelX,
    imuAccelY,
    imuAccelZ,
} ImuFieldIndex;

class bno055
{
public:
    bno055();
    virtual ~bno055();

    bool init(const std::string port, uint32_t baudrate);
    void config(uint16_t regCount, uint16_t timeout, uint16_t retries);
    void reqestReading();
    bool getReading(int timeout, std::vector<double> &values);

    void getStatus(uint8_t &mag, uint8_t &acc, uint8_t &gyr, uint8_t &sys);
    uint8_t getStatus();

    void printDiag();
    
    bool isCalibrated();
    bool isCalSaved();
private:
    void threadTask();
    void stopThread();
    void startThread();

    bool readIMU(ImuData *data);

    bool reset();
    bool setMode(uint8_t mode);
    bool setPage(uint8_t id);

    bool loadOffsets();
    bool saveOffsets();
    bool clearOffsets();
    bool getOffsets();
    bool setOffsets();
    void printOffsets();

    ImuData imuReading;
    std::atomic<uint8_t> calStatus;

    std::thread *updateThread;
    std::mutex dataMutex;
    std::mutex requestMutex;

    std::atomic<bool> stopRequest;
    bool dataReady;
    bool dataRequest;

    std::condition_variable signal;

    serial::Serial serial_;
    
    OffsetData offsets;
    BNO055OpMode mode;

    uint16_t retryCount;
    uint16_t regCount;
    uint16_t timeout;
    bool calSaved;

    static constexpr double quatScale = (1.0 / (1<<14));
    static constexpr double rpyScaleDeg = 1.0/16.0;
    static constexpr double rpyScaleRad = 1.0/900.0;
    static constexpr double accelScale = 1/100.0;
    static constexpr double magScale = 1/16.0;
    static constexpr double gyroScale = 1/900.0;

    bool write(BNO055Register reg, const void *data, unsigned length, bool ack);
    bool write(BNO055Register reg, const void *data, unsigned length);

    bool writeByte(BNO055Register reg, uint8_t byte,  bool ack);
    bool writeByte(BNO055Register reg, uint8_t byte);

    bool read(BNO055Register reg, uint8_t *data, unsigned length);
    uint8_t readByte(BNO055Register reg);

};// class bno055

}// namespace
