#include <ros/ros.h>

#include <iostream>
#include <stdio.h>

#include <serial/serial.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/ByteMultiArray.h>

#include <tf/transform_broadcaster.h>

#include "ev3.h"
#include "ev3_port.h"

#include "utils.h"
#include "bno055.h"

const std::string imuPort = "/dev/tty_in2";
const uint32_t imuBaud = 115200;

int publishRate = 5;

ros::NodeHandle *rosNode;
ev3ros::bno055 *imu = NULL;
ros::Publisher imuPub;
ros::Publisher calibPub;

std::thread *imuThread = NULL;
std::atomic<bool> stopRequest(false);

void imuTask()
{
    uint8_t calMag=0, calAcc=0, calGyr=0, calSys=0;
    double ax, ay, az;
    double gx, gy, gz;
    double roll, pitch, yaw;

    uint16_t imuPubCycle = 0;
    std_msgs::Float32MultiArray msgImu;
    std_msgs::ByteMultiArray msgCalib;

    ros::Rate loop(publishRate);

    while (!stopRequest.load())
    {
        if (imu->getRPY(roll, pitch, yaw) && imu->getGyro(gx, gy, gz) && imu->getMaxAccel(ax, ay , az))
        {
            msgImu.data.clear();

            msgImu.data.push_back(roll);
            msgImu.data.push_back(pitch);
            msgImu.data.push_back(yaw);
            msgImu.data.push_back(gx);
            msgImu.data.push_back(gy);
            msgImu.data.push_back(gz);
            msgImu.data.push_back(ax);
            msgImu.data.push_back(ay);
            msgImu.data.push_back(az);
            imuPub.publish(msgImu);
        }

        if (imuPubCycle == 0)
        {
            if (imu->getCalib( calMag, calAcc, calGyr, calSys))
            {
                msgCalib.data.clear();
                msgCalib.data.push_back(calMag);
                msgCalib.data.push_back(calAcc);
                msgCalib.data.push_back(calGyr);
                msgCalib.data.push_back(calSys);

                calibPub.publish(msgCalib);
            }
        }
        imuPubCycle++;
        if (imuPubCycle>publishRate-1)
        {
            imuPubCycle = 0;
        }
        loop.sleep();
    }
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "imu");

    rosNode = new ros::NodeHandle();
    imu = new ev3ros::bno055(rosNode, 0);

    ROS_INFO( "imu node starting..." );

    if (!(ev3_init()))
    {
        return -1;
    }

    if (imu->init(imuPort, imuBaud))
    {
        calibPub = rosNode->advertise<std_msgs::ByteMultiArray>("/imu/calib", 1);
        imuPub = rosNode->advertise<std_msgs::Float32MultiArray >("/imu", 1);

        imuThread = new std::thread(&imuTask);

        ROS_INFO( "imu ready" );
        ros::spin();
    } else
    {
        ROS_ERROR( "imu node failed to initialize" );
    }
    ROS_INFO( "imu node exiting" );

    delete imu;

    ev3_uninit();
    return ( 0 );
}
