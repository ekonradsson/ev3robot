#include <ros/ros.h>

#include <iostream>
#include <cassert>

#include <math.h>
#include <stdio.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/PointCloud.h>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <dynamic_reconfigure/server.h>

#include <std_msgs/String.h>
#include <serial/serial.h>
#include <boost/thread/thread.hpp>

#include <ev3/ev3ParamsConfig.h>

#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"

#include "config.h"
#include "utils.h"
#include "driveCtrl.h"
#include "sensorHub.h"
#include "bno055.h"
#include "motor.h"
#include "pid.h"

const std::string imuPort = "/dev/tty_in4";
const uint32_t imuBaud = 115200;

const std::string hubPort = "/dev/tty_in1";
const uint32_t hubBaud = 57600;
const uint16_t hubTimeout = 10;

const std::string baseFrame = "/base_footprint";

const int rcStickAil = 0;
const int rcSwitchFlap = 1;
const int rcStickEle = 2;
const int rcSwitchGear = 3;
const int rcStickRud = 4;
const int rcStickThr = 6;

const int irFrontRight = 1;
const int irRearRight = 2;
const int irRearLeft = 7;
const int irFrontLeft = 8;

const int digitalInputs = 0;

int imuReadRate = 20;
int hubReadRate = 30;
int driveCtrlRate = 15;
int odomPubRate = 5;
int statusPubRate = 15;

double maxRot = 3.0;
double maxVel = 0.3;
double rollOffset = 0.0244751;
double pitchOffset = 0.0154649;
double edgeLimit = 100;
double obstacleLimit = 1023;

int rcVelocity = rcStickEle;
int rcRotation = rcStickAil;
int rcTilt = rcStickThr;

bool ESTOP = false;

ros::NodeHandle *rosNode;

ev3ros::driveCtrl *drive = NULL;
ev3ros::bno055 *imu = NULL;
ev3ros::sensorHub *hub = NULL;

ros::Publisher odomPub;
ros::Publisher statusPub;
ros::Publisher calibPub;

ros::Timer odomPubTimer;
ros::Timer statusPubTimer;

geometry_msgs::TransformStamped odom_trans;
nav_msgs::Odometry odom;

double linTarget, rotTarget;

bool initialized = false;

void setVel(double linear, double angular)
{
    static std::vector<uint16_t> hubReadings;

    if (fabs(linear)<0.02) linear = 0;
    if (fabs(angular)<0.1) angular = 0;

    if (hub->getValues(0, ev3ros::sensorCount, hubReadings))
    {
        if (ESTOP && (linear==0))
        {
            ESTOP=false;
        }

        if (hubReadings[irFrontLeft]<edgeLimit)
        {
            if (linear>0)
            {
                ESTOP = true;
            }
        }

        if (hubReadings[irFrontRight]<edgeLimit)
        {
            if (linear>0)
            {
                ESTOP = true;
            }
        }

        if (hubReadings[irRearLeft]<edgeLimit)
        {
            if (linear<0)
            {
                ESTOP = true;
            }
        }

        if (hubReadings[irRearRight]<edgeLimit)
        {
            if (linear<0)
            {
                ESTOP = true;
            }
        }

        if (ESTOP)
        {
            linear = 0;
        }

        drive->setVel(linear, angular, !ESTOP);
    } else
    {
        drive->setVel(0, 0, false);
    }
}

void configCallback(ev3::ev3ParamsConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure request, level %d", level);

    if ((level & 64) || (level==-1))
    {
        if ((imuReadRate!=config.IMUReadRate) && (config.IMUReadRate>0))
        {
            imuReadRate = config.IMUReadRate;
            if (imu!=NULL)
            {
                imu->setRate(imuReadRate);
            }
        }

        if ((driveCtrlRate!=config.DriveCtrlRate) && (config.DriveCtrlRate>0))
        {
            driveCtrlRate = config.DriveCtrlRate;
            if (drive!=NULL)
            {
                drive->setRate(driveCtrlRate);
            }
        }

        if ((hubReadRate!=config.HubReadRate) && (config.HubReadRate>0))
        {
            hubReadRate = config.HubReadRate;
            if (hub!=NULL)
            {
                hub->setRate(hubReadRate);
            }
        }

        if ((odomPubRate!=config.OdomPubRate) && (config.OdomPubRate>0))
        {
            odomPubRate = config.OdomPubRate;
            ROS_INFO("odomPubRate %d", odomPubRate);
            odomPubTimer.setPeriod(ros::Duration((float)1/odomPubRate));
        }

        if ((statusPubRate!=config.StatusPubRate) && (config.StatusPubRate>0))
        {
            statusPubRate = config.StatusPubRate;
            ROS_INFO("statusPubRate %d", statusPubRate);
            statusPubTimer.setPeriod(ros::Duration((float)1/statusPubRate));
        }
    }

    if ((level == 0) || (level==-1))
    {
        rollOffset = config.RollOffset;
        pitchOffset = config.PitchOffset;

        edgeLimit = config.EdgeLimit;
        obstacleLimit = config.ObstacleLimit;

        ROS_INFO("Offsets roll %.4f, pitch %.4f", rollOffset, pitchOffset);
    }

    if ((level & 4) || (level==-1))
    {
        if (imu!=NULL)
        {
            imu->config(config.IMUTimeout, config.IMURetryCount, config.IMURetrySleep, config.IMUAckSleep);
        }

        if (hub!=NULL)
        {
            hub->config(config.HubTimeout);
        }
    }

    if ((level & 2) || (level==-1))
    {
        if (drive!=NULL)
        {
            maxRot = config.MaxRotVel;
            maxVel = config.MaxLinVel;

            drive->configure
                    (
                        config.TrackWidth, config.WheelDiam, config.GearRatio,
                        config.MaxLinVel, config.MaxRotVel, config.LinAccel,
                        config.RotAccel, config.CmdTimeout
                        );

        }
    }

    if ((level & 8) || (level==-1))
    {
        if (drive!=NULL)
        {
            drive->setVelPID(config.VelKp, config.VelKi, config.VelKd);
            drive->setRotPID(config.RotKp, config.RotKi, config.RotKd);
            drive->setHoldPID(config.HoldKp, config.HoldKi, config.HoldKd);
        }
    }
}

void statusPubCallback(const ros::TimerEvent&)
{
    double qx, qy, qz, qw;
    double ax, ay, az, am;
    double gx, gy, gz;
    double roll, pitch, yaw;
    double lVel, rVel;

    static uint16_t statusPubCycle = 0;
    static std_msgs::Float32MultiArray msgStatus;
    static std_msgs::ByteMultiArray msgCalib;
    static std::vector<uint16_t> hubReadings;
    static std::vector<double> input, inputNorm, resultVals, resultNorm;
    static int lastState = -1;

    if (hub->getValues(0, ev3ros::inputCount, hubReadings))
    {
        if (drive->getVel(lVel, rVel) && imu->getQuat(qx, qy, qz, qw) && imu->getGyro(gx, gy, gz) && imu->getMaxAccel(ax, ay , az))
        {
            tf::Quaternion q(-qx, -qy, qz, qw);
            tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

            msgStatus.data.clear();
            for (int i=0;i<ev3ros::sensorCount + ev3ros::sonarCount;++i)
            {
                msgStatus.data.push_back(hubReadings[i]);
            }
            roll -= rollOffset;
            pitch -= pitchOffset;

            msgStatus.data.push_back(roll);
            msgStatus.data.push_back(pitch);
            msgStatus.data.push_back(yaw);
            msgStatus.data.push_back(gx);
            msgStatus.data.push_back(gy);
            msgStatus.data.push_back(gz);
            msgStatus.data.push_back(ax);
            msgStatus.data.push_back(ay);
            msgStatus.data.push_back(az);
            msgStatus.data.push_back(lVel);
            msgStatus.data.push_back(rVel);

            statusPub.publish(msgStatus);
        }
    }

    if (statusPubCycle == 0)
    {

        uint8_t calMag=0, calAcc=0, calGyr=0, calSys=0;
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
    statusPubCycle++;
    if (statusPubCycle>statusPubRate-1)
    {
        statusPubCycle = 0;
    }
}

void odomPubCallback(const ros::TimerEvent&)
{
    double lPosNew, rPosNew, rotNew;
    double qx, qy, qz, qw;
    double yawRate = 0;

    static double xPos = 0, yPos = 0;
    static double lPos = 0, rPos = 0;

    static tf::TransformBroadcaster odomBroadcaster;
    static geometry_msgs::Quaternion odom_quat;

    if ((drive->getPose(lPosNew, rPosNew, rotNew) && imu->getQuat(qx, qy, qz, qw) && imu->getYawRate(yawRate)))
    {
        double dl, dr, displ;
        double dx, dy;
        double vx, vy;
        double dt = 1/odomPubRate;
        double roll, pitch, yaw;

        tf::Quaternion q(-qx, -qy, qz, qw);
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        yaw = atan2(sin(yaw), cos(yaw)); //make sure heading is within [-Pi, Pi]

        //Publish odom transformation

        dl = lPosNew - lPos;
        dr = rPosNew - rPos;
        lPos = lPosNew;
        rPos = rPosNew;

        displ = (dl+dr)/2;

        dx = displ * cos(yaw);
        dy = displ * sin(yaw);

        xPos +=  dx;
        yPos +=  dy;

        vx = dx / dt;
        vy = dy / dt;

        //      ROS_INFO("Odom left=%.3f right=%.3f rot=%.4f heading=%.4f, rate=%.4f", lPosNew, rPosNew, rotNew, heading, vz );

        //Odom transform
        odom_quat = tf::createQuaternionMsgFromYaw(yaw);

        odom_quat.x = qx;
        odom_quat.y = qy;
        odom_quat.z = qz;
        odom_quat.w = qw;

        odom_trans.header.stamp = ros::Time::now();

        odom_trans.transform.translation.x = xPos;
        odom_trans.transform.translation.y = yPos;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odomBroadcaster.sendTransform(odom_trans);

        //Publish odom data
        odom.header.stamp = ros::Time::now();

        odom.pose.pose.position.x = xPos;
        odom.pose.pose.position.y = yPos;
        odom.pose.pose.position.z = 0.00;
        odom.pose.pose.orientation = odom_quat;

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = yawRate;

        odomPub.publish(odom);
    }
}

void velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    setVel(cmd_vel->linear.x, cmd_vel->angular.z);
}

void rcCallback(const std_msgs::Int16MultiArray::ConstPtr& cmd_rc)
{
    setVel(scale(cmd_rc->data[rcVelocity], 306, 1740, -maxVel, maxVel), scale(cmd_rc->data[rcRotation], 306, 1740, -maxRot, maxRot));
}

int main(int argc, char** argv)
{
    uint8_t calMag=0, calAcc=0, calGyr=0, calSys=0;
    initialized = false;

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = baseFrame;
    odom.header.frame_id = "odom";
    odom.child_frame_id = baseFrame;

    ros::init(argc, argv, "ev3");
    rosNode = new ros::NodeHandle();

    ROS_INFO( "EV3 node starting..." );

    if (!(ev3_init() && ev3_tacho_init() && ev3_sensor_init()))
    {
        return -1;
    }

    ROS_INFO( "EV3 setting up imu..." );
    imu = new ev3ros::bno055(rosNode, imuReadRate);
    ROS_INFO( "EV3 setting up sensor hub..." );
    hub = new ev3ros::sensorHub(rosNode, hubReadRate);
    ROS_INFO( "EV3 setting up drive control..." );
    drive = new ev3ros::driveCtrl(rosNode, imu, driveCtrlRate);

    if (hub->init(hubPort, hubBaud) && drive->init() && imu->init(imuPort, imuBaud))
    {
        ROS_INFO( "EV3 setting up publishers..." );
        odomPub = rosNode->advertise<nav_msgs::Odometry>("/odom", 10);
        calibPub = rosNode->advertise<std_msgs::ByteMultiArray>("/imu/calib", 5);
        statusPub = rosNode->advertise<std_msgs::Float32MultiArray >("/status", 5);

        ROS_INFO( "EV3 setting up timers..." );
        odomPubTimer = rosNode->createTimer(ros::Duration((float)1/odomPubRate), odomPubCallback);
        statusPubTimer = rosNode->createTimer(ros::Duration((float)1/statusPubRate), statusPubCallback);

        ROS_INFO( "EV3 setting up subscribers..." );
        ros::Subscriber vel_sub = rosNode->subscribe<geometry_msgs::Twist>("/cmd_vel", 1, velCallback);
        ros::Subscriber rc_sub = rosNode->subscribe<std_msgs::Int16MultiArray>("/rc", 1, rcCallback);


        // Set up dynamic reconfigure
        ROS_INFO( "EV3 setting up dynamic reconfigure..." );
        dynamic_reconfigure::Server<ev3::ev3ParamsConfig> srv;
        dynamic_reconfigure::Server<ev3::ev3ParamsConfig>::CallbackType f;

        f = boost::bind(configCallback, _1, _2);
        srv.setCallback(f);

        uint8_t calStatus;

        while (!initialized)
        {
            calStatus = imu->getCalStatus();
            if (!imu->isCalSaved())
            {
                if (calStatus < 0x3c)
                {
                    imu->getCalib( calMag, calAcc, calGyr, calSys);
                    ROS_INFO("IMU calibration in progress (%d), mag %d, acc %d, gyr %d, sys %d", calStatus, calMag, calAcc, calGyr, calSys);
                } else if (imu->saveOffsets())
                {
                    initialized = true;
                    ROS_INFO("IMU calibration done");
                }
            } else
            {
                /*
                if (calStatus < 0x3c)
                {
                    imu->getCalib( calMag, calAcc, calGyr, calSys);
                    ROS_INFO("IMU not fully calibrated (%d), mag %d, acc %d, gyr %d, sys %d", calStatus, calMag, calAcc, calGyr, calSys);
                } else
                {
                    initialized = true;
                    ROS_INFO("IMU ready");
                }
                */
                initialized = true;
            }
            Sleep(100);
        }

        ROS_INFO( "EV3 ready" );

        ros::spin();
    } else
    {
        ROS_ERROR( "EV3 Node failed to initialize" );
    }

    ROS_INFO( "EV3 Node Exiting" );

    delete hub;
    delete imu;
    delete drive;

    ev3_uninit();
    return ( 0 );
}
