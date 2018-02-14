#include <iostream>
#include <cassert>

#include <math.h>
#include <stdio.h>
#include <signal.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <serial/serial.h>

#include <dynamic_reconfigure/server.h>
#include <ev3/ev3ParamsConfig.h>

#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"

#include "utils.h"
#include "driveCtrl.h"
#include "bno055.h"
#include "sensorHub.h"

const std::string baseFrame = "/base_footprint";

const int irFrontRight = 0;
const int irRearRight = 1;
const int irRearLeft = 2;
const int irFrontLeft = 3;
const int irTopRear = 4;
const int irTopFront = 5;

const int irSensors[] = {irFrontRight, irRearRight, irRearLeft, irFrontLeft, irTopRear, irTopFront};
const std::string irSensorName[] = {"Front Right", "Rear Right", "Rear Left", "Front Left", "Top Rear", "Top Front" };
const int irSensorCount = sizeof(irSensors)/sizeof(int);

const double cmdVelTimeout = 0.5;

int driveCtrlRate = 20;
ros::Time cmdVelTimeStamp;

int imuTimeout = 10;
int imuRetryCount = 5;
int imuRegCount = 6;

int hubTimeout = 10;
int hubRetryCount = 3;

ros::NodeHandle *rosNode;

ev3ros::bno055 *imu = NULL;
const std::string imuPort = "/dev/tty_in2";
const uint32_t imuBaud = 115200;

ev3ros::sensorHub *hub = NULL;
const std::string hubPort = "/dev/tty_in1";
const uint32_t hubBaud = 57600;

ev3ros::driveCtrl *drive = NULL;
ros::Publisher odomPub;
ros::Publisher hubPub;
ros::Subscriber vel_sub;

std::thread *navigatorThread = NULL;

std::atomic<bool> ESTOP(false);
std::atomic<bool> allClear(false);
std::atomic<bool> stopRequest(false);
std::atomic<double> linTarget;
std::atomic<double> rotTarget;
std::atomic<double> edgeThreshold(100);
std::atomic<double> obstacleThreshold(650);

std::mutex configMutex;

bool killed = false;

void sig_handler(int signo)
{
    if (signo == SIGINT)
    {
        printf("\nTerminating\n");
        killed = true;
    }
}

void velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    if (ESTOP.load() && fabs(cmd_vel->linear.x)==0)
    {
        if (allClear.load())
        {
            ESTOP.store(false);
            ROS_INFO( "ESTOP Reset" );
        }
    }

    linTarget.store(cmd_vel->linear.x);
    rotTarget.store(cmd_vel->angular.z);

    cmdVelTimeStamp = ros::Time::now();
}

void publishOdom(double roll, double pitch, double yaw, double yawRate, double dt)
{
    static double xPos = 0, yPos = 0;
    static double lastlPos = 0, lastrPos = 0;

    static tf::TransformBroadcaster odomBroadcaster;
    static geometry_msgs::Quaternion odom_quat;

    static geometry_msgs::TransformStamped odom_trans;
    static nav_msgs::Odometry odom;

    double dl, dr, displ;
    double dx, dy;
    double vx, vy;
    double lPos, rPos, driveYaw;

    double _yaw = atan2(sin(yaw), cos(yaw)); //make sure heading is within [-Pi, Pi]

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = baseFrame;
    odom.header.frame_id = "odom";
    odom.child_frame_id = baseFrame;

    //Publish odom transformation

    drive->getPose(lPos, rPos, driveYaw);

    dl = lPos - lastlPos;
    dr = rPos - lastrPos;
    lastlPos = lPos;
    lastrPos = rPos;

    displ = (dl+dr)/2;

    dx = displ * cos(_yaw);
    dy = displ * sin(_yaw);

    xPos +=  dx;
    yPos +=  dy;

    if (dt!=0)
    {
        vx = dx / dt;
        vy = dy / dt;
    } else
    {
        vx = 0;
        vy = 0;
    }

    //      ROS_INFO("Odom left=%.3f right=%.3f rot=%.4f heading=%.4f, rate=%.4f", lPosNew, rPosNew, rotNew, heading, vz );

    ros::Time now = ros::Time::now();

    //Odom transform

    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    //tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    //odom_quat = tf::createQuaternionMsgFromYaw(_yaw);
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, _yaw);

    odom_trans.header.stamp = now;

    odom_trans.transform.translation.x = xPos;
    odom_trans.transform.translation.y = yPos;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odomBroadcaster.sendTransform(odom_trans);

    //Publish odom data
    odom.header.stamp = now;

    odom.pose.pose.position.x = xPos;
    odom.pose.pose.position.y = yPos;
    odom.pose.pose.position.z = 0.00;
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = yawRate;

    odomPub.publish(odom);
}

void navigatorTask()
{
    double linear = 0;
    double angular = 0;
    double edge;
    double obstacle;
    int loopCount = 0;

    std_msgs::Int16MultiArray msgReadings;

    std::vector<double> imuReadings;
    std::vector<uint16_t> hubReadings;
    std::vector<uint16_t> hazzards;
    std::vector<uint16_t> sensorReadings;

    ros::Rate loop(driveCtrlRate);

    double dt = loop.expectedCycleTime().toSec();

    ROS_INFO("Navigator started");

    while (!stopRequest.load())
    {
        linear = linTarget.load();
        angular = rotTarget.load();
        edge = edgeThreshold.load();
        obstacle = obstacleThreshold.load();
        {
            std::unique_lock<std::mutex> lock(configMutex,std::defer_lock);

            if (lock.try_lock())
            {
                imu->reqestReading();
                hub->reqestReading();

                bool stop = ESTOP.load();

                if (hub->getReading(10, hubReadings))
                {
                    sensorReadings.clear();
                    for (int i=0;i<irSensorCount;++i)
                    {
                        if (irSensors[i]==irTopFront)
                        {
                            if (hubReadings[0] & 2)
                            {
                                sensorReadings.push_back(0);
                            } else
                            {
                                sensorReadings.push_back(edge);
                            }
                        } else if (irSensors[i]==irTopRear)
                        {
                            if (hubReadings[0] & 1)
                            {
                                sensorReadings.push_back(0);
                            } else
                            {
                                sensorReadings.push_back(edge);
                            }
                        } else
                        {
                            sensorReadings.push_back(hubReadings[i+1]);
                        }
                    }
                    if (stop)
                    {
                        linear = 0;
                        angular = 0;
                    }

                    for (int i = 0; i<irSensorCount; ++i)
                    {
                        int hazzardIndex = std::find(hazzards.begin(), hazzards.end(), irSensors[i]) - hazzards.begin();
                        if ((irSensors[i]==irTopFront) || (irSensors[i]==irTopRear))
                        {
                            if (sensorReadings[i]==0xffff)
                            {

                            }
                        }
                        if ((sensorReadings[i]<edge) || (sensorReadings[i]>obstacle) || (sensorReadings[i]==0xffff))
                        {
                            stop = true;
                            switch(irSensors[i])
                            {
                            case irFrontLeft:
                                if (linear>0) linear = 0;
                                if (angular>0) angular = 0;
                                break;

                            case irFrontRight:
                                if (linear>0) linear = 0;
                                if (angular<0) angular = 0;
                                break;
                            case irRearLeft:
                                if (linear<0) linear = 0;
                                if (angular<0) angular = 0;
                                break;
                            case irRearRight:
                                if (linear<0) linear = 0;
                                if (angular>0) angular = 0;
                                break;
                            case irTopFront:
                                if (linear>0) linear = 0;
                                break;
                            case irTopRear:
                                if (linear<0) linear = 0;
                                break;
                            }

                            if (hazzardIndex>=hazzards.size())
                            {
                                if (sensorReadings[i]==0xffff)
                                {
                                    ROS_ERROR("ev3 : sensor %s - disconnected", irSensorName[i].c_str());
                                    stop = true;
                                } else
                                {
                                    ROS_ERROR("ev3 : sensor %s - hazzard detected", irSensorName[i].c_str());
                                    stop = true;
                                }
                                hazzards.push_back(irSensors[i]);
                            }
                        } else
                        {
                            if (hazzardIndex<hazzards.size())
                            {
                                ROS_WARN("ev3 : sensor %s - warning cleared (%d)", irSensorName[i].c_str(), sensorReadings[i]);
                                hazzards.erase(hazzards.begin() + hazzardIndex);
                            }
                        }
                    }

                    allClear.store(hazzards.size()==0);

                    if (loopCount % 2 == 0) // publish every 2 cycles
                    {
                        //ROS_INFO("EV3 : Publish hub readings");

                        msgReadings.data.clear();
                        for (int i=0;i<sensorReadings.size();++i)
                        {
                            msgReadings.data.push_back(sensorReadings[i]);
                        }
                        hubPub.publish(msgReadings);
                    }
                } else
                {
                    if (!ESTOP.load())
                    {
                        drive->stop();
                        ESTOP.store(true);
                        linear = 0;
                        angular = 0;
                        linTarget.store(0);
                        rotTarget.store(0);

                        ROS_ERROR("EV3 : ESTOP - sensor hub timeout. Set requested velocity to zero to reset");
                    }
                }

                drive->setVel(linear, angular, !stop);

                if (imu->getReading(10, imuReadings))
                {
                    drive->updateMotors(imuReadings[ev3ros::imuYaw], imuReadings[ev3ros::imuGyroZ], dt);
                    if (loopCount % 2 == 0) // publish at half drive rate
                    {
                        //ROS_INFO("EV3 : Publish odometer");

                        publishOdom(
                            -imuReadings[ev3ros::imuPitch],
                            imuReadings[ev3ros::imuRoll],
                            imuReadings[ev3ros::imuYaw],
                            imuReadings[ev3ros::imuGyroZ], dt);
                    }
                } else
                {
                    if (!ESTOP.load())
                    {
                        drive->stop();
                        ESTOP.store(true);
                        linTarget.store(0);
                        rotTarget.store(0);

                        ROS_ERROR("EV3 : ESTOP - imu timeout. Set requested velocity to zero to reset");
                    }
                }

            } else {
                ROS_WARN("EV3 : config lock failed");
            }
        }
        loop.sleep();
        loopCount++;
        if (loopCount>driveCtrlRate)
        {
            loopCount=0;
        }
    }
    ROS_INFO("Navigator stopped");
}

void configCallback(ev3::ev3ParamsConfig &config, uint32_t level)
{
    {
        std::lock_guard<std::mutex> lock(configMutex);

        ROS_INFO("Ev3 reconfigure request, level %d", level);

        if ((level & 1))
        {
            edgeThreshold.store(config.EdgeThreshold);
            obstacleThreshold.store(config.ObstacleThreshold);

            hubRetryCount = config.hubRetryCount;
            hubTimeout = config.hubTimeout;

            hub->config(hubTimeout, hubRetryCount);
        }

        if ((level & 4))
        {
            imuRegCount = config.imuRegCount;
            imuTimeout = config.imuTimeout;
            imuRetryCount = config.imuRetryCount;

            imu->config(imuRegCount, imuTimeout, imuRetryCount);
        }

        if ((level & 2))
        {
            if (drive!=NULL)
            {
                drive->configure
                        (
                            config.TrackWidth, config.WheelDiam, config.GearRatio,
                            config.LinAccel, config.RotAccel,
                            config.MaxOutRatio
                            );
            }

            if (config.driveCtrlRate != driveCtrlRate)
            {
                driveCtrlRate = config.driveCtrlRate;
                if (navigatorThread!=NULL)
                {
                    ROS_INFO( "EV3 : waiting for navigator to stop..." );

                    stopRequest.store(true);
                    navigatorThread->join();
                    delete navigatorThread;
                    stopRequest.store(false);

                    navigatorThread = new std::thread(&navigatorTask);
                }

            }
        }

        if ((level & 8))
        {
            if (drive!=NULL)
            {
                drive->setVelPID(config.VelKp, config.VelKi, config.VelKd);
                drive->setRotPID(config.RotKp, config.RotKi, config.RotKd);
                drive->setHoldPID(config.HoldKp, config.HoldKi, config.HoldKd);
            }
        }
    }
}

int main(int argc, char** argv)
{
    bool reconnect = false;

    if (signal(SIGINT, sig_handler) == SIG_ERR)
    {
        printf("\ncan't catch SIGINT\n");
    }

    if (!(ev3_init() && ev3_tacho_init()))
    {
        printf("Failed to initialize EV3\n");
        return -1;
    }

    do
    {
        ros::init(argc, argv, "ev3");
        rosNode = new ros::NodeHandle();

        ESTOP.store(false);

        odomPub = rosNode->advertise<nav_msgs::Odometry>("/odom", 5);
        hubPub = rosNode->advertise<std_msgs::Int16MultiArray>("/sensorHub", 5);
        ros::Subscriber vel_sub = rosNode->subscribe<geometry_msgs::Twist>("/cmd_vel", 3, velCallback);

        imu = new ev3ros::bno055();
        hub = new ev3ros::sensorHub(hubTimeout);
        drive = new ev3ros::driveCtrl();

        // Set up dynamic reconfigure
        dynamic_reconfigure::Server<ev3::ev3ParamsConfig> srv;
        dynamic_reconfigure::Server<ev3::ev3ParamsConfig>::CallbackType f;

        f = boost::bind(configCallback, _1, _2);
        srv.setCallback(f);

        if (imu->init(imuPort, imuBaud) && hub->init(hubPort, hubBaud) && drive->init())
        {
            navigatorThread = new std::thread(&navigatorTask);

            cmdVelTimeStamp = ros::Time::now();

            ROS_INFO( "EV3 ready" );

            ros::Rate loop(5);

            while (!killed && ros::ok() && ros::master::check())
            {
                ros::Time now = ros::Time::now();
                if ((now - cmdVelTimeStamp).toSec() >= cmdVelTimeout)
                {
                    if (!ESTOP.load())
                    {
                        ESTOP.store(true);
                        linTarget.store(0);
                        rotTarget.store(0);
                        ROS_ERROR("EV3 : ESTOP - No commands received for %.3f s",(now - cmdVelTimeStamp).toSec());
                    }
                }
                loop.sleep();
                ros::spinOnce();
            }
            stopRequest.store(true);
            navigatorThread->join();
            stopRequest.store(false);
            delete navigatorThread;
        } else
        {
            ROS_ERROR( "EV3 Node failed to initialize" );
        }
        delete drive;
        delete imu;
        delete hub;
        delete rosNode;
        reconnect = !ros::master::check() && ros::ok();
    } while (!killed && reconnect);

    ROS_INFO( "EV3 Node Exiting" );

    ev3_uninit();
    return ( 0 );
}
