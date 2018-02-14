
#include <iostream>
#include <stdio.h>
#include <signal.h>

#include <boost/thread/thread.hpp>

#include "ev3.h"
#include "ev3_port.h"

#include "receiver.h"
#include "utils.h"

#include <ros/ros.h>

#include <serial/serial.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <nav_msgs/Odometry.h>

#include <dynamic_reconfigure/server.h>
#include <ev3/receiverParamsConfig.h>

struct channel
{
    uint16_t min;
    uint16_t max;
    uint16_t center;
    uint16_t deadBand;
} channelConfig[7];

const std::string receiverPort = "/dev/tty_in3";
const uint32_t receiverBaud = 115200;

int publishRate = 10;

double linearMin = -0.35;
double linearMax =  0.35;
double angularMin = -0.2;
double angularMax =  0.2;
int16_t servoPosMin = -200;
int16_t servoPosMax =  2500;
int16_t servoPosHome = 1900;

int linearChannel = 2;
int angularChannel = 0;
int servoChannel = 6;

ros::NodeHandle *rosNode;
ros::Publisher cmdVelPub;
ros::Publisher servoPub;
ros::Publisher rcPub;

ev3ros::receiver *receiver = NULL;

std::thread *receiverThread = NULL;
std::atomic<bool> stopRequest(false);

void sig_handler(int signo)
{
    if (signo == SIGINT)
    {
        printf("\nTerminating\n");
        stopRequest.store(true);
    }
}

void receiverTask();

void configCallback(ev3::receiverParamsConfig &config, uint32_t level)
{
    ROS_INFO("Receiver reconfigure request");

    channelConfig[0].min = config.ch0min;
    channelConfig[0].max = config.ch0max;
    channelConfig[0].center = config.ch0center;
    channelConfig[0].deadBand = config.ch0deadBand;

    channelConfig[1].min = config.ch1min;
    channelConfig[1].max = config.ch1max;
    channelConfig[1].center = config.ch1center;
    channelConfig[1].deadBand = config.ch1deadBand;

    channelConfig[2].min = config.ch2min;
    channelConfig[2].max = config.ch2max;
    channelConfig[2].center = config.ch2center;
    channelConfig[2].deadBand = config.ch2deadBand;

    channelConfig[3].min = config.ch3min;
    channelConfig[3].max = config.ch3max;
    channelConfig[3].center = config.ch3center;
    channelConfig[3].deadBand = config.ch3deadBand;

    channelConfig[4].min = config.ch4min;
    channelConfig[4].max = config.ch4max;
    channelConfig[4].center = config.ch4center;
    channelConfig[4].deadBand = config.ch4deadBand;

    channelConfig[5].min = config.ch5min;
    channelConfig[5].max = config.ch5max;
    channelConfig[5].center = config.ch5center;
    channelConfig[5].deadBand = config.ch5deadBand;

    channelConfig[6].min = config.ch6min;
    channelConfig[6].max = config.ch6max;
    channelConfig[6].center = config.ch6center;
    channelConfig[6].deadBand = config.ch6deadBand;

    linearChannel = config.linearChannel;
    linearMin = config.linearMin;
    linearMax = config.linearMax;

    angularChannel = config.angularChannel;
    angularMin = config.angularMin;
    angularMax = config.angularMax;

    servoChannel = config.servoChannel;
    servoPosMin = config.servoPosMin;
    servoPosMax = config.servoPosMax;
    servoPosHome = config.servoPosHome;

    if (config.PublishRate != publishRate)
    {
        publishRate = config.PublishRate;
        ROS_INFO("Receiver publish rete %d", publishRate);
        if (receiverThread!=NULL)
        {
            stopRequest.store(true);
            receiverThread->join();
            delete receiverThread;
            receiverThread = NULL;
            stopRequest.store(false);
        }
        if (publishRate>0)
        {
            receiverThread = new std::thread(&receiverTask);
            assert(receiverThread!=NULL);
        }
    }
}

void receiverTask()
{
    std::vector<uint16_t> channelReading;
    geometry_msgs::Twist cmdVel;
    std_msgs::Int16 servoPos;
    std_msgs::Int16MultiArray rcChannels;

    ros::Rate loop(publishRate);

    ROS_INFO( "receiver node : thread starting, rate=%d",publishRate);

    while (!stopRequest.load()) {
        if (receiver->getValues(0, ev3ros::channelCount, channelReading))
        {
            if (abs(channelReading[linearChannel] - channelConfig[linearChannel].center) > channelConfig[linearChannel].deadBand)
            {
                if (channelReading[linearChannel] > channelConfig[linearChannel].center)
                {
                    cmdVel.linear.x = scale(channelReading[linearChannel], channelConfig[linearChannel].center, channelConfig[linearChannel].max, 0, linearMax);
                } else
                {
                    cmdVel.linear.x = scale(channelReading[linearChannel], channelConfig[linearChannel].min, channelConfig[linearChannel].center, linearMin, 0);
                }
            } else
            {
                cmdVel.linear.x = 0;
            }
            cmdVel.linear.y = 0;
            cmdVel.linear.z = 0;

            if (abs(channelReading[angularChannel] - channelConfig[angularChannel].center) > channelConfig[angularChannel].deadBand)
            {
                if (channelReading[angularChannel] > channelConfig[angularChannel].center)
                {
                    cmdVel.angular.z = scale(channelReading[angularChannel], channelConfig[angularChannel].center, channelConfig[angularChannel].max, 0, angularMax);
                } else
                {
                    cmdVel.angular.z = scale(channelReading[angularChannel], channelConfig[angularChannel].min, channelConfig[angularChannel].center, angularMin, 0);
                }
            } else
            {
                cmdVel.angular.z = 0;
            }

            cmdVel.angular.x = 0;
            cmdVel.angular.y = 0;

            cmdVelPub.publish(cmdVel);

            servoPos.data = scale(channelReading[servoChannel], channelConfig[servoChannel].min, channelConfig[servoChannel].max, servoPosMin, servoPosMax);
            servoPub.publish(servoPos);

            rcChannels.data.clear();
            for (int i=0; i<channelReading.size(); ++i)
            {
                rcChannels.data.push_back(channelReading[i]);
            }
            rcPub.publish(rcChannels);
        }
        loop.sleep();
    }
    ROS_INFO( "receiver node : thread exiting");
}

int main(int argc, char** argv)
{
    bool reconnect = false;
    if (signal(SIGINT, sig_handler) == SIG_ERR)
    {
        printf("\ncan't catch SIGINT\n");
    }

    if (!(ev3_init()))
    {
        return -1;
    }

    do
    {
        ros::init(argc, argv, "receiver");

        stopRequest.store(false);

        rosNode = new ros::NodeHandle();
        ROS_INFO( "receiver node starting..." );

        cmdVelPub = rosNode->advertise<geometry_msgs::Twist>("/cmd_vel", 5);
        servoPub = rosNode->advertise<std_msgs::Int16>("/servo", 5);
        rcPub = rosNode->advertise<std_msgs::Int16MultiArray>("/rc", 5);

        receiver = new ev3ros::receiver();

        dynamic_reconfigure::Server<ev3::receiverParamsConfig> srv;
        dynamic_reconfigure::Server<ev3::receiverParamsConfig>::CallbackType f;

        f = boost::bind(configCallback, _1, _2);
        srv.setCallback(f);

        if (receiver->init(receiverPort, receiverBaud))
        {
            receiverThread = new std::thread(&receiverTask);

            ros::Rate loop(1);

            ROS_INFO( "receiver ready" );

            while (!stopRequest.load() && ros::master::check() && ros::ok())
            {
                loop.sleep();
                ros::spinOnce();
            }
            reconnect = !ros::master::check() && ros::ok();
        } else
        {
            reconnect = false;
            ROS_ERROR( "receiver node failed to initialize" );
        }
        ROS_INFO( "receiver node exiting" );

        stopRequest.store(true);
        receiverThread->join();
        stopRequest.store(false);
        delete receiverThread;
        delete receiver;
        delete rosNode;
        receiverThread = NULL;
    } while (!stopRequest.load() && reconnect);

    ev3_uninit();
    return ( 0 );
}
