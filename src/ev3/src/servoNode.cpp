#include <math.h>
#include <stdio.h>
#include <signal.h>
#include <cassert>

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int16MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include <ev3/servoParamsConfig.h>

#include <boost/thread/thread.hpp>

#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"

#include "servo.h"
#include "utils.h"

uint8_t updateRate = 15;
double angleFactor = -0.0404;
double angleOffset = -82.181;

ros::NodeHandle *rosNode;
ros::Publisher tiltAnglePub;
ros::Subscriber rc_sub;

ev3ros::servo *servo = NULL;
bool killed = false;

void sig_handler(int signo)
{
    if (signo == SIGINT)
    {
        printf("\nTerminating\n");
        killed = true;
    }
}

void configCallback(ev3::servoParamsConfig &config, uint32_t level)
{
    ROS_INFO("servo reconfigure request");
    servo->setPosPID(config.PosKp, config.PosKi,config.PosKd);
    servo->setVelPID(config.VelKp, config.VelKi,config.VelKd);
    servo->setParams(config.Accel, config.Vel, config.Margin, config.Scale);

    angleFactor = config.AngleFactor;
    angleOffset = config.AngleOffset;

    if ((updateRate!=config.UpdateRate) && (config.UpdateRate>0))
    {
        updateRate = config.UpdateRate;
        servo->setRate(updateRate);
    }
}

void posCallback(const std_msgs::Int16::ConstPtr& servoPos)
{
    servo->setPos((int16_t)servoPos->data);
}

int main(int argc, char** argv)
{
    bool reconnect = false;
    std_msgs::Float64 tiltAngle;

    if (signal(SIGINT, sig_handler) == SIG_ERR)
    {
        printf("\ncan't catch SIGINT\n");
    }

    if (!(ev3_init() && ev3_tacho_init() && ev3_sensor_init()))
    {
        return -1;
    }

    do
    {
        ros::init(argc, argv, "servo");
        rosNode = new ros::NodeHandle();
        ROS_INFO( "servo node starting..." );

        tiltAnglePub = rosNode->advertise<std_msgs::Float64>("/camera/tilt", 5);
        rc_sub = rosNode->subscribe<std_msgs::Int16>("/servo", 1, posCallback);

        servo = new ev3ros::servo();

        dynamic_reconfigure::Server<ev3::servoParamsConfig> srv;
        dynamic_reconfigure::Server<ev3::servoParamsConfig>::CallbackType f;

        f = boost::bind(configCallback, _1, _2);
        srv.setCallback(f);

        if (servo->init(-1))
        {
            ros::Rate loop(updateRate/2);

            ROS_INFO( "servo ready" );

            while (!killed && ros::master::check() && ros::ok())
            {
                double pos = servo->getPos();
                tiltAngle.data = pos*angleFactor + angleOffset;
                tiltAnglePub.publish(tiltAngle);

                loop.sleep();
                ros::spinOnce();
            }

            reconnect = !ros::master::check() && ros::ok();

        } else
        {
            reconnect = false;
            ROS_ERROR( "servo failed to initialize" );
        }

        ROS_INFO( "servo node exiting" );

        delete servo;
        delete rosNode;

    } while (!killed && reconnect);

    ev3_uninit();
    return ( 0 );
}
