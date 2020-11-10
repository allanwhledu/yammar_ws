// for action
#include <ios>
#include <locale>
#include <locale.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"

// for CAN
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"
#include <ctime>
#include <cstdlib>
#include <cstdio>
#include "unistd.h"
#include "CAN_DEVICE.h"
#include <bitset>

// channel_idx应该是1或2
CAN_DEVICE can_2(2);

void Callback(const std_msgs::UInt16::ConstPtr& msg)
{
    ROS_INFO("got control mode: ", msg->data);
    can_2.control_height(msg->data);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_reap_server");
    setlocale( LC_ALL, "" );

    ros::NodeHandle n;

    // 初始化CAN卡
    can_2.init_CAN();

    ros::Subscriber sub = n.subscribe("height_control_mode", 1, Callback);

    ros::spin();

    can_2.closeCAN();

    return 0;
}