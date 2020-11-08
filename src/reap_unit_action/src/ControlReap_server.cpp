// for action
#include <ios>
#include <locale>
#include <locale.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "reap_unit_action/ControlReapAction.h"

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
CAN_DEVICE can_1(1);

std::vector<int> log_error;

typedef actionlib::SimpleActionServer<reap_unit_action::ControlReapAction> Server;

// 收到action的goal后调用的回调函数
void execute(const reap_unit_action::ControlReapGoalConstPtr& goal, Server* as)
{
    ROS_INFO("Control mode", goal->control_mode);

    can_1.control_height(goal->control_mode);
//    can_1.control_height(0); // 控制之后需要静止

    as->setSucceeded();

    ROS_INFO_STREAM("Action complete. Wait for next invoke.\n");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_reap_server");
    setlocale( LC_ALL, "" );

    ros::NodeHandle n;

    // 初始化CAN卡
    can_1.init_CAN();

	// 定义一个服务器，do_dishes就是topic
	Server server(n, "control_reap", boost::bind(&execute, _1, &server), false);

	// 服务器开始运行
	server.start();

	ros::spin();

    can_1.closeCAN();

    return 0;
}