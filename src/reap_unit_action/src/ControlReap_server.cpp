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

std::string getTime(void)
{
    timeval tv;
    time_t timep;
    tm* timeNow;
    gettimeofday(&tv, NULL);//获取当下精确的s和us的时间
    time(&timep);//获取从1900年至今的秒数
    timeNow = gmtime(&timep); //注意tm_year和tm_mon的转换后才是实际的时间
    timeNow->tm_year+=1900;//实际年
    timeNow->tm_mon+=1;//实际月
    timeNow->tm_hour+=8;//实际小时
    if(timeNow->tm_hour>=24)
    {
        timeNow->tm_hour-=24;
    }
    long int ms = (tv.tv_sec*1000.0 + tv.tv_usec / 1000.0) - timep * 1000.0; //实际ms
    std::string res="";
    res+='[';
    res+=std::to_string(timeNow->tm_year);
    res+='-';
    res+=std::to_string(timeNow->tm_mon);
    res+='-';
    res+=std::to_string(timeNow->tm_mday);
    res+=' ';
    res+=std::to_string(timeNow->tm_hour);
    res+=':';
    res+=std::to_string(timeNow->tm_min);
    res+=':';
    res+=std::to_string(timeNow->tm_sec);
    res+=':';
    res+=std::to_string(int(ms));
    res+=']';
    return res;
}

typedef actionlib::SimpleActionServer<reap_unit_action::ControlReapAction> Server;

// 收到action的goal后调用的回调函数
void execute(const reap_unit_action::ControlReapGoalConstPtr& goal, Server* as)
{
    ROS_INFO_STREAM("goal got.");
    ROS_INFO_STREAM("MOTOR: "<<goal->dishwasher_id<<" target speed: "<<goal->target_speed);

    usleep(10000);


    as->setSucceeded();

	ROS_INFO_STREAM("Once action complete. Wait for next invoke.\n");
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
//    ros::spinOnce();

	// 停下电机
    can_1.set_motor_speed(40, 0);
    can_1.set_motor_speed(39, 0);
    can_1.set_motor_speed(38, 0);
    can_1.set_motor_speed(37, 0);

    can_1.closeCAN();

    return 0;
}