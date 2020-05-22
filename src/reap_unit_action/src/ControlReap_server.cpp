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
    if(goal->dishwasher_id < 100){
        can_1.pfd.percent_complete = 10000;
        can_1.open_receive();
        // 保存server的指针到全局变量
        can_1.pas = as;

        // 清理can卡缓冲区,但是感觉没有太大的用处
//    VCI_ClearBuffer(VCI_USBCAN2,0,0);

//	reap_unit_action::ControlReapFeedback feedback;
//	pfd = &feedback;

        ROS_INFO("Start Motor %d.", goal->dishwasher_id);

        can_1.setMotor(goal->dishwasher_id);
        // 设置电机为速度模式
        can_1.set_speed_mode(goal->dishwasher_id);
        // 发送电机驱动指令
        can_1.set_motor_speed(goal->dishwasher_id, goal->target_speed);
//    usleep(10000);

        // 读取电机速度误差
        can_1.check_speed();//线程关闭指令,这里是关闭接受线程
        ROS_INFO_STREAM("succeed.");

        as->setSucceeded();

        ROS_INFO_STREAM("Action complete. Wait for next invoke.\n");
    }
    else
    {
        ROS_INFO_STREAM("to get target motor current.");
        //todo

        can_1.pfd.percent_complete = 10000;
        can_1.open_receive();
        // 保存server的指针到全局变量
        can_1.pas = as;

        ROS_INFO("to get Motor %d current.", goal->dishwasher_id - 100);
        can_1.setMotor(goal->dishwasher_id - 100);

        can_1.callCurrent(goal->dishwasher_id - 100);

        // 读取电机速度误差
        can_1.wait_current();//线程关闭指令,这里是关闭接受线程
        ROS_INFO_STREAM("check current end.");

        as->setSucceeded();
    }
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
    can_1.set_motor_speed(36, 0);
    can_1.set_motor_speed(35, 0);
    can_1.set_motor_speed(34, 0);
    can_1.set_motor_speed(33, 0);
    can_1.set_motor_speed(32, 0);
    can_1.set_motor_speed(31, 0);

    can_1.closeCAN();

    return 0;
}