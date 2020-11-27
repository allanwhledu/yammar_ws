//
// Created by sjtu_wanghaili on 2020/6/4.
//

#include<iostream>
#include<string>
#include<unistd.h>  /* UNIX standard function definitions */
#include<fcntl.h>   /* File control definitions */
#include<termios.h> /* POSIX terminal control definitions */
#include<sys/time.h>    //for time
#include<iostream>
#include<fstream>    // 读写文件的头文件
#include<string>
#include<vector>
#include<algorithm>
#include<signal.h>
#include<pthread.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"
#include "height_border_msgs/height_border.h"
#include <actionlib/server/simple_action_server.h>


using namespace std;

string current_time = "";
ofstream* open_file;

bool endFlag = false;

int angle1=0;
int angle2=0;
int control_mode = 100;
float height_visual = 0;

bool save_height = false;
bool save_mode = false;


// 函数申明
void* getTime(void*)
{
    // 时间在后台20ms更新一次
    while(!endFlag)
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

        current_time ="";
        current_time+='[';
        current_time+=to_string(timeNow->tm_year);
        current_time+='-';
        current_time+=to_string(timeNow->tm_mon);
        current_time+='-';
        current_time+=to_string(timeNow->tm_mday);
        current_time+=' ';
        current_time+=to_string(timeNow->tm_hour);
        current_time+=':';
        current_time+=to_string(timeNow->tm_min);
        current_time+=':';
        string s_string = to_string(timeNow->tm_sec);
        while (s_string.size()<2)
        {
            s_string="0"+s_string;
        }
        current_time+=s_string;
        current_time+=':';
        string ms_string = to_string(int(ms));
        if(ms_string.size()>3){
            ms_string = "000";
        }
        while (ms_string.size()<3)
        {
            ms_string="0"+ms_string;
        }
        current_time+=ms_string;
        current_time+=']';
        usleep(20000);
    }
    ROS_WARN_STREAM("current time sync stoped.");
//    endFlag = false;
}
// --- --- //

void angle2_callback(const std_msgs::BoolConstPtr &msg) {

}

void height_control_mode_callback(const std_msgs::UInt16ConstPtr &msg) {
    ROS_INFO_STREAM("callback! controlmode: "<<msg->data);
    control_mode = msg->data;
    save_mode = true;
}

void height_callback(const height_border_msgs::height_borderConstPtr &msg) {
    ROS_INFO_STREAM("callback! height: "<<msg->height);
    height_visual = msg->height;
    save_height = true;
}

void angle1_callback(const std_msgs::Int64ConstPtr &msg) {
    ROS_INFO_STREAM("callback! carspeed: "<<msg->data);
    angle1 = msg->data;
    // 记录速度
    string angle1_str = to_string(angle1);
    while (angle1_str.size() < 4)
    {
        angle1_str= "0" + angle1_str;
    }

    int height_int = height_visual;
    string height_str = to_string(height_int);
    if(height_visual<0){
        height_str = "0000";
    }
    while (height_str.size() < 4)
    {
        height_str= "0" + height_str;
    }

    string controlmode_str = "0100";
    if(save_mode){
        controlmode_str = to_string(control_mode);
        while (controlmode_str.size() < 4)
        {
            controlmode_str= "0" + controlmode_str;
        }
        save_mode = false;
    }

    *(open_file) << angle1_str << " " << height_str << " " << controlmode_str << endl;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "hello") ;
    ROS_INFO_STREAM("Hello, ROS!") ;
    ros::NodeHandle n_;

    pthread_t time_sync;
    pthread_create(&time_sync, nullptr, getTime, nullptr);
    ROS_INFO_STREAM("time sync spread make.");

    ofstream ofs;
    string filename = "/home/sjtu_wanghaili/yammar_ws/experiment20201113/";
    filename = filename + current_time + "_speed.txt";
    ofs.open(filename, ios::out);
    if(!ofs)
    {
        cerr<<"Open File Fail."<<endl;
        exit(1);
    }
    open_file = &ofs;

    ros::Subscriber sub2_;
    ros::Subscriber sub3_;
    ros::Subscriber sub_;
    ros::Subscriber sub4_;

    //Topic you want to subscribe

    sub3_ = n_.subscribe("/height_control_mode", 1, &height_control_mode_callback);
    sub4_ = n_.subscribe("/height_border_param", 1, &height_callback);
    sub_ = n_.subscribe("/reap_angle1", 1, &angle1_callback);
    sub2_ = n_.subscribe("/reap_angle2", 1, &angle2_callback);

    ros::spin();

    ROS_INFO_STREAM("wait spread close.");
    endFlag = true;
    pthread_kill(time_sync, 0);
    ros::Duration(10);

    ofs.close();
    return 0;
}
