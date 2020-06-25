//
// Created by bit on 2020/5/8.
//

#ifndef SRC_CAN_DEVICE_H
#define SRC_CAN_DEVICE_H


#include <controlcan.h>
#include <vector>
#include <pthread.h>
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
#include <locale.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

class CAN_DEVICE {
public:
    VCI_BOARD_INFO pInfo; //用来获取设备信息
    int count; //数据列表中，用来存储列表序号

    // 接收线程
    pthread_t receive_thread;
    int m_run0;

    std::vector<int> log_error; //用于debug的时候记录变量

    int motor;
    int channel;

    int angle1 = 0;
    int angle2 = 0;
    std_msgs::Float32 car_speed;

    ros::Publisher* pub_c1;
    ros::Publisher* pub_c2;
    ros::Publisher* pub_c3;

    void setMotor(int motor);

    CAN_DEVICE(int channel_idx);

    void init_CAN();
    friend void* receive_func(void* param);
    void transmit_msg(VCI_CAN_OBJ *send, char *com);
    void set_speed_mode(int num_motor);
    void set_motor_speed(int num_motor, int speed);
    void callFeedback(int num_motor);

    void callCurrent(int num_motor);
    void wait_current();

    void open_receive();
    void check_speed();

    void closeCAN();

    void close_receive();

    void init_ICAN();
};


#endif //SRC_CAN_DEVICE_H
