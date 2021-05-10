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
    std::vector<float> current_buffer0;
    std::vector<float> current_buffer1;
    std::vector<float> current_buffer2;
    std::vector<float> current_buffer3;
    std::vector<float> current_buffer4;
    std::vector<float> current_buffer5;
    std::vector<float> current_buffer6;
    std::vector<float> current_buffer7;
    std::vector<float> current_buffer8;
    std::vector<float> current_buffer9;
    std::vector<float> current_buffer10;
    int buffer_length = 100;

    int channel;

    int angle1 = 0;
    int angle2 = 0;

    std_msgs::Float32 car_speed;
    float torque = 0;
    float current = 0.0;
    int angle_turn = 0;
    int angle_speed = 0;

    ros::Publisher* reap_angle1_pub;
    ros::Publisher* reap_angle2_pub;
    ros::Publisher* car_speed_pub;
    ros::Publisher* torque_pub;
//    ros::Publisher* pub_c5;
//    ros::Publisher* pub_c5_raw;
    ros::Publisher* cm7290_current_pub;
    ros::Publisher* angle_turn_pub;
    ros::Publisher* angle_speed_pub;

    ros::Publisher* pub_c_motor1;
    ros::Publisher* pub_c_motor1_raw;
    ros::Publisher* pub_c_motor2;
    ros::Publisher* pub_c_motor2_raw;
    ros::Publisher* pub_c_motor3;
    ros::Publisher* pub_c_motor3_raw;
    ros::Publisher* pub_c_motor4;
    ros::Publisher* pub_c_motor4_raw;
    ros::Publisher* pub_c_motor5;
    ros::Publisher* pub_c_motor5_raw;
    ros::Publisher* pub_c_motor6;
    ros::Publisher* pub_c_motor6_raw;
    ros::Publisher* pub_c_motor7;
    ros::Publisher* pub_c_motor7_raw;
    ros::Publisher* pub_c_motor8;
    ros::Publisher* pub_c_motor8_raw;
    ros::Publisher* pub_c_motor9;
    ros::Publisher* pub_c_motor9_raw;
    ros::Publisher* pub_c_motor10;
    ros::Publisher* pub_c_motor10_raw;

    CAN_DEVICE(int channel_idx);

    void init_CAN();
    friend void* receive_func(void* param);
    float calculate_rms0(float current_now);
    float calculate_rms1(float current_now);
    float calculate_rms2(float current_now);
    float calculate_rms3(float current_now);
    float calculate_rms4(float current_now);
    float calculate_rms5(float current_now);
    float calculate_rms6(float current_now);
    float calculate_rms7(float current_now);
    float calculate_rms8(float current_now);
    float calculate_rms9(float current_now);
    float calculate_rms10(float current_now);
    void transmit_msg(VCI_CAN_OBJ *send, char *com);
    void control_height(int mode); //驱动第num_motor号电机，速度为speed.

    void open_receive();

    void closeCAN();

    void close_receive();

    void init_ICAN(int id);
};


#endif //SRC_CAN_DEVICE_H
