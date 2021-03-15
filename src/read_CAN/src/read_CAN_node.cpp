#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int64.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include "unistd.h"
#include "CAN_DEVICE.h"
#include <bitset>

// channel_idx应该是1或2
CAN_DEVICE can_1(1);
CAN_DEVICE can_2(2);


void height_control_mode_callback(const std_msgs::UInt16::ConstPtr &msg) {
    ROS_INFO("got control mode: ", msg->data);
    can_2.control_height(msg->data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "reap_height_capture");
    ros::NodeHandle n;

    // 采集信号后发布到制定topic
    ros::Publisher chatter_pub1 = n.advertise<std_msgs::Int64>("reap_angle1", 1000);
    ros::Publisher chatter_pub2 = n.advertise<std_msgs::Int64>("reap_angle2", 1000);
    ros::Publisher chatter_pub3 = n.advertise<std_msgs::Float32>("car_speed", 1000);
    ros::Publisher chatter_pub4 = n.advertise<std_msgs::Float32>("torque", 1000);
    ros::Publisher chatter_pub5 = n.advertise<std_msgs::Float32>("current_rms", 1000);
    ros::Publisher chatter_pub_raw_current = n.advertise<std_msgs::Float32>("current_raw", 1000);
    ros::Publisher chatter_pub_cm7290_current = n.advertise<std_msgs::Float32>("current_cm7290", 1000);
    ros::Publisher chatter_pub6 = n.advertise<std_msgs::Float32>("angle_turn", 1000);
    ros::Publisher chatter_pub7 = n.advertise<std_msgs::Float32>("angle_speed", 1000);
    can_1.pub_c1 = &chatter_pub1;
    can_1.pub_c2 = &chatter_pub2;
    can_1.pub_c3 = &chatter_pub3;
    can_1.pub_c4 = &chatter_pub4;
    can_1.pub_c5 = &chatter_pub5;
    can_1.pub_c5_raw = &chatter_pub_raw_current;
    can_1.pub_c5_cm7290 = &chatter_pub_cm7290_current;
    can_1.pub_turn_c6 = &chatter_pub6;
    can_1.pub_speed_c7 = &chatter_pub7;

    ros::Publisher chatter_pub_motor4 = n.advertise<std_msgs::Float32>("current4_rms", 1000);
    ros::Publisher chatter_pub_raw_current_motor4 = n.advertise<std_msgs::Float32>("current4_raw", 1000);
    can_1.pub_c_motor4 = &chatter_pub_motor4;
    can_1.pub_c_motor4_raw = &chatter_pub_raw_current_motor4;

    ros::Publisher chatter_pub_motor5 = n.advertise<std_msgs::Float32>("REEL_current", 1000);
    ros::Publisher chatter_pub_raw_current_motor5 = n.advertise<std_msgs::Float32>("current5_raw", 1000);
    can_1.pub_c_motor5 = &chatter_pub_motor5;
    can_1.pub_c_motor5_raw = &chatter_pub_raw_current_motor5;

    ros::Publisher chatter_pub_motor6 = n.advertise<std_msgs::Float32>("CB_current", 1000);
    ros::Publisher chatter_pub_raw_current_motor6 = n.advertise<std_msgs::Float32>("current6_raw", 1000);
    can_1.pub_c_motor6 = &chatter_pub_motor6;
    can_1.pub_c_motor6_raw = &chatter_pub_raw_current_motor6;

    ros::Publisher chatter_pub_motor7 = n.advertise<std_msgs::Float32>("PF_current", 1000);
    ros::Publisher chatter_pub_raw_current_motor7 = n.advertise<std_msgs::Float32>("current7_raw", 1000);
    can_1.pub_c_motor7 = &chatter_pub_motor7;
    can_1.pub_c_motor7_raw = &chatter_pub_raw_current_motor7;

    // 接受topic指令后发送can信号
    ros::Subscriber sub = n.subscribe("height_control_mode", 1, height_control_mode_callback);


//    can_1.m_run0 = 0;
//    ROS_INFO_STREAM("wait close");
//    pthread_join(can_1.receive_thread, NULL);//等待线程关闭
//    ROS_INFO_STREAM("receive_thread_close.");
    can_1.init_CAN();
    can_1.init_ICAN(2);

    ROS_INFO_STREAM("can1 receive thread starting...");
    can_1.open_receive();

//    //测试是否可以双通道运行

    can_2.init_CAN();
    ROS_INFO_STREAM("can2 receive thread starting...");
    can_2.open_receive();

    int spin_or_watchdog = 0;
    if (spin_or_watchdog == 0) {
        ros::spin();
    } else if (spin_or_watchdog == 1) {
        ros::Rate loop_rate(1);
        int count = 0;
        while (ros::ok() && count < 10000) {
            ROS_INFO_STREAM("Watchdog running...");
            loop_rate.sleep();
            ++count;
        }
    }

    can_1.close_receive();
    can_1.closeCAN();
    can_2.close_receive();
    can_2.closeCAN();
    return 0;
}