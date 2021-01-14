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


void height_control_mode_callback(const std_msgs::UInt16::ConstPtr& msg)  // 虽然这个包主要用来读取can消息，但是也通过这个函数来发送一些控制信息
{
    ROS_INFO("got control mode: ", msg->data);
    can_2.control_height(msg->data);
}

void test_topic_callback(const std_msgs::UInt16::ConstPtr& msg)  // 虽然这个包主要用来读取can消息，但是也通过这个函数来发送一些控制信息
{
    float current_now = msg->data;
    float rms = can_2.calculate_rms(current_now);
    ROS_INFO_STREAM("rms: "<<rms);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reap_height_capture");
	ros::NodeHandle n;

	// 采集信号后发布到制定topic
	ros::Publisher chatter_pub1 = n.advertise<std_msgs::Int64>("reap_angle1", 1000);
    ros::Publisher chatter_pub2 = n.advertise<std_msgs::Int64>("reap_angle2", 1000);
    ros::Publisher chatter_pub3 = n.advertise<std_msgs::Float32>("car_speed", 1000);
    ros::Publisher chatter_pub4 = n.advertise<std_msgs::Float32>("torque", 1000);
    ros::Publisher chatter_pub5 = n.advertise<std_msgs::Float32>("current_rms", 1000);
	can_1.pub_c1 = &chatter_pub1;
	can_1.pub_c2 = &chatter_pub2;
    can_1.pub_c3 = &chatter_pub3;
	can_1.pub_c4 = &chatter_pub4;
    can_1.pub_c5 = &chatter_pub5;

	// 接受topic指令后发送can信号
    ros::Subscriber sub = n.subscribe("height_control_mode", 1, height_control_mode_callback);
    ros::Subscriber sub_test = n.subscribe("test_topic", 1, test_topic_callback);


//    can_1.m_run0 = 0;
//    ROS_INFO_STREAM("wait close");
//    pthread_join(can_1.receive_thread, NULL);//等待线程关闭
//    ROS_INFO_STREAM("receive_thread_close.");
    can_1.init_CAN();
    can_1.init_ICAN();

    ROS_INFO_STREAM("can1 receive thread starting...");
    can_1.open_receive();

//    //测试是否可以双通道运行

    can_2.init_CAN();
//    ROS_INFO_STREAM("can2 receive thread starting...");
//    can_2.open_receive();

    int spin_or_watchdog = 0;
    if(spin_or_watchdog == 0)
    {
        ros::spin();
    }
    else if(spin_or_watchdog == 1)
    {
        ros::Rate loop_rate(1);
        int count = 0;
        while (ros::ok() && count < 10000)
        {
            ROS_INFO_STREAM("Watchdog running...");
            loop_rate.sleep();
            ++count;
        }
    }

    can_1.close_receive();
	can_1.closeCAN();
//    can_2.close_receive();
    can_2.closeCAN();
	return 0;
}
