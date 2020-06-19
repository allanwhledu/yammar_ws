#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int64.h"
#include <sstream>
#include "unistd.h"
#include "CAN_DEVICE.h"
#include <bitset>

// channel_idx应该是1或2
CAN_DEVICE can_1(1);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "reap_height_capture");
	ros::NodeHandle n;
	ros::Publisher chatter_pub1 = n.advertise<std_msgs::Int64>("reap_angle1", 1000);
    ros::Publisher chatter_pub2 = n.advertise<std_msgs::Int64>("reap_angle2", 1000);
	can_1.pub_c1 = &chatter_pub1;
	can_1.pub_c2 = &chatter_pub2;


//    can_1.m_run0 = 0;
//    ROS_INFO_STREAM("wait close");
//    pthread_join(can_1.receive_thread, NULL);//等待线程关闭
//    ROS_INFO_STREAM("receive_thread_close.");
    can_1.init_CAN();
    can_1.init_ICAN();

    ROS_INFO_STREAM("receive thread starting...");
    can_1.open_receive();

    ros::Rate loop_rate(1);
	int count = 0;
	while (ros::ok() && count < 10000)
	{
        ROS_INFO_STREAM("Watchdog running...");
		loop_rate.sleep();
		++count;
	}

    can_1.close_receive();
	can_1.closeCAN();
	return 0;
}
