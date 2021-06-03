#include "ros/publisher.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fake_msgs_gen");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("speed1", 1000);
	ros::Publisher chatter_pub2 = n.advertise<std_msgs::Float32>("/chart",1000);
	ros::Publisher chatter_pub3 = n.advertise<std_msgs::Int16>("smach_fback",1000);
	ros::Rate loop_rate(10);

	std_msgs::Float32 msg;
	std_msgs::Float32 msg2;
	std_msgs::Int16 msg3;
	int count = 0;
	while (ros::ok())
	{
		/**
		 * 向 Topic: chatter 发送消息, 发送频率为10Hz（1秒发10次）；消息池最大容量1000。
		 */
		msg.data = count;
		msg2.data = 1.5;
		msg3.data = 1;
		chatter_pub.publish(msg);
		chatter_pub2.publish(msg2);
		chatter_pub3.publish(msg3);

		loop_rate.sleep();
		// ++count;
		count  = count + 100;
	}
	return 0;
}
