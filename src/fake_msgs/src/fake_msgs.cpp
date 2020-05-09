#include "ros/publisher.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fake_msgs_gen");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float32>("FH_speed", 1000);
	ros::Publisher chatter_pub2 = n.advertise<std_msgs::Float32>("/chart",1000);
	ros::Rate loop_rate(10);

	std_msgs::Float32 msg;
	std_msgs::Float32 msg2;
	int count = 0;
	while (ros::ok() && count < 2000)
	{
		/**
		 * 向 Topic: chatter 发送消息, 发送频率为10Hz（1秒发10次）；消息池最大容量1000。
		 */
		msg.data = count;
		msg2.data = 1.5;
		chatter_pub.publish(msg);
		chatter_pub2.publish(msg2);

		loop_rate.sleep();
		// ++count;
		count  = count + 100;
	}
	return 0;
}
