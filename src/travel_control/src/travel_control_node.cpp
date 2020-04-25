#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
	/*...TODO...*/ 
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "travel_control");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/automatic_nodes/cmd_vel", 2, chatterCallback);

	ros::Rate loop_rate(5);
	while (ros::ok())
	{
		/*...TODO...*/ 

		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
