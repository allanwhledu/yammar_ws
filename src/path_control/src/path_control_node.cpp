#include <ros/ros.h>
#include "ros/subscriber.h"
#include "std_msgs/String.h"

class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{
			//Topic you want to publish
			pub_ = n_.advertise<std_msgs::String>("/automatic_nodes/cmd_vel", 1);

			//Topic you want to subscribe
			sub_ = n_.subscribe("cmd_avoiding_obstacle", 1, &SubscribeAndPublish::callback, this);
			sub2_ = n_.subscribe("path_data", 1, &SubscribeAndPublish::callback2, this);
		}

		void callback(const std_msgs::StringConstPtr& input)
		{
			std_msgs::String output;
			//.... do something with the input and generate the output...
			pub_.publish(output);
		}
		void callback2(const std_msgs::StringConstPtr& input)
		{
			std_msgs::String output;
			//.... do something with the input and generate the output...
			pub_.publish(output);
		}

	private:
		ros::NodeHandle n_; 
		ros::Publisher pub_;
		ros::Subscriber sub_;
		ros::Subscriber sub2_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "path_control");

	//Create an object of class SubscribeAndPublish that will take care of everything
	SubscribeAndPublish SAPObject;

	ros::spin();

	return 0;
}
