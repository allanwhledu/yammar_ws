#include <ros/ros.h>
#include "std_msgs/String.h"

class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{
			//Topic you want to publish
			pub_ = n_.advertise<std_msgs::String>("grain_height_stream", 1);
			pub2_ = n_.advertise<std_msgs::String>("grain_height_data", 1);

            height_sub_ = n_.subscribe("/realsense_sr300/ylx/rgb", 1, &SubscribeAndPublish::callback, this);
			sub2_ = n_.subscribe("/realsense_sr300/ylx/depth", 1, &SubscribeAndPublish::callback2, this);
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
		ros::Publisher pub2_;
		ros::Subscriber height_sub_;
		ros::Subscriber sub2_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "grain_height_detecte");

	//Create an object of class SubscribeAndPublish that will take care of everything
	SubscribeAndPublish SAPObject;

	ros::spin();

	return 0;
}
