#include <ros/ros.h>
#include "ros/publisher.h"
#include "std_msgs/String.h"

class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{
			//Topic you want to publish
			pub_ = n_.advertise<std_msgs::String>("path_stream", 1);
			pub2_ = n_.advertise<std_msgs::String>("path_data", 1);

			//Topic you want to subscribe
			sub_ = n_.subscribe("/perceptual_nodes/harvest_line_data", 1, &SubscribeAndPublish::callback, this);
		}

		void callback(const std_msgs::StringConstPtr& input)
		{
			std_msgs::String output;
			//.... do something with the input and generate the output...
			pub_.publish(output);
		}

	private:
		ros::NodeHandle n_; 
		ros::Publisher pub_;
		ros::Publisher pub2_;
		ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "path_planning");

	//Create an object of class SubscribeAndPublish that will take care of everything
	SubscribeAndPublish SAPObject;

	ros::spin();

	return 0;
}
