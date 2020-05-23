#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{
			//Topic you want to publish
			pub_ = n_.advertise<std_msgs::String>("cmd_avoiding_obstacle", 1);

			//Topic you want to subscribe
			sub_ = n_.subscribe("/perceptual_nodes/detected_obstacle", 1, &SubscribeAndPublish::callback, this);
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
		ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "avoiding_obstacle");

	//Create an object of class SubscribeAndPublish that will take care of everything
	SubscribeAndPublish SAPObject;

	ros::spin();

	return 0;
}
