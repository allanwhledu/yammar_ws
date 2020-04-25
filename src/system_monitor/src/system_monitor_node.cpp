#include <ros/ros.h>
#include "std_msgs/String.h"

class SubscribeAndPublish
{
	public:
		SubscribeAndPublish()
		{
			//Topic you want to publish
			pub_ = n_.advertise<std_msgs::String>("system_status", 1);

			//Topic you want to subscribe
			sub_ = n_.subscribe("/perceptual_nodes/full_view_stream", 1, &SubscribeAndPublish::callback, this);
			sub2_ = n_.subscribe("/perceptual_nodes/detected_obstacle", 1, &SubscribeAndPublish::callback2, this);
			sub3_ = n_.subscribe("/perceptual_nodes/harvest_line_data", 1, &SubscribeAndPublish::callback3, this);
			sub4_ = n_.subscribe("/perceptual_nodes/grain_height_data", 1, &SubscribeAndPublish::callback4, this);
			sub5_ = n_.subscribe("/capture_nodes/reap_height_data", 1, &SubscribeAndPublish::callback5, this);
			sub6_ = n_.subscribe("/automatic_nodes/path_data", 1, &SubscribeAndPublish::callback6, this);
			// sub7_ = n_.subscribe("camera", 1, &SubscribeAndPublish::callback7, this);
			// sub8_ = n_.subscribe("camera", 1, &SubscribeAndPublish::callback8, this);
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
		void callback3(const std_msgs::StringConstPtr& input)
		{
			std_msgs::String output;
			//.... do something with the input and generate the output...
			pub_.publish(output);
		}
		void callback4(const std_msgs::StringConstPtr& input)
		{
			std_msgs::String output;
			//.... do something with the input and generate the output...
			pub_.publish(output);
		}
		void callback5(const std_msgs::StringConstPtr& input)
		{
			std_msgs::String output;
			//.... do something with the input and generate the output...
			pub_.publish(output);
		}
		void callback6(const std_msgs::StringConstPtr& input)
		{
			std_msgs::String output;
			//.... do something with the input and generate the output...
			pub_.publish(output);
		}
		// void callback7(const std_msgs::StringConstPtr& input)
		// {
		//         std_msgs::String output;
		//         //.... do something with the input and generate the output...
		//         pub_.publish(output);
		// }
		// void callback8(const std_msgs::StringConstPtr& input)
		// {
		//         std_msgs::String output;
		//         //.... do something with the input and generate the output...
		//         pub_.publish(output);
		// }
	private:
		ros::NodeHandle n_; 
		ros::Publisher pub_;
		ros::Subscriber sub_;
		ros::Subscriber sub2_;
		ros::Subscriber sub3_;
		ros::Subscriber sub4_;
		ros::Subscriber sub5_;
		ros::Subscriber sub6_;
		// ros::Subscriber sub7_;
		// ros::Subscriber sub8_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "system_monitor");

	//Create an object of class SubscribeAndPublish that will take care of everything
	SubscribeAndPublish SAPObject;

	ros::spin();

	return 0;
}
