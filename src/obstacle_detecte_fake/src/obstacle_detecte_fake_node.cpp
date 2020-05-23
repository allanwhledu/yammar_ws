#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>

class SubscribeAndPublish
{
	public:
        int obstacle;
        int sign_obstacle;
        int clean;
        int turn;

		SubscribeAndPublish()
		{
			//Topic you want to publish
			pub_ = n_.advertise<std_msgs::String>("PNPConditionEvent", 1);

			//Topic you want to subscribe
			sub_ = n_.subscribe("/scan", 1, &SubscribeAndPublish::callback, this);

			obstacle = 0;
			sign_obstacle = 0;
			clean = 0;
			turn = 0;
		}

		void callback(const sensor_msgs::LaserScanConstPtr& input)
		{
            //.... do something with the input and generate the output...
            std_msgs::String output;

            obstacle = 0;
            for(auto it = input->ranges.begin();it!=input->ranges.end();it++)
            {
                if((*it) < 0.1)
                {
                    obstacle ++;
                }
            }
            if(obstacle > 100){
                sign_obstacle = 1;
                clean = 0;
            }
            else{
              clean = 1;
              sign_obstacle =0;
            }

            if(sign_obstacle ==1 && turn == 0){
                ROS_WARN_STREAM("obstacle detected!");
                output.data = "obstacle";
                pub_.publish(output);
                ROS_WARN_STREAM("obstacle send.");
                sign_obstacle = 0;
                turn = 1;
            }

            if(clean == 1 && turn == 1){
                ROS_WARN_STREAM("obstacle clean!");
                output.data = "obstacleclean";
                pub_.publish(output);
                ROS_WARN_STREAM("clean send.");
                clean = 0;
                turn = 0;
            }
		}

	private:
		ros::NodeHandle n_; 
		ros::Publisher pub_;
		ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
	//Initiate ROS
	ros::init(argc, argv, "obstacle_detecte_fake");

	//Create an object of class SubscribeAndPublish that will take care of everything
	SubscribeAndPublish SAPObject;

//	ros::Rate loop_rate(10);
//	ROS_INFO_STREAM("init complete.");
//	while(ros::ok()){
//	    ros::spinOnce();
//	    ROS_INFO_STREAM("spin once.");
//	    loop_rate.sleep();
//	}
    ros::spin();

	return 0;
}
