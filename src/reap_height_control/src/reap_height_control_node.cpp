#include "ros/ros.h"
#include "std_msgs/String.h"
#include <actionlib/client/simple_action_client.h>
#include "reap_height_action/ControlReapHeightAction.h"
#include "ros/node_handle.h"
#include "ros/subscriber.h"

typedef actionlib::SimpleActionClient<reap_height_action::ControlReapHeightAction> Client;

// 当action完成后会调用次回调函数一次
void doneCb(const actionlib::SimpleClientGoalState& state,
        const reap_height_action::ControlReapHeightResultConstPtr& result)
{
    ROS_INFO("Yay! The dishes are now clean");
    ros::shutdown();
}

// 当action激活后会调用次回调函数一次
void activeCb()
{
    ROS_INFO("Goal just went active");
}

// 收到feedback后调用的回调函数
void feedbackCb(const reap_height_action::ControlReapHeightFeedbackConstPtr& feedback)
{
    ROS_INFO(" percent_complete : %f ", feedback->percent_complete);
}

void velCallback(const std_msgs::String::ConstPtr& msg)
{
	/*...TODO...*/ 
}

void velCallback2(const std_msgs::String::ConstPtr& msg)
{
	/*...TODO...*/ 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_reap_height_client");

	// 定义一个客户端，do_dishes就是topic
    Client client("/action_nodes/control_reap_height", true);
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/capture_nodes/reap_height", 1, velCallback);
    ros::Subscriber sub2 = n.subscribe("/perceptual_nodes/grain_height_data", 1, velCallback2);


	// 等待服务器端
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");

	// 创建一个action的goal
    reap_height_action::ControlReapHeightGoal goal;
    goal.dishwasher_id = 1;

    // 发送action的goal给服务器端，并且设置回调函数
    client.sendGoal(goal,  &doneCb, &activeCb, &feedbackCb);

    ros::spin();

    return 0;
}
