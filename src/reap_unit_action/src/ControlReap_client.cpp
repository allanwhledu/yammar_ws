#include <actionlib/client/simple_action_client.h>
#include "reap_unit_action/ControlReapAction.h"

typedef actionlib::SimpleActionClient<reap_unit_action::ControlReapAction> Client;

// 当action完成后会调用次回调函数一次
void doneCb(const actionlib::SimpleClientGoalState& state,
        const reap_unit_action::ControlReapResultConstPtr& result)
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
void feedbackCb(const reap_unit_action::ControlReapFeedbackConstPtr& feedback)
{
    ROS_INFO(" percent_complete : %d ", feedback->percent_complete);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "control_reap_client");

	// 定义一个客户端，do_dishes就是topic
    Client client("control_reap", true);

	// 等待服务器端
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");

	// 创建一个action的goal
    reap_unit_action::ControlReapGoal goal;
    goal.dishwasher_id = 40;
    goal.target_speed = 1000;

    // 发送action的goal给服务器端，并且设置回调函数
    client.sendGoal(goal,  &doneCb, &activeCb, &feedbackCb);

    ros::spin();

    return 0;
}
