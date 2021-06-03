//
// Created by sjtu_wanghaili on 2020/11/9.
//

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include "height_border_msgs/height_border.h"
#include <cmath> // Needed for the pow function

class SubscribeAndPublish {
public:
    SubscribeAndPublish() {
        //Topic you want to publish
        pub_ = n_.advertise<std_msgs::UInt16>("height_control_mode", 1);

        //Topic you want to subscribe
        reap_height_sub_ = n_.subscribe("/reap_height_target", 100, &SubscribeAndPublish::reap_height_Callback, this);
        height_sub_ = n_.subscribe("/height_border", 100, &SubscribeAndPublish::reel_height_Callback, this);
        angle1_sub_ = n_.subscribe("/reap_angle1", 1, &SubscribeAndPublish::angle1_Callback, this);
        angle2_sub_ = n_.subscribe("/reap_angle2", 1, &SubscribeAndPublish::angle2_Callback, this);
    }

    void reap_height_Callback(const std_msgs::Int16::ConstPtr &msg) {
        //做一些计算，以确定下一步的控制信号
        //.... do something with the input and generate the output...
        reap_height_target = msg->data; // reap高度由自动驾驶的状态决定
    }
    void reel_height_Callback(const height_border_msgs::height_borderConstPtr &msg) {
        //做一些计算，以确定下一步的控制信号
        //.... do something with the input and generate the output...
        real_height_target = msg->height; // reel高度在谷物高度
    }

    void angle1_Callback(const std_msgs::Int64::ConstPtr &msg) {
        //做一些计算，以确定下一步的控制信号
        ROS_INFO_STREAM("get angle1");
        angle1 = msg->data;
    }

    void angle2_Callback(const std_msgs::Int64::ConstPtr &msg) {
        //做一些计算，以确定下一步的控制信号
        ROS_INFO_STREAM("get angle2");
        angle2 = msg->data;
    }

    void control_reap_height() {

        float a1 = -1.374e-05;
        float a2 = -1.893e-03;
        float a3 = 7.415e+01;
        float true_height = a1 * angle1 * angle1 + a2 * angle1 + a3;
        ROS_INFO_STREAM("angle1 is:"<<angle1);
        ROS_INFO_STREAM("true reap unit height:"<<true_height);

        std_msgs::UInt16 output;
        if(reap_height_target > 15){
            if(true_height - reap_height_target > 5){
                control_block = true;
		reap_control_once = false;
                output.data = 110;
                pub_.publish(output); // 发送控制模式
                ROS_INFO_STREAM("set down");
            } else if (true_height - reap_height_target < -5){
                control_block = true;
		reap_control_once = false;
                output.data = 120;
                pub_.publish(output);
                ROS_INFO_STREAM("set up");
            } else{
                control_block = false;
		if(!reap_control_once){
			output.data = 100;
                	pub_.publish(output);
		}

		reap_control_once = true;
            }
        }
    }

    void control_reel_height() {

        float a1 = 1.788e-06;
        float a2 = 1.472e-02;
        float a3 = 7.114e+01;
        float true_height = a1 * angle2 * angle2 + a2 * angle2 + a3;
        ROS_INFO_STREAM("angle2 is:"<<angle2);
        ROS_INFO_STREAM("true reel height:"<<true_height);

        // 若reap unit在进行控制，则reel不动作
        if(control_block){
            ROS_WARN_STREAM("reap unit block reel height control");
            return;
        }
        // 若reap unit在高位（收起），则reel也不进行调整
        if(reap_height_target==40){
		ROS_WARN_STREAM("reap unit at highest position.");
            return;
        }

        std_msgs::UInt16 output;
        if(real_height_target > 0){
		//ROS_WARN_STREAM("reel height"<<true_height);
		//ROS_WARN_STREAM("reel target"<<real_height_target);
            if(true_height - real_height_target > 5){
                output.data = 101;
                pub_.publish(output); // 发送控制模式
                ROS_INFO_STREAM("set reel down");
            } else if (true_height - real_height_target < -5){
                output.data = 102;
                pub_.publish(output);
                ROS_INFO_STREAM("set reel up");
            } else{
		ROS_INFO_STREAM("set reel steady");
                output.data = 100;
                pub_.publish(output);
            }
        }
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber reap_height_sub_;
    ros::Subscriber height_sub_;
    ros::Subscriber angle1_sub_;
    ros::Subscriber angle2_sub_;

    float angle1;
    float angle2;
    float reap_height_target;
    float real_height_target;

    bool control_block = false;
    bool reap_control_once = false;

};//End of class SubscribeAndPublish

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "height_controller");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    while(ros::ok()){
        ros::spinOnce();
        SAPObject.control_reap_height();
        SAPObject.control_reel_height();
        usleep(100000);
    }

    return 0;
}
