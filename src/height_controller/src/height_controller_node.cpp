//
// Created by sjtu_wanghaili on 2020/11/9.
//

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"
#include "height_border_msgs/height_border.h"

class SubscribeAndPublish {
public:
    SubscribeAndPublish() {
        //Topic you want to publish
        pub_ = n_.advertise<std_msgs::UInt16>("height_control_mode", 1);

        //Topic you want to subscribe
        height_sub_ = n_.subscribe("/height_border_param", 100, &SubscribeAndPublish::height_border_Callback, this);
        angle1_sub_ = n_.subscribe("/reap_angle1", 100, &SubscribeAndPublish::angle1_Callback, this);
        angle2_sub_ = n_.subscribe("/reap_angle2", 100, &SubscribeAndPublish::angle2_Callback, this);
    }

    void height_border_Callback(const height_border_msgs::height_borderConstPtr &msg) {
        //做一些计算，以确定下一步的控制信号
        std_msgs::UInt16 output;
        //.... do something with the input and generate the output...
        float true_height = angle1 + angle2;
        if(true_height - msg->height > 20){
            output.data = 110;
            pub_.publish(output); // 发送控制模式
        } else if (true_height - msg->height < -20){
            output.data = 120;
            pub_.publish(output);
        } else{
            output.data = 100;
            pub_.publish(output);
        }
    }

    void angle1_Callback(const std_msgs::UInt16ConstPtr &msg) {
        //做一些计算，以确定下一步的控制信号
        angle1 = msg->data;
    }

    void angle2_Callback(const std_msgs::UInt16ConstPtr &msg) {
        //做一些计算，以确定下一步的控制信号
        angle2 = msg->data;
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber height_sub_;
    ros::Subscriber angle1_sub_;
    ros::Subscriber angle2_sub_;

    int angle1;
    int angle2;

};//End of class SubscribeAndPublish

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "height_controller");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    ros::spin();

    return 0;
}

