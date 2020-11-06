//
// Created by sjtu_wanghaili on 2020/9/4.
//

#include <ros/ros.h>
#include "std_msgs/Float32.h"

class SubscribeAndPublish {
public:
    SubscribeAndPublish() {
        //Topic you want to publish
        pub_ = n_.advertise<std_msgs::Float32>("car_speed", 1);
    }

    void send_fake_speed(double msg) {
        //.... do something with the input and generate the output...
        std_msgs::Float32 output;
        output.data = msg;
        pub_.publish(output);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv) {
    //Initiate ROS
    ros::init(argc, argv, "fake_speed_sender");

    //Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAPObject;

    double fake_speed = 0;
    double time_count = 0;
    int time_count_shreshhold = 30;
    while (ros::ok() && (time_count < time_count_shreshhold * 60 / 5)){
        fake_speed = fake_speed + 0.1;
        SAPObject.send_fake_speed(fake_speed);

        if(fake_speed > 1.5)
            fake_speed = 0;

        usleep(5000000);
        time_count++;
    }

    return 0;
}