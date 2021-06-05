//
// Created by yangzt on 2021/6/4.
//
#include <iostream>
#include <ros/ros.h>
#include "fstream"
#include <geometry_msgs/Pose2D.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
geometry_msgs::Pose2D curr_pos;
void gps_cb(const geometry_msgs::Pose2DConstPtr & msg){
    curr_pos.x = msg->x;
    curr_pos.y = msg->y;
    curr_pos.theta = msg->theta;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "gps_recorder");
    ros::NodeHandle nh;

    ros::Subscriber curr_pos_sub = nh.subscribe<geometry_msgs::Pose2D>("curr_pose_gps", 1000, gps_cb);

    string gps_traj_name = "/home/yangzt/yammar_ws/src/serialPort/gps_traj_gaoyou.txt";
    ofstream gps_traj;
    gps_traj.open(gps_traj_name.c_str());

    // Generate an img to Show;
    cv::Mat img = Mat::zeros(480,640, CV_8UC3);
    img = Scalar(255, 255, 255);
    ros::Rate rate(50);
    ros::spinOnce();
    double x_origin = curr_pos.x;
    double y_origin = curr_pos.y;

    while(ros::ok()){
        ros::spinOnce();
        gps_traj<<std::setiosflags(std::ios::fixed)  << std::setprecision(9)<<curr_pos.x<<" "<<curr_pos.y<<" "<<curr_pos.theta<<endl;
        cv::circle(img, Point(int(curr_pos.x - x_origin) + 320, int(curr_pos.y - y_origin) + 240), 5, Scalar(0,255,0), -1);
        cv::imshow("Gps", img);
        cv::waitKey(1);
        rate.sleep();
    }
    gps_traj.close();
    return 0;
}
