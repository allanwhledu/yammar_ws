#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "height_border_msgs/height_border.h"
#include <cmath>
#include "curve_point_z_msgs/curve_point_z.h"


using namespace std;

double alpha = 0;
double d = 3;//孙汉的视觉距离
double theta = 0;
bool need_turn = false;
int record = 0;
int gogo=1;// need change for true harvest
double before_angle;
int before_dis;


// for path track
int curr_val_path_track = 0;
int w2pls(double W)
{
    int pls;
    if ( W< -0.03)
        pls = -(1805*(-W)*(-W)*(-W)-778.4*(-W)*(-W)+273.5*(-W)+10.34)*91.4;
    else if(0.03<W)
        pls = (1805*W*W*W-778.4*W*W+273.5*W+10.34)*91.4;
    else
        pls = 0;
    return pls;
}

//速度1m/s
//int w2pls(double W)
//{
//    int pls;
//    if (W < -0.03)
//        pls = -(158.8 * (-W) * (-W) * (-W) - 152.6 * (-W) * (-W) + 135.8 * (-W) + 9.302) * 91.4;
//    else if (0.03 < W)
//        pls = (158.8 * W * W * W - 152.6 * W * W + 135.8 * W + 9.302) * 91.4;
//    else
//        pls = 0;
//    return pls;
//
//    //158.8 * x * x * x - 152.6 * x * x + 135.8 * x + 9.302
//}


void Callback_start(const std_msgs::Int16::ConstPtr &msg)
{
    gogo = msg->data;
}

void Callback(const height_border_msgs::height_border& height_borderMsg)
{
    if(height_borderMsg.angle_3d.empty() || height_borderMsg.dis_3d.empty()) return;
    string be_angle=height_borderMsg.angle_3d;
//    before_angle = atof(be_angle.c_str());//修改************************************************
    before_angle = stod(height_borderMsg.angle_3d);
    if(before_angle > 15.0 || before_angle < -15.0) before_angle = 0;
    string be_dis=height_borderMsg.dis_3d;
//    before_dis = atof(be_dis.c_str());//修改************************************************
    before_dis = stoi(height_borderMsg.angle_3d);
    if(before_dis < -50 || before_dis > 50) before_dis = 0;
    need_turn = height_borderMsg.is_corner;
    cout<<before_angle<<"  "<<before_dis<<endl;
}
void path_track_cb(const std_msgs::Int16ConstPtr &msg){
    curr_val_path_track = msg->data;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "sub_pub1");
    ros::NodeHandle nh;

    ros::Subscriber sub_start = nh.subscribe("/smach_fback", 1000, &Callback_start);
    ros::Publisher pub_start = nh.advertise<std_msgs::Int16>("/submodules_status",1000);


    ros::Subscriber sub_w = nh.subscribe("/height_border",100,&Callback);

    ros::Publisher pub_turn = nh.advertise<std_msgs::Int16>("/turn",1000);
    ros::Publisher pub_speed = nh.advertise<std_msgs::Int16>("/speed",1000);
    ros::Publisher pub_stop = nh.advertise<std_msgs::Int16>("/stop",1000);
    ros::Publisher pub_height = nh.advertise<std_msgs::Int16>("/reap_height_target",1000);
    ros::Subscriber path_track_sub = nh.subscribe("/path_track", 1000, path_track_cb);

    std_msgs::Int16 speed;
    std_msgs::Int16 msg_turn;
    int judge = 0;
    usleep(2000000);
    //先进行收割机方向盘转向功能检测
    for(int i = 0; i < 10; i++){
        msg_turn.data = 2000;
        pub_turn.publish(msg_turn);
    }
    usleep(1000000);
    for(int i = 0; i < 10; i++){
        msg_turn.data = 1;
        pub_turn.publish(msg_turn);
    }

    cout<<"方向盘是否转动，若方向盘转动,则按数字'1'，若方向盘不转动，则C杀死进程，并拨打孙晗电话"<<endl;
    cin>>judge;

    if(judge==0){
        while(ros::ok()){}
    }
     
    for(int i = 0; i < 10; i++){
        speed.data = 11000;
        pub_speed.publish(speed);
    }
  
    while (ros::ok()) {
        ros::spinOnce();
        usleep(10000);
        if(curr_val_path_track < -200 ){
            msg_turn.data = 2500;
        }else if(curr_val_path_track < -100){
            msg_turn.data = 2000;
        }else if(curr_val_path_track > 60){
            msg_turn.data = -1500;
        }else if(curr_val_path_track > 100){
            msg_turn.data = -2000;
        }else if(curr_val_path_track > 150){
            msg_turn.data = -2250;
        }else if(curr_val_path_track > 200){
            msg_turn.data = -2500;
        }
        else{
            msg_turn.data = 0;
        }
        pub_turn.publish(msg_turn);
        ROS_INFO_STREAM(msg_turn.data);
    }

    return 0;
}








//    std_msgs::Int16 height_control;

//    double pid_p = 0.6;
//    double offset_ange = 0;
//    double offset_dis = 0;
//
//    nh.getParam("frontview_dis",d);
//    nh.getParam("p",pid_p);
//    nh.getParam("offset_ange",offset_ange);
//    nh.getParam("offset_dis",offset_dis);
//    // i am ok
//    ready.data=2;
//    for(int f = 0; f<10;f++){
//        pub_start.publish(ready);
//        usleep(10000);
//    }
//    // wait for msg to go
//    int count  = 0;
//    while (ros::ok()) {
//
//        ros::spinOnce();
//        double angle = before_angle + offset_ange;
//        double dis = before_dis + offset_dis;
//
//        if (gogo == 0){
//            usleep(100000);
//            speed.data = 0;
//            pub_speed.publish(speed);
////          speed.data = 15000;// speed = 1m/s
//            continue;
//        }
//
//        height_control.data = 20;// speed = 0.5m/s
//        pub_height.publish(height_control);
//        usleep(1000000);
//        speed.data = 11000;// speed = 0.5m/s
//        pub_speed.publish(speed);
//
//        // *********************************** first turn without eyes ******************************//
////        if (need_turn)
////        {
////            record += 1;
////
////            if (record == 0)
////            {
////                cout<<"///////////////////////*****************************START GO!!!!!!!!************************************////////////////////////"<<endl;
////                msg_turn.data = 0;
////                pub_turn.publish(msg_turn);
////                usleep(15000000);
////                cout<<"///////////////////////*****************************  TURN  ***********************************////////////////////////"<<endl;
////                msg_turn.data = -7000;
////                pub_turn.publish(msg_turn);
//                //   height_control.data = 40;
//                //   pub_height.publish(height_control);
////                usleep(6500000);
////                msg_turn.data = 0;
////                pub_turn.publish(msg_turn);
////                cout<<"///////////////////////*****************************  GO AGAIN  ***********************************////////////////////////"<<endl;
////                usleep(700000);
////                record = 0;
////            }
////        }
//        // *********************************** first turn without eyes ******************************//
//
//
//        // *********************************** second turn complex ******************************//
////    if (need_turn)
////    {
////        record += 1;
////        if (record == 10)
////        {
//        //    cout<<"///////////////////////*****************************go along************************************////////////////////////"<<endl;
//        //    msg_turn.data = 0;
//        //    pub_turn.publish(msg_turn);
//        //    usleep(15000000);
//
////            cout<<"///////////////////////*****************************go back  ***********************************////////////////////////"<<endl;
//
////            height_control.data = 40;
////            pub_height.publish(height_control);
//
////            msg_turn.data = -9000;
////            pub_turn.publish(msg_turn);
////            speed.data = -11000;
////            pub_speed.publish(speed);
////            usleep(5000000);
////            cout<<"///////////////////////*****************************  go again  ***********************************////////////////////////"<<endl;
//
////            height_control.data = 20;
////            pub_height.publish(height_control);
//
////            msg_turn.data = 0;
////            pub_turn.publish(msg_turn);
////            speed.data = 11000;
////            pub_speed.publish(speed);
////            cout<<"///////////////////////*****************************  youzhuan  ***********************************////////////////////////"<<endl;
////            record = 0;
////        }
////    }
//
//        double error = dis*0.01;
//        theta = atan(error/d);
//        alpha = theta + (angle/180)*3.1415926;
////     double w = sin(alpha)/sqrt( (error+0.6)*(error*0.6)+d*d);
//        double w = sin(alpha);
//
//        int pls = -w2pls(pid_p*w);//修改************************************************
//
//
//        if(pls>4000)
//        {
//            pls = 4000;
//        }
//        else if(pls<-4000)
//        {
//            pls = -4000;
//        }
//        msg_turn.data = pls;
//        pub_turn.publish(msg_turn);
//        if(false){
//            cout<<"error: "<<error<<endl;
//            cout<<"<<<<<<<<<<<<theta "<<theta<<endl;
//            cout<<"<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<angle:"<<(angle/180)*3.1415926<<endl;
//            cout<<"<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<alpha"<<alpha<<endl;
//            cout<<"<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<w"<<w<<endl;
//            cout<<"<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<pls:"<<pls<<endl;
//            count =0;
//        }
////        count++;
//
//    }


