#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "height_border_msgs/height_border.h"
#include <cmath>

using namespace std;

double alpha = 0;
double d = 3;//孙汉的视觉距离
double theta = 0;
bool need_turn = false;
int record = 0;
int gogo=0;
double angle;
double dis;

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
    string be_angle=height_borderMsg.angle_3d;
    angle = stod(be_angle.c_str())-3.2;//修改************************************************
    string be_dis=height_borderMsg.dis_3d;
    dis = stod(be_dis.c_str())+95;//修改************************************************
    need_turn = height_borderMsg.is_corner;

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
    std_msgs::Int16 ready;
    std_msgs::Int16 speed;
    std_msgs::Int16 msg_turn;
    double pid_p = 0.6;

    nh.getParam("frontview_dis",d);
    nh.getParam("p",pid_p);
    // i am ok
    ready.data=2;
    for(int f = 0; f<10;f++){
        pub_start.publish(ready);
        usleep(10000);
    }
    // wait for msg to go
    while (ros::ok()) {

        ros::spinOnce();
        if (gogo == 0){
            usleep(100000);
            speed.data = 0;
            pub_speed.publish(speed);
//          speed.data = 15000;// speed = 1m/s
            continue;
        }

        speed.data = 11000;// speed = 0.5m/s
        pub_speed.publish(speed);
        // *********************************** first turn without eyes ******************************//
//        if (need_turn)
//        {
//            record += 1;
//
//            if (record == 20)
//            {
//                cout<<"///////////////////////*****************************START GO!!!!!!!!************************************////////////////////////"<<endl;
//                msg_turn.data = 0;
//                pub_turn.publish(msg_turn);
//                usleep(15000000);
//                cout<<"///////////////////////*****************************  TURN  ***********************************////////////////////////"<<endl;
//                msg_turn.data = -7000;
//                pub_turn.publish(msg_turn);
//                usleep(6500000);
//                msg_turn.data = 0;
//                pub_turn.publish(msg_turn);
//                cout<<"///////////////////////*****************************  GO AGAIN  ***********************************////////////////////////"<<endl;
//                usleep(700000);
//                record = 0;
//            }
//        }
        // *********************************** first turn without eyes ******************************//


        // *********************************** second turn complex ******************************//
//    if (need_turn)
//    {
//        record += 1;
//        if (record == 5)
//        {
//            cout<<"///////////////////////*****************************go along************************************////////////////////////"<<endl;
//            msg_turn.data = 0;
//            pub_turn.publish(msg_turn);
//            usleep(5000000);
//            cout<<"///////////////////////*****************************go back  ***********************************////////////////////////"<<endl;
//            msg_turn.data = 9000;
//            pub_turn.publish(msg_turn);
//            speed.data = -11000;
//            pub_speed.publish(speed);
//            usleep(5000000);
//            cout<<"///////////////////////*****************************  go again  ***********************************////////////////////////"<<endl;
//            msg_turn.data = 0;
//            pub_turn.publish(msg_turn);
//            speed.data = 11000;
//            pub_speed.publish(speed);
//            cout<<"///////////////////////*****************************  youzhuan  ***********************************////////////////////////"<<endl;
//            record = 0;
//        }
//    }

        double error = dis*0.01;
        theta = atan(error/d);
        alpha = theta + (angle/180)*3.1415926;
//     double w = sin(alpha)/sqrt( (error+0.6)*(error*0.6)+d*d);
        double w = sin(alpha);
        cout<<"error: "<<error<<endl;
        cout<<"<<<<<<<<<<<<theta "<<theta<<endl;
        cout<<"<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<angle:"<<(angle/180)*3.1415926<<endl;
        cout<<"<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<alpha"<<alpha<<endl;
        int pls = -w2pls(pid_p*w);//修改************************************************
        cout<<"<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<w"<<w<<endl;

        if(pls>4000)
        {
            pls = 4000;
        }
        else if(pls<-4000)
        {
            pls = -4000;
        }
        msg_turn.data = pls;
        cout<<"<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<pls:"<<pls<<endl;
        pub_turn.publish(msg_turn);

    }

    return 0;
}

