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


class sub_pub
{
private:
	ros::NodeHandle nh;
	// ros::Subscriber sub1;
	// ros::Subscriber sub2;
	// ros::Subscriber sub3;
	// ros::Subscriber sub4;
	ros::Subscriber sub_w;
	ros::Publisher pub_turn;
	ros::Publisher pub_speed;
	ros::Publisher pub_stop;


public:
	sub_pub()
	{
		// sub1 = nh.subscribe("/travel_angle", 1000, &sub_pub::Callback, this);
		// sub2 = nh.subscribe("/turn_angle", 1000, &sub_pub::Callback, this);
		// sub3 = nh.subscribe("/mainlever_angle", 1000, &sub_pub::Callback, this);
		// sub4 = nh.subscribe("/natural_angle", 1000, &sub_pub::Callback, this);
		///////
		sub_w = nh.subscribe("/height_border",100,&sub_pub::Callback, this);
		pub_turn = nh.advertise<std_msgs::Int16>("/turn",1000);
		pub_speed = nh.advertise<std_msgs::Int16>("/speed",1000);
	    pub_stop = nh.advertise<std_msgs::Int16>("/stop",1000);
	}
	void Callback(const height_border_msgs::height_border& height_borderMsg);           
};

void sub_pub::Callback(const height_border_msgs::height_border& height_borderMsg)
{
	string be_angle=height_borderMsg.angle_3d;
    double angle = atof(be_angle.c_str())-3.2;
	string be_dis=height_borderMsg.dis_3d;
	double dis = atof(be_dis.c_str())+95;
	need_turn = height_borderMsg.is_corner;

	std_msgs::Int16 speed;
	speed.data = 11000;
	pub_speed.publish(speed);
    std_msgs::Int16 msg_turn;
    // first turn without eyes
    if (need_turn)
    {
        record += 1;

        if (record == 20)
        {
            cout<<"///////////////////////*****************************START GO!!!!!!!!************************************////////////////////////"<<endl;
            msg_turn.data = 0;
            pub_turn.publish(msg_turn);
            usleep(15000000);
            cout<<"///////////////////////*****************************  TURN  ***********************************////////////////////////"<<endl;
            msg_turn.data = -7000;
            pub_turn.publish(msg_turn);
            usleep(6500000);
            msg_turn.data = 0;
            pub_turn.publish(msg_turn);
            cout<<"///////////////////////*****************************  GO AGAIN  ***********************************////////////////////////"<<endl;
            usleep(700000);
            record = 0;
        }
    }
    // second turn complex
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
//  double w = sin(alpha)/sqrt( (error+0.6)*(error*0.6)+d*d);
    double w = sin(alpha);
    cout<<"error: "<<error<<endl;
    cout<<"<<<<<<<<<<<<theta "<<theta<<endl;
    cout<<"<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<angle:"<<(angle/180)*3.1415926<<endl;
    cout<<"<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<alpha"<<alpha<<endl;
    int pls = -w2pls(0.6*w);
    cout<<"<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<w"<<w<<endl;

    if(pls>5000)
        {
            pls = 5000;
        }
        else if(pls<-4000)
        {
            pls = -4000;
        }
    msg_turn.data = pls;
    cout<<"<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<pls:"<<pls<<endl;
    pub_turn.publish(msg_turn);
}
double dis_3d;
void cb(const height_border_msgs::height_border& height_borderMsg){
    string str_dis_3d = height_borderMsg.dis_3d;
    dis_3d = stod(str_dis_3d);
    cout<<"Dis 3d "<<dis_3d<<endl;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "sub_pub1");
    sub_pub sub_puber;
//    ros::NodeHandle nh;
//    ros::Subscriber sub = nh.subscribe("/height_border",1,cb);
    ros::spin();
    return 0;
}

