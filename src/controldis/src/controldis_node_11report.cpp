#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "height_border_msgs/height_border.h"


using namespace std;

struct pid
{
	float pid_sety;      /****期望速度****/
	float y;
	float Kp;
	float Ki;
	float Kd;
	float err;               /**偏差**/
	float last_err;          /**上一次的偏差**/
	float voltage;           /**方向盘转角**/
	float T;                 /**更新周期**/
	float integral;
	float theta;
	float vy;
	float Vi;
	float Vo;
	float w;
	float v;
}_pid;

void pid_init()
{
	_pid.pid_sety = 0.0;
	_pid.Kp = -2000;    //PID调节
	_pid.Ki = 0;
	_pid.Kd = 0;
	_pid.err = 0.0;
	_pid.last_err = 0.0;
	_pid.integral = 0.0;
	_pid.y = 0;
	_pid.w = 0;
}

//PID开始运行
float pid_run(float sety,float dis)
{
	_pid.pid_sety = sety;
	_pid.y = dis;
	_pid.err = _pid.pid_sety - _pid.y;
	_pid.integral += _pid.err;
	_pid.w = _pid.err*_pid.Kp + _pid.integral*_pid.Ki + (_pid.err - _pid.last_err)*_pid.Kd;
	if (_pid.w >10000)
		_pid.w = 10000;
	else if (_pid.w < -10000)
		_pid.w = -10000;
	_pid.last_err = _pid.err;
	return _pid.w;
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
		sub_w = nh.subscribe("/height_borderMsg",100,&sub_pub::Callback, this);
		pub_turn = nh.advertise<std_msgs::Int16>("/turn",1000);
		pub_speed = nh.advertise<std_msgs::Int16>("/speed",1000);
	    pub_stop = nh.advertise<std_msgs::Int16>("/stop",1000);
	}
	void Callback(const height_border_msgs::height_border& height_borderMsg);           
};

void sub_pub::Callback(const height_border_msgs::height_border& height_borderMsg)
{
	string be_angle=height_borderMsg.angle_3d;
    double angle = atof(be_angle.c_str());
	
	string be_dis=height_borderMsg.dis_2d;
    double dis = atof(be_dis.c_str());
    // int tu=travel_angle->data;
    // int ml=travel_angle->data;
    // int na=natural_angle->data;
	std_msgs::Int16 speed;
	speed.data = 6000;
	pub_speed.publish(speed);

    
//   if((angle>10)||(angle<-10))
//   {
	         
// 			 std_msgs::Int16 turn;
// 			 unsigned char rate;  
//              rate = 50;  
//              ros::Rate loop_rate(rate); 
//        //如果车向左边偏转，那我就方向盘右转
// 			 if(angle>0)
//             {
// 			 float output = 300*(10-angle)-1200;
// 			 cout<<"motor turn"<<output<<endl;
//              turn.data = output;
// 			 pub_turn.publish(turn);
// 			}
// 			  //如果车向右边偏转，那我就方向盘左转
// 		    if(angle<0)
//             {
// 			 float output = -300*(10+angle)+1200;
// 			 cout<<"motor turn"<<output<<endl;
//              turn.data = output;
// 			 pub_turn.publish(turn);
// 			}
//   }
//   else
//   {	 
	        if(dis>25)
			{
		 //如果车向左边偏转，那我就方向盘右转
	         std_msgs::Int16 turn;		
			 float output = -500;
               turn.data = output;
			   pub_turn.publish(turn);

			   	 output = -1000;
               turn.data = output;
			   pub_turn.publish(turn);
			      	 output = -1500;
               turn.data = output;
			   pub_turn.publish(turn);
			      	 output = -2000;
               turn.data = output;
			   pub_turn.publish(turn);
		
           }

		
		
		
		
		   	        if(dis>10)
			{
		 //如果车向左边偏转，那我就方向盘右转
	         std_msgs::Int16 turn;		
			 float output = -1500;
               turn.data = output;
			   pub_turn.publish(turn);
		
           }
		   	        if(dis>5)
			{
		 //如果车向左边偏转，那我就方向盘右转
	         std_msgs::Int16 turn;		
			 float output = -500;
               turn.data = output;
			   pub_turn.publish(turn);
		
           }
		   		   	        if(dis<5)
			{
		 //如果车向左边偏转，那我就方向盘右转
	         std_msgs::Int16 turn;		
			 float output = 500;
               turn.data = output;
			   pub_turn.publish(turn);
		
           }

		//    	        if((dis<-5)||(dis>5))
		// 	{
		//  //如果车向左边偏转，那我就方向盘右转
	    //      std_msgs::Int16 turn;
	  	//      //float output = pid_run(1.5,dis);
		// 	if(dis<0)
		// 	{
		// 	   float output = -dis*100+1200;
		// 	   	cout<<"motor turn"<<output<<endl;
        //        turn.data = output;
		// 	   pub_turn.publish(turn);
		// 	}
		// 	else
        //      {
		// 	   float output = -dis * 100 -1200 ;
		// 	   cout<<"motor turn"<<output<<endl;
        //        turn.data = output;
		// 	   pub_turn.publish(turn);
		// 	 }
		
        //    }
	      
//   }

  	// if (dis<1)
	// {
	//   std_msgs::Int16 msg_stop;
	//   msg_stop.data = 1;
    //   pub_stop.publish(msg_stop);
	// }
}
    

int main(int argc, char **argv)
{

    ros::init(argc, argv, "sub_pub1");
	pid_init();
    sub_pub sub_puber;
    ros::spin();
    return 0;
}

