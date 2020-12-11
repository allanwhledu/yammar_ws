/**
 * 该例程将发布chatter话题，消息类型String
 */
 
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <termio.h>
#include <stdio.h>
#include <stdlib.h>
#include<sstream>


using namespace std;


// int scanKeyboard()
// {
// int in;
// struct termios new_settings;
// struct termios stored_settings;
// tcgetattr(0,&stored_settings);
// new_settings = stored_settings;
// new_settings.c_lflag &= (~ICANON);
// new_settings.c_cc[VTIME] = 0;
// tcgetattr(0,&stored_settings);
// new_settings.c_cc[VMIN] = 1;
// tcsetattr(0,TCSANOW,&new_settings);
 
// in = getchar();
 
// tcsetattr(0,TCSANOW,&stored_settings);
// return in;
// }
string Convert(float Num)
{
	ostringstream oss;
	oss<<Num;
	string str(oss.str());
	return str;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher speed_pub = n.advertise<std_msgs::String>("dis", 1000);

  ros::Rate loop_rate(10);




  while (ros::ok())
  {


   
     std_msgs::String msg_speed;
     float speed = 0;
     cout<<"<<distance:"<<endl;
     cin>>speed;
     string sa=Convert(speed);//sa=="123"


     msg_speed.data = sa;
     speed_pub.publish(msg_speed);
   

    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}

