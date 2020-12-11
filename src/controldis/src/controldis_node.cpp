
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <termio.h>
#include <stdio.h>
#include<iostream>
#include "height_border_msgs/height_border.h"

using namespace std;

// void start(const std_msgs::Int16::ConstPtr &msg);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control");
  ros::NodeHandle n;

  ros::Publisher speed_pub = n.advertise<std_msgs::Int16>("speed", 1000);
  ros::Publisher turn_pub = n.advertise<std_msgs::Int16>("turn", 1000);
  ros::Publisher stop_pub = n.advertise<std_msgs::Int16>("stop", 1000);
  ros::Publisher fake_height_pub = n.advertise<std_msgs::Int16>("/height_border_param1", 1000);
  // ros::Subscriber sub_0 = n_.subscribe("/start", 1, &start,this);
  //  int a = msg->data;
  //   if(a!=0)

  std_msgs::Int16 msg_speed;
  std_msgs::Int16 msg_turn;
  std_msgs::Int16 msg_stop;
  std_msgs::Int16 msg_fake_height;

  
//ros::Duration(5).sleep();
        usleep(5000000);
//直线行驶20秒
  ROS_INFO_STREAM("set speed:"<<6000);
  msg_speed.data = 6000;
  speed_pub.publish(msg_speed);

  //ros::Duration(20).sleep();
        usleep(20000000);
  //左转60度

  msg_fake_height.data = 65;
  fake_height_pub.publish(msg_fake_height);

for(int i = 1 ; i<9;i++)
{
  msg_turn.data = -1000*i;
  turn_pub.publish(msg_turn);
  cout<<msg_turn.data<<endl;
  //ros::Duration(0.2).sleep();
        usleep(200000);
}

  msg_fake_height.data = 100;
  fake_height_pub.publish(msg_fake_height);
//ros::Duration(6).sleep();
        usleep(23000000);

//回到0度
for(int a = 9 ; a>-1;a--)
{
  msg_turn.data = -1000*a;
  turn_pub.publish(msg_turn);
    cout<<msg_turn.data<<endl;
  //ros::Duration(0.3).sleep();
        usleep(300000);
}

  msg_fake_height.data = 80;
  fake_height_pub.publish(msg_fake_height);


//ros::Duration(20).sleep();
        usleep(20000000);
  msg_speed.data = 0;
  speed_pub.publish(msg_speed);

return 0;
}



// void start(const std_msgs::Int16::ConstPtr &msg)
//  {
   


// }
