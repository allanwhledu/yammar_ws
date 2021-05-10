/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2018
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <qobjectdefs.h>
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <sstream>
#include <qnode.h>
#include "ros/init.h"
#include "ros/rate.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include <std_msgs/UInt16.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace test_gui
{

/*****************************************************************************
** Implementation
*****************************************************************************/
QNode::QNode(int argc, char** argv )
  : init_argc(argc), init_argv(argv)
{}

QNode::~QNode()
{
  if(ros::isStarted())
  {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

void QNode::myCallback_img(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    ROS_INFO_STREAM("image_received.");
    //cv::imshow("gui_subscriber", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
    img = cv_ptr->image;
    //QImage image(conversion_mat_.data, conversion_mat_.cols, conversion_mat_.rows, conversion_mat_.step[0], QImage::Format_RGB888);
    image = QImage(img.data,img.cols,img.rows,img.step[0],QImage::Format_RGB888);
    //img = cv_bridge::toCvShare(msg, "bgr8")->image;
    //    ROS_INFO("I'm setting picture in mul_t callback function!");
    Q_EMIT loggingCamera();
    //cv::waitKey(33);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

bool QNode::init()
{
  ros::init(init_argc,init_argv,"test_gui");
  if (!ros::master::check())
  {
    return false;
  }

  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;
  ros::NodeHandle im;
  image_transport::ImageTransport it(im);

  // Add your ros communications here.
  text_subscriber = n.subscribe("/height_border_param", 100, &QNode::height_border_Callback, this);
  chart_subscriber = n.subscribe("/chart",1,&QNode::ChartCallback,this);
  REEL_subscriber = n.subscribe("/motor_3_speed", 1, &QNode::motor1_speed_Callback, this);
  CB_subscriber = n.subscribe("/motor_4_speed", 1, &QNode::motor2_speed_Callback, this);
  PF_subscriber = n.subscribe("/motor_2_speed", 1, &QNode::motor3_speed_Callback, this);
  FH_subscriber = n.subscribe("/motor_1_speed", 1, &QNode::motor4_speed_Callback, this);
  REEL_current_subscriber = n.subscribe("/current1_rms", 1, &QNode::current1_Callback, this);
  CB_current_subscriber = n.subscribe("/current2_rms", 1, &QNode::current2_Callback, this);
  PF_current_subscriber = n.subscribe("/current3_rms", 1, &QNode::current3_Callback, this);
  FH_current_subscriber = n.subscribe("/current_cm7290", 1, &QNode::current4_Callback, this);
  obstacle_subscriber = n.subscribe("/is_obstacle",1,&QNode::is_obstacle_Callback,this);
  reap_height1_subscriber = n.subscribe("/reap_angle1",1,&QNode::reap_height1_Callback,this);
  reap_height2_subscriber = n.subscribe("/reap_angle2",1,&QNode::reap_height2_Callback,this);
  torque_subscriber = n.subscribe("/car_speed",1,&QNode::torque_Callback,this);
  //暂时借用，显示速度
//  image_sub = it.subscribe("/perceptual_nodes/harvest_line_stream",100,&QNode::myCallback_img,this);//相机尝试
  image_sub = it.subscribe("/boud_depth",100,&QNode::myCallback_img,this);//相机尝试

  car_speed_pub = n.advertise<std_msgs::Float32>("car_speed", 1000);
  is_stop_pub = n.advertise<std_msgs::Int16>("stop", 1000);
  car_turn_pub = n.advertise<std_msgs::Int16>("turn", 1000);

  height_control_mode_pub = n.advertise<std_msgs::UInt16>("/height_control_mode", 1);


  start();
  ROS_INFO_STREAM("inited qnode.");


  // ros::spin();
  // run();

  // int loop_count = 1;
  // ros::Rate loop_rate(10);
  // while (!ros::isShuttingDown()) {
  //         ros::spinOnce();
  //         loop_rate.sleep();
  //         loop_count++;
  // }
  // std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  // Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)

  return true;
}


void QNode::height_border_Callback(const height_border_msgs::height_borderConstPtr &msg) {
    leader_line_error_string = QString::fromStdString(msg->angle_3d);
    leader_line_error = atof(msg->angle_3d.c_str());

    height_string = QString::number(msg->height);
    height_value = msg->height;

//        ROS_INFO_STREAM(msg->height);

    Q_EMIT logging_leader_line_error();
}

void QNode::ChartCallback(const std_msgs::Float32Ptr &msg)
{
  chart = msg->data;
  ROS_INFO_STREAM("chart receive: "<<chart);
  Q_EMIT loggingChart();
}

void QNode::motor1_speed_Callback(const std_msgs::Float32Ptr &msg)
{
    REEL_speed = msg->data;
  ROS_INFO_STREAM("REEL_speed receive: " << REEL_speed);
  Q_EMIT logging_REEL_speed();
}

void QNode::motor2_speed_Callback(const std_msgs::Float32Ptr &msg)
{
    CB_speed = msg->data;
  ROS_INFO_STREAM("REEL_speed receive: " << CB_speed);
  Q_EMIT logging_CB_speed();
}

void QNode::motor3_speed_Callback(const std_msgs::Float32Ptr &msg)
{
    PF_speed = msg->data;
  ROS_INFO_STREAM("REEL_speed receive: " << PF_speed);
  Q_EMIT logging_PF_speed();
}

void QNode::motor4_speed_Callback(const std_msgs::Float32Ptr &msg)
{
    FH_speed = msg->data;
  ROS_INFO_STREAM("FH_speed receive: " << FH_speed);
  Q_EMIT logging_FH_speed();
}

void QNode::current1_Callback(const std_msgs::Float32Ptr &msg)
{
    REEL_current = msg->data;
  ROS_INFO_STREAM("REEL_current receive: " << REEL_current);
  Q_EMIT logging_REEL_current();
}

void QNode::current2_Callback(const std_msgs::Float32Ptr &msg)
{
    CB_current = msg->data;
  ROS_INFO_STREAM("CB_current receive: " << CB_current);
  Q_EMIT logging_CB_current();
}

void QNode::current3_Callback(const std_msgs::Float32Ptr &msg)
{
    PF_current = msg->data;
  ROS_INFO_STREAM("PF_current receive: " << PF_current);
  Q_EMIT logging_PF_current();
}

void QNode::current4_Callback(const std_msgs::Float32Ptr &msg)
{
    FH_current = msg->data;
  ROS_INFO_STREAM("FH_current receive: " << FH_current);
  Q_EMIT logging_FH_current();
}

void QNode::is_obstacle_Callback(const std_msgs::BoolPtr &msg)
{
  int is_obstacle = msg->data;
  ROS_INFO_STREAM("is obstacle: "<<is_obstacle);
  if(is_obstacle == true)
    Q_EMIT logging_is_obstacle();
  else
  {
    Q_EMIT logging_no_obstacle();
  }
}

void QNode::reap_height1_Callback(const std_msgs::Int64Ptr &msg)
{
  reap_height1 = msg->data;
  ROS_INFO_STREAM("angle1: "<<reap_height1);
  Q_EMIT logging_reap_height1();
}

void QNode::reap_height2_Callback(const std_msgs::Int64Ptr &msg)
{
  reap_height2 = msg->data;
  ROS_INFO_STREAM("angle2: "<<reap_height2);
  Q_EMIT logging_reap_height2();
}

void QNode::torque_Callback(const std_msgs::Float32Ptr &msg)
{
  torque = msg->data;
  ROS_INFO_STREAM("torque: "<<torque);
  Q_EMIT logging_torque();
}

void QNode::run()
{
  ROS_INFO_STREAM("running qnode.");
  //ros::Rate loop_rate(1);
  ros::Duration initDur(0.1);
  int count = 0;
  while (ros::ok())
  {
    //        std_msgs::String msg;
    //        std::stringstream ss;
    //        ss << "hello world -- " << count;
    //        msg.data = ss.str();
    //        chatter_publisher.publish(msg);
    //        log(Info,std::string("I sent: ")+msg.data);
    ros::spinOnce();
    //loop_rate.sleep();
    initDur.sleep();
    ++count;
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::ros_test(const std::string s)
{
  ROS_INFO_STREAM(s);
}

void QNode::pub_car_speed(float msg)
{

  std_msgs::Float32 car_speed;
  car_speed.data = msg;
  car_speed_pub.publish(car_speed);
}

void QNode::pub_car_turn(int msg)
{
  std_msgs::Int16 car_turn;
  car_turn.data = msg;
  car_turn_pub.publish(car_turn);
  std::cout<<"left_qtg"<<std::endl;

}

void QNode::pub_is_stop(int msg)
{
  std_msgs::Int16 is_stop;
  is_stop.data = msg;
  is_stop_pub.publish(is_stop);
}

void QNode::pub_height_control_mode(float msg) {
    std_msgs::UInt16 control_mode;
    control_mode.data = msg;
    height_control_mode_pub.publish(control_mode);
}

void QNode::log(const LogLevel &level, const std::string &msg)
{
  logging_model.insertRows(logging_model.rowCount(),1);
  std::stringstream logging_model_msg;
  switch (level)
  {
  case(Debug):
  {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Info):
  {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Warn):
  {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Error):
  {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Fatal):
  {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
}

void QNode::log_listen(const LogLevel &level, const std::string &msg)
{
  logging_listen.insertRows(logging_listen.rowCount(),1);
  std::stringstream logging_model_msg;
  switch (level)
  {
  case(Debug):
  {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Info):
  {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Warn):
  {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Error):
  {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case(Fatal):
  {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_listen.setData(logging_listen.index(logging_listen.rowCount()-1),new_row);
  Q_EMIT loggingListen(); // used to readjust the scrollbar
}

}  // namespace test_gui
