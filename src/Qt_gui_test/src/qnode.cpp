﻿/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2018
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <qnode.h>
#include "sensor_msgs/image_encodings.h"

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
    ros::NodeHandle nSub;

    ros::NodeHandle im;
    image_transport::ImageTransport it(im);
    // Add your ros communications here.
    chatter_publisher = n.advertise<std_msgs::String>("testgui_chat", 1000);
    chatter_subscriber = nSub.subscribe("testgui_chat", 100, &QNode::RecvTopicCallback, this);
    image_sub = it.subscribe("/boud_depth",100,&QNode::myCallback_img,this);//相机尝试

    //ros::spin();
    start();
    return true;
}

void QNode::RecvTopicCallback(const std_msgs::StringConstPtr &msg)
{
    log_listen(Info, std::string("I heard: ")+msg->data.c_str());
}

void QNode::run()
{
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
