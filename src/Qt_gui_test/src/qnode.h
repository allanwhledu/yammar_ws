/**
 * @file /include/test_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2018
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/
#ifndef test_gui_QNODE_HPP_
#define test_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#include <ros/ros.h>
#include "ros/subscriber.h"
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <QImage>
#include <QString>

#include "height_border_msgs/height_border.h"

/*****************************************************************************
** Namespaces
*****************************************************************************/
namespace test_gui
{

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
Q_OBJECT
public:
    QNode(int argc, char** argv);
    virtual ~QNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();
    void myCallback_img(const sensor_msgs::ImageConstPtr& msg);//camera callback function

    ros::Publisher car_speed_pub;
    void pub_car_speed(float msg);

    ros::Publisher is_stop_pub;
    void pub_is_stop(int msg);

    ros::Publisher force_stop_pub;
    void pub_force_stop(bool msg);

    ros::Publisher car_turn_pub;
    void  pub_car_turn(int msg);

    ros::Publisher hmi_ready_pub;
    void  pub_hmi_ready(int msg);

    ros::Publisher height_control_mode_pub;
    void pub_height_control_mode(float msg);

    QString str;
    cv::Mat img;
    QImage image;
    QString leader_line_error_string;
    float leader_line_error;
    QString height_string;
    float height_value;

    float chart;
    float speed3 = 0;
    float speed4 = 0;
    float speed2 = 0;
    float speed1 = 0;
    float speed5 = 0;
    float speed7 = 0;
    float speed8 = 0;
    float speed11 = 0;
    float speed9 = 0;
    float speed10 = 0;

    float current3 = 0;
    float current4 = 0;
    float current2 = 0;
    float current1 = 0;
    float current5 = 0;
    float current7 = 0;
    float current8 = 0;
    float current11 = 0;
    float current9 = 0;
    float current10 = 0;

    double reap_height1;
    double reap_height2;
    double torque;

    /*********************
    ** Logging
    **********************/
    enum LogLevel
    {
        Debug,
        Info,
        Warn,
        Error,
        Fatal
    };

    QStringListModel* loggingModel()
        {return &logging_model;}
    void log( const LogLevel &level, const std::string &msg);

    void ChartCallback(const std_msgs::Float32Ptr &msg);
    void s3_Callback(const std_msgs::Float32Ptr &msg);
    void s4_Callback(const std_msgs::Float32Ptr &msg);
    void s2_Callback(const std_msgs::Float32Ptr &msg);
    void s1_Callback(const std_msgs::Float32Ptr &msg);
    void s5_Callback(const std_msgs::Float32Ptr &msg);
    void s7_Callback(const std_msgs::Float32Ptr &msg);
    void s8_Callback(const std_msgs::Float32Ptr &msg);
    void s11_Callback(const std_msgs::Float32Ptr &msg);
    void s9_Callback(const std_msgs::Float32Ptr &msg);
    void s10_Callback(const std_msgs::Float32Ptr &msg);

    void c3_Callback(const std_msgs::Float32Ptr &msg);
    void c4_Callback(const std_msgs::Float32Ptr &msg);
    void c2_Callback(const std_msgs::Float32Ptr &msg);
    void c1_Callback(const std_msgs::Float32Ptr &msg);
    void c5_Callback(const std_msgs::Float32Ptr &msg);
    void c7_Callback(const std_msgs::Float32Ptr &msg);
    void c8_Callback(const std_msgs::Float32Ptr &msg);
    void c11_Callback(const std_msgs::Float32Ptr &msg);
    void c9_Callback(const std_msgs::Float32Ptr &msg);
    void c10_Callback(const std_msgs::Float32Ptr &msg);

    void is_obstacle_Callback(const std_msgs::BoolPtr &msg);
    void reap_height1_Callback(const std_msgs::Int64Ptr &msg);
    void reap_height2_Callback(const std_msgs::Int64Ptr &msg);
    void torque_Callback(const std_msgs::Float32Ptr &msg);
    void height_border_Callback(const height_border_msgs::height_borderConstPtr &msg);
    QStringListModel* loggingModelLis()
        {return &logging_listen;}
    void log_listen(const LogLevel &level, const std::string &msg);

    void ros_test(const std::string s);

Q_SIGNALS:
    void loggingListen();
    void rosShutdown();
    void loggingCamera();
    void logging_leader_line_error();
    void loggingChart();
    void logging_speed_3();
    void logging_speed_4();
    void logging_speed_2();
    void logging_speed_1();
    void logging_speed_5();
    void logging_speed_7();
    void logging_speed_8();
    void logging_speed_11();
    void logging_speed_9();
    void logging_speed_10();

    void logging_current_3();
    void logging_current_4();
    void logging_current_2();
    void logging_current_1();
    void logging_current_5();
    void logging_current_7();
    void logging_current_8();
    void logging_current_11();
    void logging_current_9();
    void logging_current_10();

//    void logging_REEL_current();
//    void logging_CB_current();
//    void logging_PF_current();
//    void logging_FH_current();

    void logging_is_obstacle();
    void logging_no_obstacle();
    void logging_reap_height1();
    void logging_reap_height2();
    void logging_torque();

private:
    int init_argc;
    char** init_argv;
    ros::Subscriber chatter_subscriber;
    ros::Subscriber text_subscriber;
    ros::Subscriber chart_subscriber;
    ros::Subscriber s3_subscriber;
    ros::Subscriber s4_subscriber;
    ros::Subscriber s2_subscriber;
    ros::Subscriber s1_subscriber;
    ros::Subscriber s5_subscriber;
    ros::Subscriber s7_subscriber;
    ros::Subscriber s8_subscriber;
    ros::Subscriber s11_subscriber;
    ros::Subscriber s9_subscriber;
    ros::Subscriber s10_subscriber;

    ros::Subscriber c3_subscriber;
    ros::Subscriber c4_subscriber;
    ros::Subscriber c2_subscriber;
    ros::Subscriber c1_subscriber;
    ros::Subscriber c5_subscriber;
    ros::Subscriber c7_subscriber;
    ros::Subscriber c8_subscriber;
    ros::Subscriber c11_subscriber;
    ros::Subscriber c9_subscriber;
    ros::Subscriber c10_subscriber;

    ros::Subscriber obstacle_subscriber;
    ros::Subscriber reap_height1_subscriber;
    ros::Subscriber reap_height2_subscriber;
    ros::Subscriber torque_subscriber;

    QStringListModel logging_model;
    QStringListModel logging_listen;
    image_transport::Subscriber image_sub;
};

}  // namespace test_gui

#endif /* test_gui_QNODE_HPP_ */
