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

    QString str;
    cv::Mat img;
    QImage image;
    QString text;
    float chart;
    float FH;
    double reap_height;

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
    void FHCallback(const std_msgs::Float32Ptr &msg);
    void is_obstacle_Callback(const std_msgs::BoolPtr &msg);
    void reap_height_Callback(const std_msgs::Int64Ptr &msg);
    void TextCallback(const height_border_msgs::height_borderConstPtr &msg);
    QStringListModel* loggingModelLis()
        {return &logging_listen;}
    void log_listen(const LogLevel &level, const std::string &msg);

    void ros_test(const std::string s);

Q_SIGNALS:
    void loggingListen();
    void rosShutdown();
    void loggingCamera();
    void loggingText();
    void loggingChart();
    void loggingFH();
    void logging_is_obstacle();
    void logging_no_obstacle();
    void logging_reap_height();

private:
    int init_argc;
    char** init_argv;
    ros::Subscriber chatter_subscriber;
    ros::Subscriber text_subscriber;
    ros::Subscriber chart_subscriber;
    ros::Subscriber FH_subscriber;
    ros::Subscriber obstacle_subscriber;
    ros::Subscriber reap_height_subscriber;

    QStringListModel logging_model;
    QStringListModel logging_listen;
    image_transport::Subscriber image_sub;
};

}  // namespace test_gui

#endif /* test_gui_QNODE_HPP_ */
