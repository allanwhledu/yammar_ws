#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <pnp_msgs/PNPAction.h>
#include <pnp_msgs/PNPCondition.h>
#include <pnp_ros/PNPActionServer.h>

#include "MyActions.h"
#include "reap_unit_action/ControlReapAction.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define radians(a) ((a)/180.0*M_PI)
int checkStart = 0;

int checkLeaderLine = 0;
int checkGrainHeight = 0;

int carSpeed = 500;
int speedChange = 0;

bool sign_stop = false;

int smach_fback = 0;


//int con_start(string params) {
//    int r = -1; // unknown
//
//    if (checkStart == 1) {
//        r = 1;
//    } else {
//        r = 0;
//    }
//    return r;
//}

int con_stop(string params) {
    int r = 0;
    return r;
}

int con_sys(string params) {
    int r = -1; // unknown

    if (checkLeaderLine == 1 && checkGrainHeight == 1) {
        r = 1;
    } else {
        r = 0;
    }
    return r;
}

//int con_speedchange(string params) {
//    int r = -1; // unknown
//
//    if (speedChange == 1) {
//        ROS_WARN_STREAM("Speed changed!");
//        r = 1;
//        speedChange = 0;
//    } else {
//        r = 0;
//    }
//    return r;
//}

//void startreap(string params, bool *run) {
//    cout << "### Executing startreap ... " << params << endl;
//    ros::NodeHandle globaln;
//    ros::Publisher pub_smach;
//    // Set turn topic
//    std_msgs::Int32 msg;
//    msg.data = 400;
//    pub_smach = globaln.advertise<std_msgs::Int32>("/sm_reset", 10);
//    pub_smach.publish(msg);
//
//    while (smach_fback != 1)
//    {
//        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
//        pub_smach.publish(msg);
//    }
//
//    if (*run)
//        cout << "### Finished startreap " << endl;
//    else
//        cout << "### Aborted startreap  " << endl;
//}
void startreap(string params, bool *run) {
#if 0
    cout << "\033[22;31;1m### Executing turn360 ... " << params << "\033[0m" << endl;

    cout << "HELLO FROM " << robotname << " !!!"<<endl;
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    if (*run)
        cout << "### Finished turn360 " << endl;
    else
        cout << "### Aborted turn360  " << endl;
#else

    int i=params.find("_");
    int motor_num=atof(params.substr(0,i).c_str());

    // Set turn topic
    std::string turn_topic = "control_reap";

    // Define the action client (true: we want to spin a thread)
    actionlib::SimpleActionClient<reap_unit_action::ControlReapAction> ac(turn_topic, true);

    // Wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for control_reap action server to come up");
    }

    // Cancel all goals (JUST IN CASE SOME GOAL WAS NOT CLOSED BEFORE)
    ac.cancelAllGoals();
//    ros::Duration(1).sleep(); // wait 1 sec

    int counter = 0;

    while (counter++ != 1)
    {
        // Set the goal
        reap_unit_action::ControlReapGoal goal;
        goal.dishwasher_id = motor_num;
        goal.target_speed = carSpeed;

        // Send the goal
        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        // Wait for termination
        while (!ac.waitForResult(ros::Duration(1.0))) {
            ROS_INFO_STREAM("Running... [" << ac.getState().toString() << "]");
        }
        ROS_INFO_STREAM("Finished [" << ac.getState().toString() << "]");

        // Print result
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Turn successful");
        else
            ROS_INFO("Turn failed");

        // Cancel all goals (NEEDED TO ISSUE NEW GOALS LATER)
        ac.cancelAllGoals();
//      ros::Duration(1).sleep(); // wait 1 sec
    }
#endif
}

void stopreap(string params, bool *run) {

    cout << "### Executing stopreap ... " << params << endl;
//    ros::NodeHandle globaln;
//    ros::Publisher pub_smach;
//    // Set turn topic
//    std_msgs::Int32 msg;
//    msg.data = 0;
//    pub_smach = globaln.advertise<std_msgs::Int32>("/sm_reset", 10);
//    pub_smach.publish(msg);
//
//    while (smach_fback != 1)
//    {
//        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
//        pub_smach.publish(msg);
//    }
//
//    if (*run)
//        cout << "### Finished stopreap " << endl;
//    else
//        cout << "### Aborted stopreap  " << endl;

#if 0
    cout << "\033[22;31;1m### Executing turn360 ... " << params << "\033[0m" << endl;

    cout << "HELLO FROM " << robotname << " !!!"<<endl;
    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

    if (*run)
        cout << "### Finished turn360 " << endl;
    else
        cout << "### Aborted turn360  " << endl;
#else

    int i=params.find("_");
    int motor_num=atof(params.substr(0,i).c_str());

    // Set turn topic
    std::string turn_topic = "control_reap";

    // Define the action client (true: we want to spin a thread)
    actionlib::SimpleActionClient<reap_unit_action::ControlReapAction> ac(turn_topic, true);

    // Wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for control_reap action server to come up");
    }

    // Cancel all goals (JUST IN CASE SOME GOAL WAS NOT CLOSED BEFORE)
    ac.cancelAllGoals();
//    ros::Duration(1).sleep(); // wait 1 sec

    int counter = 0;

    while (counter++ != 1)
    {
        // Set the goal
        reap_unit_action::ControlReapGoal goal;
        goal.dishwasher_id = motor_num;
        goal.target_speed = 0;

        // Send the goal
        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        // Wait for termination
        while (!ac.waitForResult(ros::Duration(1.0))) {
            ROS_INFO_STREAM("Running... [" << ac.getState().toString() << "]");
        }
        ROS_INFO_STREAM("Finished [" << ac.getState().toString() << "]");

        // Print result
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Turn successful");
        else
            ROS_INFO("Turn failed");

        // Cancel all goals (NEEDED TO ISSUE NEW GOALS LATER)
        ac.cancelAllGoals();
//      ros::Duration(1).sleep(); // wait 1 sec
    }

    if(motor_num == 37)
        sign_stop = true;
#endif
}


class MyPNPActionServer : public PNPActionServer
{
private:

    ros::NodeHandle handle;
    ros::Publisher event_pub;
    ros::Subscriber laser_sub;
    ros::Subscriber checkLeaderLine_sub;
    ros::Subscriber checkGrainHeight_sub;
    ros::Subscriber checkCANBus_sub;

    ros::Subscriber wait_sub;
    ros::Subscriber smach_fback_sub;

public:
    // 这里有三个内容,分别是：action,event,condition
    MyPNPActionServer() : PNPActionServer() {
        // interrupt才对应的event,如果是一个sensing则是对应的condition
        event_pub = handle.advertise<std_msgs::String>("PNPConditionEvent", 10);
        laser_sub = handle.subscribe("scan", 10, &MyPNPActionServer::laser_callback, this);

        checkCANBus_sub = handle.subscribe("CANBus", 10, &MyPNPActionServer::canbus_callback, this);

        checkLeaderLine_sub = handle.subscribe("LeaderLine", 10, &MyPNPActionServer::leaderline_callback, this);
        checkGrainHeight_sub = handle.subscribe("GrainHeight", 10, &MyPNPActionServer::grainheight_callback, this);

        wait_sub = handle.subscribe("Start", 10, &MyPNPActionServer::start_callback, this);
        smach_fback_sub = handle.subscribe("smach_fback", 10, &MyPNPActionServer::smach_callback, this);

        // robotname external defined in MyActions.h/cpp
        handle.param("robot_name",robotname,std::string(""));
        ROS_INFO("ROBOTNAME: %s",robotname.c_str());

        register_action("init",&ainit);
        register_action("gotopose",&gotopose);
        register_action("home",&home);
        register_action("wave",&wave);

        register_action("waitstart",&waitstart);
        register_action("syscheck",&syscheck);
        register_action("waiterror",&waiterror);

        register_action("startreap", &startreap);
        register_action("stopreap",&stopreap);
        register_action("controlcar",&controlcar);

        register_condition("closeToHome",&closeToHomeCond);
        register_condition("sys" , &con_sys);
//        register_condition("start" , &con_start);
        register_condition("stop" , &con_stop);
//        register_condition("speedchange" , &con_speedchange);
    }

    /*
    virtual int evalCondition(string cond)
    {
      if (cond == "closeToHome")
      {
        int res = closeToHomeCond();

        cerr << "\033[22;34;1mCloseToHome: " << ((res == 1) ? "true" : "false") << "\033[0m" << endl;

        return res;
      }

      return PNPActionServer::evalCondition(cond);
    }
    */

    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        std::vector<float> scans;
        scans=std::vector<float>(msg->ranges);
        if (scans[scans.size()/2]<1.0) {
            std_msgs::String cond;
            cond.data = "obstacle";
            event_pub.publish(cond);
        }
    }

    void start_callback(const std_msgs::Int8::ConstPtr& msg)
    {
        if(msg->data==1)
            checkStart = 1;
        else
            checkStart = 0;
    }

    void leaderline_callback(const std_msgs::Int8::ConstPtr& msg)
    {
        if(msg->data==1)
            checkLeaderLine = 1;
        else
            checkLeaderLine = 0;
    }

    void grainheight_callback(const std_msgs::Int8::ConstPtr& msg)
    {
        if(msg->data==1)
            checkGrainHeight = 1;
        else
            checkGrainHeight = 0;
    }

    void canbus_callback(const std_msgs::Int32::ConstPtr& msg)
    {
        ROS_WARN_STREAM("speed: "<<carSpeed);
        if(carSpeed != msg->data)
        {
            ROS_WARN_STREAM("speed changed.");
            std_msgs::String cond;
            cond.data = "speedchange";
            event_pub.publish(cond);
            ROS_WARN_STREAM("event pub.");
        }
        carSpeed = msg->data;
    }

    void smach_callback(const std_msgs::Int32::ConstPtr& msg)
    {
        smach_fback = msg->data;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mypnpas");

    MyPNPActionServer mypnpas;
    mypnpas.start();
    ros::spin();

    return 0;
}