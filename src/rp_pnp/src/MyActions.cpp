#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/LaserScan.h>

#include <rp_action_msgs/TurnAction.h>

#include <boost/thread/thread.hpp>
#include <std_msgs/String.h>
#include <reap_unit_action/ControlReapAction.h>

#define radians(a) ((a)/180.0*M_PI)
extern int checkStart;
extern int checkLeaderLine;
extern int checkGrainHeight;

extern ros::NodeHandle* handle_ptr;
extern ros::Publisher* event_pub_ptr;
// 这里的机器人名称应该是可以自己更改的
std::string robotname="";

// defined in robotpose.cpp
bool getRobotPose(std::string robotname, double &x, double &y, double &th_rad);

// defined in gotopose.cpp
void exec_gotopose(std::string robotname, float GX, float GY, float GTh, bool *run);

using namespace std;

std::string turn_topic = "turn";
std::string movebase_topic = "move_base";

actionlib::SimpleActionClient<rp_action_msgs::TurnAction> *ac_turn = NULL;
actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac_movebase = NULL;



/*** ACTIONS ***/

void start_gotopose(float GX, float GY, float GTh, bool *run) {

    exec_gotopose(robotname, GX, GY, GTh, run);

}


// Action implementation

// ainit就是直接到达固定的起始点(2,2)就好了
void ainit(string params, bool *run) {
    cout << "### Executing Init ... " << params << endl;
    // Set turn topic

    float GX=2.0;
    float GY=2.0;
    float GTh=0;

    start_gotopose(GX, GY, GTh, run);

    if (*run)
        cout << "### Finished Init " << endl;
    else
        cout << "### Aborted Init  " << endl;
}

// 在petri网中动作名之后用_隔开的是传入参数,这些参数是goto的目标
void gotopose(string params, bool *run) {
    cout << "### Executing Gotopose ... " << params << endl;

    int i=params.find("_");
    float GX=atof(params.substr(0,i).c_str());
    int j=params.find("_",i+1);
    float GY=atof(params.substr(i+1,j).c_str());
    float GTh=atof(params.substr(j+1).c_str());

    start_gotopose(GX, GY, GTh, run);

    if (*run)
        cout << "### Finished Gotopose " << endl;
    else
        cout << "### Aborted Gotopose  " << endl;
}

void home(string params, bool *run)
{
    cout << "### Executing Home ... " << params << endl;

    float GX=2.0;
    float GY=2.0;
    float GTh=0;

    start_gotopose(GX, GY, GTh, run);

    if (*run)
        cout << "### Finished Home " << endl;
    else
        cout << "### Aborted Home  " << endl;
}

void wave(string params, bool *run) {
    cout << "### Executing Wave ... " << params << endl;

//    cout << "HELLO FROM " << robotname << " !!!"<<endl;
    // wave就是原地sleep而已
    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

    if (*run)
        cout << "### Finished Wave " << endl;
    else
        cout << "### Aborted Wave  " << endl;
}



void waitstart(string params, bool *run) {
    cout << "### Executing waistart ... " << params << endl;
    while(checkStart == 0)
    {
        ROS_INFO_STREAM("still wait start.");
        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
    }
    if (*run)
        cout << "### Finished waitstart " << endl;
    else
        cout << "### Aborted waitstart  " << endl;
}

void syscheck(string params, bool *run) {
    cout << "### Executing syscheck ... " << params << endl;
    while(checkLeaderLine == 0 || checkGrainHeight == 0)
    {
        ROS_INFO_STREAM("still check sys.");
        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
    }

    if (*run)
        cout << "### Finished syscheck " << endl;
    else
        cout << "### Aborted syscheck  " << endl;
}

void waiterror(string params, bool *run) {
    cout << "### Executing syscheck ... " << params << endl;
    while(checkLeaderLine == 1 && checkGrainHeight == 1 && (*run))
    {
        ROS_INFO_STREAM("still waiterror.");
        boost::this_thread::sleep(boost::posix_time::milliseconds(3000));
    }

    ROS_WARN_STREAM("car should stop.");
    std_msgs::String cond;
    cond.data = "carstop";
    event_pub_ptr->publish(cond);
    ROS_WARN_STREAM("carstop event pub.");

    if (*run)
        cout << "### Finished waiterror " << endl;
    else
        cout << "### Aborted waiterror  " << endl;
}

void controlcar(string params, bool *run) {
    cout << "### Executing controlcar ... " << params << endl;

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
        goal.dishwasher_id = 36;
        goal.target_speed = 1500;

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
            ROS_WARN_STREAM("controlcar successful");
        else
            ROS_WARN_STREAM("controlcar failed");

        // Cancel all goals (NEEDED TO ISSUE NEW GOALS LATER)
        ac.cancelAllGoals();
//      ros::Duration(1).sleep(); // wait 1 sec
    }
    // pnp应该是用了*run来中断吧,但是似乎不能够中断while循环，所以这里while应该用上*run控制
    int i = 100;
    while(i-- && (*run)){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        ROS_INFO_STREAM("control car.");
    }

    if (*run)
        cout << "### Finished controlcar " << endl;
    else
        cout << "### Aborted controlcar  " << endl;
}

void stopcar(string params, bool *run) {
    cout << "### Executing stopcar ... " << params << endl;

//    // pnp应该是用了*run来中断吧,但是似乎不能够中断while循环，所以这里while应该用上*run控制
//    int i = 50;
//    while(i-- && (*run)){
//        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
//        ROS_INFO_STREAM("stopcar.");
//    }

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
        goal.dishwasher_id = 36;
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

    boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
    ROS_WARN_STREAM("stopcar.");

    if (*run)
        cout << "### Finished stopcar " << endl;
    else
        cout << "### Aborted stopcar  " << endl;
}

void waitclean(string params, bool *run) {
    cout << "### Executing waitclean ... " << params << endl;

    // pnp应该是用了*run来中断吧,但是似乎不能够中断while循环，所以这里while应该用上*run控制
    int i = 100;
    while(i-- && (*run)){
        boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
        ROS_WARN_STREAM("wait clean.");
    }

    if (*run)
        cout << "### Finished waitclean " << endl;
    else
        cout << "### Aborted waitclean  " << endl;
}



/*** CONDITIONS ***/
// 这个函数调用getRobotPose函数确定机器人是否已经在home内,但是问题是谁调用了它？
int closeToHomeCond(string params) {
    int r = -1; // unknown
    double x, y, theta;

    if (getRobotPose(robotname,x,y,theta)) {
        if ((fabs(x - 2) <= 4) && (fabs(y - 2) <= 4)) {
            // cerr << "\033[22;34;1mCloseToHome\033[0m" << endl;
            r = 1;
        }
        else {
            r = 0;
        }
    }
    return r;
}
