//
// Created by sjtu_wanghaili on 2020/6/4.
//

#include<iostream>
#include<string>
#include<unistd.h>  /* UNIX standard function definitions */
#include<fcntl.h>   /* File control definitions */
#include<termios.h> /* POSIX terminal control definitions */
#include<sys/time.h>    //for time
#include<modbus.h>
#include<fstream>
#include<vector>
#include<algorithm>
#include<signal.h>
#include<pthread.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include <actionlib/server/simple_action_server.h>
#include "control485/DriveMotorAction.h"


using namespace std;

// 初始化变量
struct harvesterSpeed
{
    double linear=0.0;
    double rotate=0.0;
};
modbus_t* com;//com用于电机速度控制反馈
uint16_t motorModbusAddr=0xB6; //0xB6在说明书中用于使能电机的rs485功能
uint16_t motorDirectionAddr=0x66; //在说明书中找到在0x66中访问数据0x01是正转，0x02是反转
uint16_t motorSpeedAddr=0x56; //在说明书中找到，0x56中设置电机的转速
uint16_t motorSpeedFeedbackAddr=0x5F; //说明书中可以找到其为读取速度的地址
uint16_t motorCurrentFeedbackAddr=0xC6; //说明书中找到而补充的电流读取，但是应该暂时不用（因为不精确吧）

uint16_t motorMODBUSAddr=0x43; //这是在网上找到的，设置从站地址
// 以上，就是现在用到的寄存器地址


double cbCof=1.2,reelCof=1.6,pfCof=4.44,fhCof=3.94;//同调率
int cbRatio=5,reelRatio=64,pfRatio=15,fhRatio=10;//减速比
const int reelMotor=1,cbMotor=2,pfMotor=3,fhMotor=4;
string port="/dev/ttyUSB0";
harvesterSpeed carSpeed;
bool is_obstacle = false;
bool is_stop = false;
ros::Publisher* pub_modified_car_speed;
ros::Publisher* pub_reel_speed;
ros::Publisher* pub_cb_speed;
ros::Publisher* pub_pf_speed;
ros::Publisher* pub_fh_speed;

std_msgs::Float32 modified_car_speed;
float last_modified_car_speed = 0;

// 函数申明
bool openSerial(const char* port);
string getTime(void);//获取当前系统时间
void motorInit(void);
void motorSetModbus(int motor,int enable);
void motorSetDirection(int motor,int dir);
void motorSetSpeed(int motor,int speed);
int motorReadSpeed(int motor);
vector<double> motorReadCurrent(void);
pair<double,double> carReadSpeed(void);
void* carSpeedFollowMode(void*);
int getCBSpeed(double carSpeed);
int getReelSpeed(double carSpeed);
int getFHSpeed(double carSpeed);
int getPFSpeed(double carSpeed);
void currentTest(void);
void test(void);
void heightRecord(void);
void carSpeedTest(void);
void singleMotorCurrentRecord(int motor);
void test1(void);
void rotate(void);
void rotateR(void);

bool openSerial(const char* port)
{
    com=modbus_new_rtu(port,9600,'N',8,1);
    if(com==nullptr)
    {
        cout<<"wrong modbus parameter."<<endl;
        return false;
    }
    timeval time_out;
    time_out.tv_sec=0;
    time_out.tv_usec=1000*100;
    modbus_set_response_timeout(com,time_out.tv_sec,time_out.tv_usec);
    modbus_rtu_set_serial_mode(com,MODBUS_RTU_RS485);
    if(modbus_connect(com)==-1)
    {
        cout<<"Cannot connect modbus at port:"<<port<<endl;
        return false;
    } else
        cout<<"Connected modbus at port:"<<port<<endl;
    return true;
}

// 使能某电机的rs485通讯
void motorSetModbus(int motor,int enable)
{
    modbus_set_slave(com,motor); //这句话的意思是不是将某从机设置为当前要访问的对象？
    modbus_write_register(com,motorModbusAddr,enable);
//    usleep(3000);
}

void motorSetID(int pre_id,int cur_id)
{
    modbus_set_slave(com,pre_id);
    modbus_write_register(com,motorMODBUSAddr,cur_id);
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "hello") ;
    ros::NodeHandle n_;

    int pre_id = 1;
    int cur_id = 2;

    ROS_INFO_STREAM(">>Open Serial!") ;
    if(openSerial(port.c_str()))
    {
        motorSetModbus(pre_id,1);

        ROS_INFO_STREAM(">>Change ID address!") ;
        motorSetID(pre_id,cur_id);
        ROS_INFO_STREAM("Done!") ;

        ROS_INFO_STREAM(">>Exit!") ;
    }
    return 0;
}
