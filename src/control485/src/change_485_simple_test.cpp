//
// Created by sjtu_wanghaili on 2020/6/4.
//

#include<iostream>
#include<string>
#include<unistd.h>  /* UNIX standard function definitions */
#include<sys/time.h>    //for time
#include<modbus.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>


using namespace std;
modbus_t* com;//com用于电机速度控制反馈
uint16_t motorModbusAddr=0xB6; //0xB6在说明书中用于使能电机的rs485功能
uint16_t motorDirectionAddr=0x66; //在说明书中找到在0x66中访问数据0x01是正转，0x02是反转
uint16_t motorSpeedAddr=0x56; //在说明书中找到，0x56中设置电机的转速
uint16_t motorSpeedFeedbackAddr=0x5F; //说明书中可以找到其为读取速度的地址
uint16_t motorCurrentFeedbackAddr=0xC6; //说明书中找到而补充的电流读取，但是应该暂时不用（因为不精确吧）
uint16_t motorIDAddr=0x43; //这是在网上找到的，设置从站地址
// 以上，就是现在用到的寄存器地址

string port="/dev/ttyUSB0";

// 函数申明
bool openSerial(const char* port);
void motorInit(void);
void motorSetModbus(int motor,int enable);
void motorSetDirection(int motor,int dir);
void motorSetSpeed(int motor,int speed);
int motorReadSpeed(int motor);
int motorReadSpeed(int motor);

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
void motorSetModbus(int motor)
{
    modbus_set_slave(com,motor); //这句话的意思是不是将某从机设置为当前要访问的对象？
    modbus_write_register(com,motorModbusAddr,1 );
    usleep(5000);
}

void motorSetSpeed(int motor_id, int speed)
{
    modbus_set_slave(com, motor_id);
    usleep(5000);
    ROS_INFO_STREAM("set direction: "<<modbus_write_register(com, motorDirectionAddr, 1));
    usleep(5000);
    ROS_INFO_STREAM("set speed: "<<modbus_write_register(com, motorSpeedAddr, speed));
    usleep(5000);
}

int motorReadSpeed(int motor)
{
    uint16_t temp=-1000;
    modbus_set_slave(com,motor);
    int flag = -1;
    do {
        usleep(5000);
        flag = modbus_read_registers(com, motorSpeedFeedbackAddr, 1, &temp);
        if (flag == -1) {
            cout << "error read motor" << motor << " speed." << endl;
        } else {
            cout << "succeed read motor" << motor << " speed." << endl;
        }
    } while (flag == -1);
    return temp;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "hello") ;
    ros::NodeHandle n_;

    int motor_id = 1;
    int speed = 500;

    ROS_INFO_STREAM(">>Open Serial!") ;
    if(openSerial(port.c_str()))
    {
        ROS_INFO_STREAM(">>Enable RS485...") ;
        motorSetModbus(motor_id);
        motorSetSpeed(motor_id, speed);

        usleep(2000000);
        ROS_INFO_STREAM(">>Current speed: "<<motorReadSpeed(motor_id));
        ROS_INFO_STREAM("Done!") ;

        ROS_INFO_STREAM(">>Exit!") ;
    }
    return 0;
}
