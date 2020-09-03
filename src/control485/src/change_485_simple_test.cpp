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
//uint16_t motorSpeedFeedbackAddr=0x5F; //说明书中可以找到其为读取速度的地址
uint16_t motorSpeedFeedbackAddr=0x56; //对于摩托车版本的驱动器，可以找到其为读取速度的地址
uint16_t motorSpeedMode=0x49;
uint16_t motorRS485Adress=0x43;
uint16_t motorCurrentFeedbackAddr=0xC6; //说明书中找到而补充的电流读取，但是应该暂时不用（因为不精确吧）
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

void test_speed_control(int motor_id, int speed);

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
    usleep(5000);
    modbus_write_register(com,motorModbusAddr,1 );
    usleep(5000);
}

// 设置速度模式
void motorSetSpeedmode(int motor)
{
    modbus_set_slave(com,motor);
    usleep(5000);
    int flag = 0;
    flag = modbus_write_register(com,motorSpeedMode,1);
    ROS_INFO_STREAM("speed mode set result: "<<flag);
    usleep(5000);
}

// 改变从站地址
void motorSetAdress(int motor, int new_id)
{
    modbus_set_slave(com,motor);
    usleep(5000);
    int flag = 0;
    flag = modbus_write_register(com,motorRS485Adress,new_id);
    ROS_INFO_STREAM("id set result: "<<flag);
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
    int error_count = 0;
    int flag = -1;
    do {
        if(error_count > 5){
            ROS_INFO_STREAM("can't read motor's speed.");
            break;
        }
        usleep(5000);
        flag = modbus_read_registers(com, motorSpeedFeedbackAddr, 1, &temp);
        if (flag == -1) {
            cout << "error read motor" << motor << " speed." << endl;
            error_count++;
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
//    int new_motor_id = 1;
    int speed = 1200;

    ROS_INFO_STREAM(">>Open Serial!") ;
    if(openSerial(port.c_str()))
    {
        ROS_INFO_STREAM(">>Enable RS485...") ;
        motorSetModbus(motor_id);
//        motorSetAdress(motor_id, new_motor_id);
        test_speed_control(motor_id, speed);


        ROS_INFO_STREAM("Done!") ;

        ROS_INFO_STREAM(">>Exit!") ;
    }
    return 0;
}

void test_speed_control(int motor_id, int speed) {
    motorSetSpeedmode(motor_id);
    motorSetSpeed(motor_id, speed);

    usleep(5000000);
    ROS_INFO_STREAM(">>Current speed: " << motorReadSpeed(motor_id));
    usleep(5000000);
    motorSetSpeed(motor_id, 0);
}
