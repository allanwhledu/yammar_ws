//
// Created by sjtu_wanghaili on 2020/6/4.
//

#include<iostream>
#include<string>
#include<unistd.h>  /* UNIX standard function definitions */
#include<sys/time.h>    //for time
#include<modbus.h>

#include <fstream>
#include <pthread.h>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int64.h"


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

string port="/dev/rs485-01";

string current_time = "";
ofstream* open_file;

float motorCurrentRaw = 0;
float motorCurrentRms = 0;
float motorCurrentCM7290 = 0;

bool endFlag = false;

// 函数申明
bool openSerial(const char* port);
void motorInit(void);
void motorSetModbus(int motor,int enable);
void motorSetDirection(int motor,int dir);
void motorSetSpeed(int motor,int speed);
int motorReadSpeed(int motor);
int motorReadSpeed(int motor);

void test_speed_control(int motor_id, int speed);

void* getTime(void*);//获取当前系统时间

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

// 设置rque模式
void motorSetTorqueMode(int motor)
{
    modbus_set_slave(com,motor);
    usleep(5000);
    int flag = 0;
    flag = modbus_write_register(com,motorSpeedMode,0);
    ROS_INFO_STREAM("torque mode set result: "<<flag);
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

void current_raw_cb(const std_msgs::Float32ConstPtr &msg){
    ROS_INFO_STREAM("Call back! current:"<<msg->data);
    motorCurrentRaw = msg->data;
}

void current_rms_cb(const std_msgs::Float32ConstPtr &msg){
    ROS_INFO_STREAM("Call back! true_current:"<<msg->data);
    motorCurrentRms = msg->data;
}

void current_cm7290_cb(const std_msgs::Float32ConstPtr &msg){
    ROS_INFO_STREAM("Call back! true_current:"<<msg->data);
    motorCurrentCM7290 = msg->data;
}

int main (int argc, char **argv)
{
    ros::init(argc, argv, "motor_current_read") ;
    ros::NodeHandle n_;

    ros::Publisher pub_motor_speed;
    ros::Subscriber sub_motor_current_raw;
    ros::Subscriber sub_motor_current_rms;
    ros::Subscriber sub_motor_current_cm7290;

    pub_motor_speed = n_.advertise<std_msgs::Int64>("motor_test_speed", 1000);
    std_msgs::Int64 motorTestSpeed;

    //Subscribe topic of current
    sub_motor_current_raw = n_.subscribe("current_raw", 10, &current_raw_cb);
    sub_motor_current_rms = n_.subscribe("current_rms", 10, &current_rms_cb);
    sub_motor_current_cm7290 = n_.subscribe("current_cm7290", 10, &current_cm7290_cb);

    pthread_t time_sync;
//    pthread_t speed_read;
    pthread_create(&time_sync, nullptr, getTime, nullptr);
    ROS_INFO_STREAM("time sync spread make.");

    ofstream ofs;
    string filename = "/home/yangzt/yammar_ws/src/control485/speed_result/";
    filename = filename + current_time + "m10_500.txt";

    int motor_id = 3;
    int realSpeed = 0;

    ROS_INFO_STREAM(">>Open Serial!") ;

    openSerial(port.c_str());
    motorSetModbus(motor_id);
    motorSetSpeedmode(motor_id);
//    motorSetTorqueMode(motor_id);
    motorSetSpeed(motor_id, 1500);

    float sec_count = 0.0, sec_duration = 360.0;
    while (ros::ok() && sec_count <= sec_duration)
    {
        // Read and save motor speed;
        realSpeed = motorReadSpeed(motor_id);
        motorTestSpeed.data = realSpeed;
        pub_motor_speed.publish(motorTestSpeed);
        // Update current
        ros::spinOnce();
        ofs.open(filename, ios_base::app);
        if(!ofs)
            cerr<<"Open File Fail."<<endl;
//            exit(1);
        ofs<<"Time: "<<current_time<<" MotorRev currentRaw currentRms currentCM7290 "<<realSpeed<<" "<<motorCurrentRaw<<" "
        <<motorCurrentRms<<" "<<motorCurrentCM7290<<endl;
        ofs.close();

        cout<<"MotorRev currentRaw currentRms currentCM7290 "<<realSpeed<<" "<<motorCurrentRaw<<" "
            <<motorCurrentRms<<" "<<motorCurrentCM7290<<endl;

        usleep(5000);
        sec_count += 0.033;
    }
//    sleep(1);
    motorSetSpeed(motor_id, 0);
    sleep(2);
    ROS_INFO_STREAM(">>Exit!") ;
    pthread_kill(time_sync, 0);

    return 0;
}

void test_speed_control(int motor_id, int speed) {
    motorSetSpeedmode(motor_id);
    motorSetSpeed(motor_id, speed);

    usleep(3000000);
    ROS_INFO_STREAM(">>Current speed: " << motorReadSpeed(motor_id));
    usleep(300000);
//    motorSetSpeed(motor_id, 0);
}

void* getTime(void*)
{
    // 时间在后台20ms更新一次
    while(!endFlag)
    {
        timeval tv;
        time_t timep;
        tm* timeNow;
        gettimeofday(&tv, NULL);//获取当下精确的s和us的时间
        time(&timep);//获取从1900年至今的秒数
        timeNow = gmtime(&timep); //注意tm_year和tm_mon的转换后才是实际的时间
        timeNow->tm_year+=1900;//实际年
        timeNow->tm_mon+=1;//实际月
        timeNow->tm_hour+=8;//实际小时
        if(timeNow->tm_hour>=24)
        {
            timeNow->tm_hour-=24;
        }
        long int ms = (tv.tv_sec*1000.0 + tv.tv_usec / 1000.0) - timep * 1000.0; //实际ms

        current_time ="";
        current_time+='[';
        current_time+=to_string(timeNow->tm_year);
        current_time+='-';
        current_time+=to_string(timeNow->tm_mon);
        current_time+='-';
        current_time+=to_string(timeNow->tm_mday);
        current_time+=' ';
        current_time+=to_string(timeNow->tm_hour);
        current_time+=':';
        current_time+=to_string(timeNow->tm_min);
        current_time+=':';
        string s_string = to_string(timeNow->tm_sec);
        while (s_string.size()<2)
        {
            s_string="0"+s_string;
        }
        current_time+=s_string;
        current_time+=':';
        string ms_string = to_string(int(ms));
        while (ms_string.size()<3)
        {
            ms_string="0"+ms_string;
        }
        current_time+=ms_string;
        current_time+=']';

//        ROS_WARN_STREAM("current time: "<<current_time);
        // todo 获取时间的函数好像执行很慢，感觉可以之后开一个线程让它在后台慢慢执行
        usleep(20000);
    }
    ROS_WARN_STREAM("current time sync stopped.");
//    endFlag = false;
}
