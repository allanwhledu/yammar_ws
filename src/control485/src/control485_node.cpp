//
// Created by sjtu_wanghaili on 2020/6/4.
//

#include<iostream>
#include<string>
#include<unistd.h>  //UNIX standard function definitions
#include<sys/time.h>  //for time
#include<modbus.h>
#include<fstream>  //读写文件的头文件
#include<algorithm>
#include<signal.h>
#include<pthread.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include "std_msgs/Float32.h"
#include <actionlib/server/simple_action_server.h>
#include "control485/DriveMotorAction.h"

using namespace std;

// 初始化常量
uint16_t motorModbusAddr=0xB6; //0xB6在说明书中用于使能电机的rs485功能
uint16_t motorDirectionAddr=0x66; //在说明书中找到在0x66中访问数据0x01是正转，0x02是反转
uint16_t motorSpeedAddr=0x56; //在说明书中找到，0x56中设置电机的转速
//uint16_t motorSpeedFeedbackAddr=0x5F; //说明书中可以找到其为读取速度的地址
uint16_t motorSpeedFeedbackAddr=0x56; //对于摩托车版本的驱动器，可以找到其为读取速度的地址
uint16_t motorSpeedMode=0x49;
uint16_t motorRS485Adress=0x43;
uint16_t motorCurrentFeedbackAddr=0xC6; //说明书中找到而补充的电流读取，但是应该暂时不用（因为不精确吧）
double cbCof=1.2,reelCof=1.6,pfCof=4.44,fhCof=3.94; //同调率
int cbRatio=5,reelRatio=64,pfRatio=15,fhRatio=10; //减速比
const int reelMotor=1,cbMotor=2,pfMotor=3,fhMotor=4;
string port="/dev/ttyUSB0";


// 初始化变量
struct harvesterSpeed
{
    double linear=0.0;
    double rotate=0.0;
};
harvesterSpeed carSpeed;
std_msgs::Float32 modified_car_speed;
float last_modified_car_speed = 0;
bool is_obstacle = false;
bool is_stop = false;
bool endFlag = false;
modbus_t* com; //com用于电机速度控制反馈
string current_time = "";
ofstream* open_file;
bool rs485_busy = false;

// 初始化publisher
ros::Publisher* pub_modified_car_speed;
ros::Publisher* pub_reel_speed;
ros::Publisher* pub_cb_speed;
ros::Publisher* pub_pf_speed;
ros::Publisher* pub_fh_speed;


// 函数申明
bool openSerial(const char* port);
void* getTime(void*);//获取当前系统时间
void motorInit(void);
void motorSetModbus(int motor,int enable);
void motorSetDirection(int motor,int dir);
void motorSetSpeed(int motor,int speed);
int motorReadSpeed(int motor);
void carspeed_callback(const std_msgs::Float32ConstPtr &msg);
void obstacle_callback(const std_msgs::BoolConstPtr &msg);
void manual_stop_callback(const std_msgs::BoolConstPtr &msg);
void* carSpeedFollowMode(void*);
void test(void);

// 获取时间
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
    ROS_WARN_STREAM("current time sync stoped.");
//    endFlag = false;
}
void* carSpeedFollowMode(void*)
{
    while(!endFlag)
    {
        if(is_stop || is_obstacle){
            modified_car_speed.data = -1;
        } else{
            modified_car_speed.data = carSpeed.linear;
        }

        if(modified_car_speed.data != last_modified_car_speed){
            ROS_WARN_STREAM("modified speed will be change.");
        }
        pub_modified_car_speed->publish(modified_car_speed);
        last_modified_car_speed = modified_car_speed.data;
    }
    ROS_WARN_STREAM("carspeedfollow stoped.");
//    endFlag = false;
}

void *read_motor_speed_background(void *) {
    int motor_id_reel = 1;
    int motor_id_cb = 2;
    int motor_id_pf = 3;
    int motor_id_fh = 4;
    int realSpeed_reel = 0;
    int realSpeed_cb = 0;
    int realSpeed_pf = 0;
    int realSpeed_fh = 0;

    while (!endFlag) {
        if (!rs485_busy)
        {
            // Read and pub motor speed;
            realSpeed_reel = motorReadSpeed(motor_id_reel);
            std_msgs::Float32 reel_speed;
            reel_speed.data = realSpeed_reel;
            pub_reel_speed->publish(reel_speed);

            usleep(40000);
            realSpeed_cb = motorReadSpeed(motor_id_cb);
            std_msgs::Float32 cb_speed;
            cb_speed.data = realSpeed_cb;
            pub_cb_speed->publish(cb_speed);

            usleep(40000);
            realSpeed_pf = motorReadSpeed(motor_id_pf);
            std_msgs::Float32 pf_speed;
            pf_speed.data = realSpeed_pf;
            pub_pf_speed->publish(pf_speed);

//            usleep(40000);
//            realSpeed_fh = motorReadSpeed(motor_id_fh);
//            std_msgs::Float32 fh_speed;
//            fh_speed.data = realSpeed_fh;
//            pub_fh_speed->publish(fh_speed);

            usleep(100000); // 250ms ==> 4hz, 每个电机每秒1次
            ROS_INFO_STREAM("back ground speed.");
        }
        else
            usleep(100000);
    }
}

// 测试action
typedef actionlib::SimpleActionServer<control485::DriveMotorAction> Server;
void execute(const control485::DriveMotorGoalConstPtr &goal, Server *as) {
    ROS_INFO("Start Motor %d.", goal->motor_id);
    rs485_busy = true;
    int target_speed = 0;
    int actual_speed = -1000;

    // 计算目标速度，读取真实速度
    target_speed = goal->target_speed;
    actual_speed = motorReadSpeed(goal->motor_id);

    ROS_INFO_STREAM("the current speed is: "<<actual_speed);
    ROS_INFO_STREAM("the target speed is: "<<target_speed);
    ROS_INFO_STREAM("the difference of speed still: "<<abs(target_speed - actual_speed));

    motorSetSpeed(goal->motor_id, target_speed);

    int count = 0;
    bool speed_ok = false;
    while (count < 500) {
        usleep(20000);
        actual_speed = motorReadSpeed(goal->motor_id);
        ROS_INFO_STREAM("reed speed onground");

        // 记录速度
        string actual_speed_str = to_string(actual_speed);
        while (actual_speed_str.size()<4)
        {
            actual_speed_str="0"+actual_speed_str;
        }
        *(open_file) << current_time << " " << goal->motor_id << " " <<actual_speed_str << " " << carSpeed.linear << endl;

        if (abs(actual_speed - target_speed) < 200)
        {
            ROS_WARN_STREAM("Speed is ok.");
            switch (goal->motor_id) {
                case 1:
                {
                    ROS_INFO_STREAM("pub reel speed.");
                    std_msgs::Float32 reel_speed;
                    reel_speed.data = actual_speed;
                    pub_reel_speed->publish(reel_speed);
                    break;
                }
                case 2:
                {
                    ROS_INFO_STREAM("pub cb speed.");
                    std_msgs::Float32 cb_speed;
                    cb_speed.data = actual_speed;
                    pub_cb_speed->publish(cb_speed);
                    break;
                }
                case 3:
                {
                    ROS_INFO_STREAM("pub pf speed.");
                    std_msgs::Float32 pf_speed;
                    pf_speed.data = actual_speed;
                    pub_pf_speed->publish(pf_speed);
                    break;
                }
                case 4:
                {
                    ROS_INFO_STREAM("pub fh speed.");
                    std_msgs::Float32 fh_speed;
                    fh_speed.data = actual_speed;
                    pub_fh_speed->publish(fh_speed);
                    break;
                }
            }
            speed_ok = true;
            break;
        }
        else
        {
            // 发布速度（未达要求的）
            ROS_WARN_STREAM("Speed is BAD.");
            switch (goal->motor_id) {
                case 1:
                {
                    ROS_INFO_STREAM("pub reel speed.");
                    std_msgs::Float32 reel_speed;
                    reel_speed.data = actual_speed;
                    pub_reel_speed->publish(reel_speed);
                    break;
                }
                case 2:
                {
                    ROS_INFO_STREAM("pub cb speed.");
                    std_msgs::Float32 cb_speed;
                    cb_speed.data = actual_speed;
                    pub_cb_speed->publish(cb_speed);
                    break;
                }
                case 3:
                {
                    ROS_INFO_STREAM("pub pf speed.");
                    std_msgs::Float32 pf_speed;
                    pf_speed.data = actual_speed;
                    pub_pf_speed->publish(pf_speed);
                    break;
                }
                case 4:
                {
                    ROS_INFO_STREAM("pub fh speed.");
                    std_msgs::Float32 fh_speed;
                    fh_speed.data = actual_speed;
                    pub_fh_speed->publish(fh_speed);
                    break;
                }
            }

            usleep(20000);
            motorSetSpeed(goal->motor_id, target_speed);
        }
        count++;
    }

    if(speed_ok)
    {
        ROS_INFO_STREAM("motor control succeeded!");
        as->setSucceeded();
        rs485_busy = false;
    } else
    {
        ROS_WARN_STREAM("motor control FAILED!");
        as->setSucceeded();
        rs485_busy = false;
    }

    ROS_INFO_STREAM("motor control complete. Wait for next invoke.\n");
}


// rs485通讯
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
void motorInit(void)
{
    // 只有这里才打开了电机，这里首先仅仅开启了reel电机
    ROS_WARN_STREAM("init reelmotor...");
    motorSetModbus(reelMotor,1);
    motorSetDirection(reelMotor,1);//正转
//    motorSetSpeed(reelMotor,0);

    ROS_WARN_STREAM("init cbmotor...");
    motorSetModbus(cbMotor,1);
    motorSetDirection(cbMotor,1);//正转
//    motorSetSpeed(cbMotor,0);

    ROS_WARN_STREAM("init pfmotor...");
    motorSetModbus(pfMotor,1);
    motorSetDirection(pfMotor,2);//正转
//    motorSetSpeed(pfMotor,0);

    ROS_WARN_STREAM("init fhmotor...");
    motorSetModbus(fhMotor,1);
    motorSetDirection(fhMotor,1);//正转
//    motorSetSpeed(fhMotor,0);
}
void motorSetModbus(int motor,int enable)
{
    modbus_set_slave(com,motor); //这句话的意思是不是将某从机设置为当前要访问的对象？
    usleep(20000);
    while (!modbus_write_register(com,motorModbusAddr,enable))
    {
        ROS_WARN_STREAM(motor<<"set modbus failed.");
        usleep(20000);
    }
}
void motorSetDirection(int motor,int dir)
{
    modbus_set_slave(com,motor);
    usleep(20000);
    while (!modbus_write_register(com,motorDirectionAddr,dir))
    {
        ROS_WARN_STREAM(motor<<"set direction failed, try again.");
        usleep(20000);
    }
}
void motorSetSpeed(int motor,int speed)
{
    if(speed>3000)
    {
        speed=3000;
    }
    if(speed<0)
    {
        speed=0;
    }

    modbus_set_slave(com,motor);
//    ROS_WARN_STREAM("set speed result: "<<modbus_write_register(com,motorSpeedAddr,speed));
    usleep(20000);
    while (!modbus_write_register(com,motorSpeedAddr,speed))
    {
        ROS_WARN_STREAM(motor<<"set speed failed， try again.");
        usleep(20000);
    }
}
int motorReadSpeed(int motor)
{
//    ROS_INFO_STREAM("Will read which motor speed: "<<motor);
    uint16_t temp=-1000;
    modbus_set_slave(com,motor);
    int faild_num = 0;
    int flag = -1;
    do {
        usleep(20000);
        flag = modbus_read_registers(com, motorSpeedFeedbackAddr, 1, &temp);
        if (flag == -1) {
            cout << "error read motor" << motor << " speed." << endl;
            faild_num++;
        } else {
            cout << "succeed read motor" << motor << " speed." << endl;
        }
    } while (flag == -1 && faild_num<10);
    if(temp > 5000)
        temp = 0;
    return temp;
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "hello") ;
    ROS_INFO_STREAM("Hello, ROS!") ;
    ros::NodeHandle n_;

    // 开一个计算时间的线程
    pthread_t time_sync;
    pthread_create(&time_sync, nullptr, getTime, nullptr);
    ROS_INFO_STREAM("time sync spread make.");

    ofstream ofs;
    string filename = "/home/sjtu_wanghaili/yammar_ws/speed_result/";
    filename = filename + current_time + "_speed.txt";
    ofs.open(filename, ios::out);
    if(!ofs)
    {
        cerr<<"Open File Fail."<<endl;
        exit(1);
    }
    open_file = &ofs;

    ros::Subscriber sub2_;
    ros::Subscriber sub3_;
    ros::Subscriber sub_;

    ros::Publisher pub_;
    ros::Publisher pub1_;
    ros::Publisher pub2_;
    ros::Publisher pub3_;
    ros::Publisher pub4_;

    pub_ = n_.advertise<std_msgs::Float32>("modified_car_speed", 1);
    pub_modified_car_speed = &pub_;

    pub1_ = n_.advertise<std_msgs::Float32>("REEL_speed", 1);
    pub_reel_speed = &pub1_;

    pub2_ = n_.advertise<std_msgs::Float32>("CB_speed", 1);
    pub_cb_speed = &pub2_;

    pub3_ = n_.advertise<std_msgs::Float32>("PF_speed", 1);
    pub_pf_speed = &pub3_;

    pub4_ = n_.advertise<std_msgs::Float32>("FH_speed", 1);
    pub_pf_speed = &pub4_;

    //Topic you want to subscribe
    sub2_ = n_.subscribe("is_obstacle", 1, &obstacle_callback);
    sub3_ = n_.subscribe("is_stop", 1, &manual_stop_callback);
    sub_ = n_.subscribe("car_speed", 1, &carspeed_callback);

    cout<<"usage sudo ./motor"<<endl;
    //modbus_set_debug(com,true);//调试模式 可以显示串口总线的调试信息
    openSerial(port.c_str());
    motorInit();

//    // 开一个综合障碍物与手动停止的修改速度的线程
//    pthread_t motorControlThread;
//    pthread_create(&motorControlThread, nullptr, carSpeedFollowMode, nullptr);
//    ROS_INFO_STREAM("motor control spread make.");

    // 开一个背景中记录速度的线程
    pthread_t speed_read_background;
    pthread_create(&speed_read_background, nullptr, read_motor_speed_background, nullptr);
    ROS_INFO_STREAM("speed read in background spread make.");

    // 定义一个服务器，control485就是topic
    Server server(n_, "control485", boost::bind(&execute, _1, &server), false);
    // 服务器开始运行
    server.start();
    ros::spin();

    ROS_INFO_STREAM("wait spread close.");
    endFlag = true;
//    pthread_kill(motorControlThread, 0);
    pthread_kill(time_sync, 0);
    pthread_kill(speed_read_background, 0);
    ros::Duration(10);

    // todo 停下所有电机
    motorSetSpeed(1, 0);
    usleep(100000);
    motorSetSpeed(2, 0);
    usleep(100000);
    motorSetSpeed(3, 0);
    usleep(100000);
    ofs.close();
    return 0;
}

// 消息订阅回调函数
void obstacle_callback(const std_msgs::BoolConstPtr &msg) {
//    ROS_INFO_STREAM("callback! is_obstacle: "<<msg->data);
    is_obstacle = msg->data;
}
void manual_stop_callback(const std_msgs::BoolConstPtr &msg) {
    ROS_INFO_STREAM("callback! is_stop: "<<msg->data);
    is_stop = msg->data;
}
void carspeed_callback(const std_msgs::Float32ConstPtr &msg) {
    ROS_INFO_STREAM("callback! carspeed: "<<msg->data);
    carSpeed.linear = msg->data;
    carSpeed.rotate = 0;

    if(is_stop || is_obstacle){
        modified_car_speed.data = -1;
    } else{
        modified_car_speed.data = carSpeed.linear;
    }

    if(modified_car_speed.data != last_modified_car_speed){
        ROS_WARN_STREAM("modified speed will be change.");
    }
    pub_modified_car_speed->publish(modified_car_speed);
    last_modified_car_speed = modified_car_speed.data;
}
