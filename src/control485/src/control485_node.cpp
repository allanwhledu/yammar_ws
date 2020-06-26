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
uint16_t motorCurrentFeedbackAddr=0xC6; //说明书中找到而补充的，但是应该暂时不用（因为不精确吧）

uint16_t motorMODBUSAddr=0x43; //这是在网上找到的，设置从站地址
// 以上，就是现在用到的寄存器地址


double cbCof=1.2,reelCof=1.6,pfCof=4.44,fhCof=3.94;//同调率
int cbRatio=5,reelRatio=64,pfRatio=15,fhRatio=10;//减速比
const int reelMotor=1,cbMotor=2,pfMotor=3,fhMotor=4;
string port="/dev/ttyUSB0";
harvesterSpeed carSpeed;
bool is_obstacle = false;
bool is_stop = false;
ros::Publisher* pub_reel_speed;

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
volatile sig_atomic_t endFlag = 0;

//在程序退出前 调用电机停止指令
static void my_handler(int sig){ // can be called asynchronously
    endFlag = 1; // set flag
}

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
string getTime(void)
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
    string res="";
    res+='[';
    res+=to_string(timeNow->tm_year);
    res+='-';
    res+=to_string(timeNow->tm_mon);
    res+='-';
    res+=to_string(timeNow->tm_mday);
    res+=' ';
    res+=to_string(timeNow->tm_hour);
    res+=':';
    res+=to_string(timeNow->tm_min);
    res+=':';
    res+=to_string(timeNow->tm_sec);
    res+=':';
    res+=to_string(int(ms));
    res+=']';
    return res;

    // todo 获取时间的函数好像执行很慢，感觉可以之后开一个线程让它在后台慢慢执行
}
void motorInit(void)
{
    // 只有这里才打开了电机，这里首先仅仅开启了reel电机
    motorSetModbus(reelMotor,1);
    motorSetDirection(reelMotor,2);//正转
    motorSetSpeed(reelMotor,0);
}

// 使能某电机的rs485通讯
void motorSetModbus(int motor,int enable)
{
    modbus_set_slave(com,motor); //这句话的意思是不是将某从机设置为当前要访问的对象？
    modbus_write_register(com,motorModbusAddr,enable);
//    usleep(3000);
}
void motorSetDirection(int motor,int dir)
{
    modbus_set_slave(com,motor);
    modbus_write_register(com,motorDirectionAddr,dir);
//    usleep(3000);
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
    modbus_write_register(com,motorSpeedAddr,speed);
//    usleep(3000);
}
int motorReadSpeed(int motor)
{
    uint16_t temp=-1000;
    modbus_set_slave(com,motor);
    int flag = -1;
    do {
        flag = modbus_read_registers(com, motorSpeedFeedbackAddr, 1, &temp);

        // todo 这里为什么ankang写作等待？事实上不是可以写成一直循环查看吗？
        //    usleep(2000);
        if (flag == -1) {
            cout << "error read motor" << motor << " speed." << endl;
        } else {
            cout << "succeed read motor" << motor << " speed." << endl;
        }
    } while (flag == -1);
    return temp;
}

// —————— 该函数已废弃 ————————  //
pair<double,double> carReadSpeed(void)
{
    uint16_t temp[14];
    //modbus_set_debug(com,true);
    modbus_set_slave(com,5);//车速信号 can-modbus 从站序号为5
    if(modbus_read_registers(com,0x00,0X0E,temp)==-1)
    {
        cout<<"error reading carspeed,slave=5"<<endl;
    }
//    usleep(3000);
    double lSpeed=0.001,rSpeed=0.001;
    if(temp[7]==0&&temp[7]==0&&temp[8]==0&&temp[9]==0)//车未启动
    {
        return {0.0,0.0};
    }
    temp[7]=(temp[7])<<8;//直线车速高8位 左移8位
    temp[7]=(temp[7])|(temp[6]);//直线车速高8位|直线车速低8位
    int l=temp[7];
    l-=32768;
    temp[9]=(temp[9])<<8;//旋转车速高8位 左移8位
    temp[9]=(temp[9])|(temp[8]);//旋转车速高8位|旋转车速低8位
    int w=temp[9];
    w-=32768;
    lSpeed*=l;
    rSpeed*=w;
    if(lSpeed>5)
    {
        lSpeed=0;
    }
    else if(lSpeed<-5)
    {
        lSpeed=0;
    }
    if(rSpeed>5)
    {
        rSpeed=0;
    }
    else if(rSpeed<-5)
    {
        rSpeed=0;
    }
    return {lSpeed,rSpeed};
}
// —————— 该函数已废弃 ———————— //

int getReelSpeed(double carSpeed)
{
    if(carSpeed<0)
    {
        carSpeed=0;
    }
    int res= reelRatio*min(50.0,min(21.23*reelCof*carSpeed+12.3,21.23*1.0*carSpeed+21.23));
    if(is_obstacle == true || is_stop == true)
        res = 0;
    return min(res,3000);
}
int getCBSpeed(double carSpeed)
{
    if(carSpeed<0)
    {
        carSpeed=0;
    }
    int res= cbRatio*min(467.0,min(398.09*cbCof*carSpeed+131.37,398.09*1.0*carSpeed+238.85));
    if(is_obstacle == true || is_stop == true)
        res = 0;
    return min(res,3000);
}
int getPFSpeed(double carSpeed)
{
    if(carSpeed<0)
    {
        carSpeed=0;
    }
    int res= pfRatio*min(187.0,min(39.16*pfCof*carSpeed+52.47,39.16*3.0*carSpeed+90.07));
    if(is_obstacle == true || is_stop == true)
        res = 0;
    return min(res,3000);
}
int getFHSpeed(double carSpeed)
{
    if(carSpeed<0)
    {
        carSpeed=0;
    }
    int res= fhRatio*min(324.0,min(76.43*fhCof*carSpeed+90.95,76.43*3.0*carSpeed+152.86));
    if(is_obstacle == true || is_stop == true)
        res = 0;
    return min(res,3000);
}

void* carSpeedFollowMode(void*)
{
    fstream outFile;
    string file=getTime();
    string filename=file+".txt";
    outFile.open(filename,ios_base::app);
    outFile<<"startTime="<<file<<" cbRatio="<<cbCof<<" reelRatio="<<reelCof<<" pfRatio="<<pfCof<<" fhRatio="<<fhCof<<endl;
    outFile<<"format:time carvl carvw cbv cbvNew cbI reelv reelvNew reelI pfv pfvNew pfI fhv fhvNew fhI"<<endl;
    outFile.close();
    signal(SIGINT, my_handler);
    int cbSpeed=0,reelSpeed=0,fhSpeed=0,pfSpeed=0;
    int cbRealSpeed=0,reelRealSpeed=0,pfRealSpeed=0,fhRealSpeed=0;
    vector<double> current;
    cout.setf(ios_base::fixed);
    cout.precision(3);
    outFile.setf(ios_base::fixed);
    outFile.precision(3);
    while(1)
    {
        //calculate new motor speed
        reelSpeed=getReelSpeed(carSpeed.linear);//reel
//        cbSpeed=getCBSpeed(carSpeed.linear);//cb
//        fhSpeed=getFHSpeed(carSpeed.linear);//feedHouse
//        pfSpeed=getPFSpeed(carSpeed.linear);//PlatformAuger

        //get motor real speed
        reelRealSpeed=motorReadSpeed(reelMotor);
        if (reelRealSpeed != -1000) {
            std_msgs::Float32 reel_speed_msg;
            reel_speed_msg.data = reelRealSpeed;
            pub_reel_speed->publish(reel_speed_msg);
        }
//        cbRealSpeed=motorReadSpeed(cbMotor);
//        pfRealSpeed=motorReadSpeed(pfMotor);
//        fhRealSpeed=motorReadSpeed(fhMotor);
//        current=motorReadCurrent();

        //check if motor speed differnece is very small
        if(abs(reelSpeed-reelRealSpeed)>10)
        {
            motorSetSpeed(reelMotor,reelSpeed);
            ROS_INFO_STREAM("reel speed change");
        }
//        if(abs(cbSpeed-cbRealSpeed)>10)
//        {
//            ROS_INFO_STREAM("cb change");
//            motorSetSpeed(cbMotor,cbSpeed);
//        }
//        if(abs(fhSpeed-fhRealSpeed)>10)
//        {
//            ROS_INFO_STREAM("fh change");
//            motorSetSpeed(fhMotor,fhSpeed);
//        }
//        if(abs(pfSpeed-pfRealSpeed)>10)
//        {
//            ROS_INFO_STREAM("pf change");
//            motorSetSpeed(pfMotor,pfSpeed);
//        }
        string time=getTime();
        cout<<time<<" carVl="<<carSpeed.linear<<" carVw="<<carSpeed.rotate<<
            " reelv="<<reelRealSpeed<<" reelvNew="<<reelSpeed<<endl;

//        string time=getTime();
//        cout<<time<<" carVl="<<carSpeed.linear<<" carVw="<<carSpeed.rotate<<
//            " reelv="<<reelRealSpeed<<" reelvNew="<<reelSpeed<<" reelI="<<current[0]<<
//            " cbv="<<cbRealSpeed<<" cbvNew="<<cbSpeed<<" cbI="<<current[1]<<
//            " pfv="<<pfRealSpeed<<" pfvNew="<<pfSpeed<<" pfI="<<current[2]<<
//            " fhv="<<fhRealSpeed<<" fhvNew="<<fhSpeed<<" fhI="<<current[3]<<endl;
//        outFile.open(filename,ios_base::app);
//        outFile<<time<<" "<<carSpeed.linear<<" "<<carSpeed.rotate<<
//               " "<<reelRealSpeed<<" "<<reelSpeed<<" "<<current[0]<<
//               " "<<cbRealSpeed<<" "<<cbSpeed<<" "<<current[1]<<
//               " "<<pfRealSpeed<<" "<<pfSpeed<<" "<<current[2]<<
//               " "<<fhRealSpeed<<" "<<fhSpeed<<" "<<current[3]<<endl;
//        outFile.close();
        if(endFlag)
        {
//            usleep(2000);
            motorSetSpeed(0,0);
            modbus_close(com);
            modbus_free(com);
            cout<<"program terminated."<<endl;
            break;
        }
    }
}
vector<double> motorReadCurrent(void)
{
    vector<double> current(4);
    uint16_t analog[4]={0,0,0,0};
    modbus_set_slave(com,6);
    //slave=6 address reel=0x02 cb=0x03 pf=0x04 fh=0x05
    int flag=modbus_read_registers(com,2,4,analog);
    if(flag==-1)
    {
        cout<<"error read current."<<endl;
    }
//    usleep(3000);
    for(int i=0;i<4;i++)
    {
        //voltage=analog/32767*1.2
        //current=vlotage/rangeV*rangeI (0-70A: 0-5V)
        current[i]=analog[i]/32767.0*1.2*5*14;
    }
    //reeli=current0 cbi=current[1] pfi=current[2] fhi=current[3]
    return current;
}
double readHeight(void)
{
    double height;
    uint16_t analog;
    modbus_set_slave(com,6);
    //slave=6 height=0x08
    int flag=modbus_read_registers(com,8,1,&analog);
    if(flag==-1)
    {
        cout<<"error read grain heap height."<<endl;
    }
//    usleep(2000);
    //voltage=analog/32767.0*1.2*5
    //height=0.2+voltage/5*(3-0.2)
    height=0.2+analog/32767.0*1.2*5.0/5.0*(3-0.2);
    return height;
}

void car_speed_callback(const std_msgs::Float32ConstPtr &msg);
void is_obstacle_callback(const std_msgs::BoolConstPtr &msg);
void is_stop_callback(const std_msgs::BoolConstPtr &msg);

int main (int argc, char **argv)
{
    ros::init(argc, argv, "hello") ;
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Hello, ROS!") ;
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    ros::Subscriber sub2_;
    ros::Subscriber sub3_;
    ros::Publisher pub_;

    pub_ = n_.advertise<std_msgs::Float32>("REEL_speed", 10);
    pub_reel_speed = &pub_;

    //Topic you want to subscribe
    sub_ = n_.subscribe("car_speed", 1, &car_speed_callback);
    sub2_ = n_.subscribe("is_obstacle", 1, &is_obstacle_callback);
    sub3_ = n_.subscribe("is_stop", 1, &is_stop_callback);

    cout<<"usage sudo ./motor"<<endl;
    //modbus_set_debug(com,true);//调试模式 可以显示串口总线的调试信息
    openSerial(port.c_str());
    motorInit();
    pthread_t motorControlThread;
    pthread_create(&motorControlThread, nullptr, carSpeedFollowMode, &pub_);
    ROS_INFO_STREAM("spread make.");

    int count = 0;
    while (ros::ok()){
//        ROS_INFO_STREAM("spinonce");
        ros::spinOnce();
        count++;
        if(count == 100000){
            break;
        }
        ros::Rate(100).sleep();
    }
    ROS_INFO_STREAM("wait spread close.");
    pthread_kill(motorControlThread, 0);
    ros::Duration(10);
}

void car_speed_callback(const std_msgs::Float32ConstPtr &msg) {
//    ROS_INFO_STREAM("callback! carspeed: "<<msg->data);
    carSpeed.linear = msg->data;
    carSpeed.rotate = 0;
}
void is_obstacle_callback(const std_msgs::BoolConstPtr &msg) {
//    ROS_INFO_STREAM("callback! is_obstacle: "<<msg->data);
    is_obstacle = msg->data;
}

void is_stop_callback(const std_msgs::BoolConstPtr &msg) {
//    ROS_INFO_STREAM("callback! is_stop: "<<msg->data);
    is_stop = msg->data;
}
