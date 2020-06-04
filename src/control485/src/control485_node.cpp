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
#include"controlcan.h"
using namespace std;
// class Motor
// {
// public:
//     static const uint16_t motorModbusAddr=0xB6;
//     static const uint16_t motorDirectionAddr=0x66;
//     static const uint16_t motorSpeedAddr=0x56;
//     static const uint16_t motorSpeedFeedbackAddr=0x5F;
//     int id;
//     string name;
//     int speedReal;
//     int speedOrder;
//     int reducerRatio;
//     double cof;
//     static modbus_t* com;//所有电机共用一个串口通信 设为静态变量
// };

// 初始化变量
struct harvesterSpeed
{
    double linear=0.0;
    double rotate=0.0;
};
modbus_t* com;//com用于电机速度控制反馈 
uint16_t motorModbusAddr=0xB6;
uint16_t motorDirectionAddr=0x66;
uint16_t motorSpeedAddr=0x56;
uint16_t motorSpeedFeedbackAddr=0x5F;
double cbCof=1.2,reelCof=1.6,pfCof=4.44,fhCof=3.94;//同调率
int cbRatio=5,reelRatio=64,pfRatio=15,fhRatio=10;//减速比
const int cbMotor=2,reelMotor=1,pfMotor=3,fhMotor=4;
string port="/dev/ttyUSB0";
harvesterSpeed carSpeed;

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
void* getCarSpeedCAN(void* speed);//CAN通信读取车速的线程函数
void* cbControl(void* input);//
bool canInit(void);
//在程序退出前 调用电机停止指令
static void my_handler(int sig){ // can be called asynchronously
  endFlag = 1; // set flag
}

// 主函数
int main(int argc,char** argv)
{
    cout<<"usage sudo ./motor"<<endl;
    //modbus_set_debug(com,true);//调试模式 可以显示串口总线的调试信息
    openSerial(port.c_str());
    motorInit();
    pthread_t carSpeedThread,motorControlThread;
    pthread_create(&carSpeedThread,nullptr,getCarSpeedCAN,nullptr);
    pthread_create(&motorControlThread,nullptr,carSpeedFollowMode,nullptr);
    pthread_join(motorControlThread,nullptr);
    pthread_kill(carSpeedThread,0);
    return 0;
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
    modbus_set_response_timeout(com,&time_out);
    modbus_rtu_set_serial_mode(com,MODBUS_RTU_RS485);
    if(modbus_connect(com)==-1)
    {
        cout<<"Cannot connect modbus at port:"<<port<<endl;
        return false;
    }
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
}
void motorInit(void)
{
   motorSetModbus(0,1);
   motorSetDirection(0,2);//正转
   motorSetDirection(fhMotor,1);
   motorSetSpeed(0,0);
}
void motorSetModbus(int motor,int enable)
{
    modbus_set_slave(com,motor);
    modbus_write_register(com,motorModbusAddr,enable);
    usleep(3000);
}
void motorSetDirection(int motor,int dir)
{
    modbus_set_slave(com,motor);
    modbus_write_register(com,motorDirectionAddr,dir);
    usleep(3000);
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
    usleep(3000);
}
int motorReadSpeed(int motor)
{
    uint16_t temp=0;
    modbus_set_slave(com,motor);
    int flag=modbus_read_registers(com,motorSpeedFeedbackAddr,1,&temp);
    usleep(2000);
    if(flag==-1)
    {
        cout<<"error read motor"<<motor<<" speed."<<endl;
    }
    return temp;
}
pair<double,double> carReadSpeed(void)
{
    uint16_t temp[14];
    //modbus_set_debug(com,true);
    modbus_set_slave(com,5);//车速信号 can-modbus 从站序号为5
    if(modbus_read_registers(com,0x00,0X0E,temp)==-1)
    {
        cout<<"error reading carspeed,slave=5"<<endl;
    }
    usleep(3000);
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
int getCBSpeed(double carSpeed)
{
    if(carSpeed<0)
    {
        carSpeed=0;
    }
    int res= cbRatio*min(467.0,min(398.09*cbCof*carSpeed+131.37,398.09*1.0*carSpeed+238.85));
    return min(res,3000);
}
int getReelSpeed(double carSpeed)
{
    if(carSpeed<0)
    {
        carSpeed=0;
    }
    int res= reelRatio*min(50.0,min(21.23*reelCof*carSpeed+12.3,21.23*1.0*carSpeed+21.23));
    return min(res,3000);
}
int getPFSpeed(double carSpeed)
{
    if(carSpeed<0)
    {
        carSpeed=0;
    }
    int res= pfRatio*min(187.0,min(39.16*pfCof*carSpeed+52.47,39.16*3.0*carSpeed+90.07));
    return min(res,3000);
}
int getFHSpeed(double carSpeed)
{
    if(carSpeed<0)
    {
        carSpeed=0;
    }
    int res= fhRatio*min(324.0,min(76.43*fhCof*carSpeed+90.95,76.43*3.0*carSpeed+152.86));
    return min(res,3000);
}
void carSpeedFollowMode(void)
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
        string time=getTime();
        //calculate new motor speed
        cbSpeed=getCBSpeed(carSpeed.linear);//cb
        reelSpeed=getReelSpeed(carSpeed.linear);//reel
        fhSpeed=getFHSpeed(carSpeed.linear);//feedHouse
        pfSpeed=getPFSpeed(carSpeed.linear);//PlatformAuger
        //get motor real speed
        cbRealSpeed=motorReadSpeed(cbMotor);
        reelRealSpeed=motorReadSpeed(reelMotor);
        pfRealSpeed=motorReadSpeed(pfMotor);
        fhRealSpeed=0;
        //fhRealSpeed=motorReadSpeed(fhMotor);
        current=motorReadCurrent();
        //check if motor speed differnece is very small
        if(abs(cbSpeed-cbRealSpeed)>10)
        {
            motorSetSpeed(cbMotor,cbSpeed);
        }
        if(abs(reelSpeed-reelRealSpeed)>10)
        {
            motorSetSpeed(reelMotor,reelSpeed);
        }
        if(abs(fhSpeed-fhRealSpeed)>10)
        {
            //motorSetSpeed(fhMotor,fhSpeed);
        }
        if(abs(pfSpeed-pfRealSpeed)>10)
        {
            motorSetSpeed(pfMotor,pfSpeed);
        }
        cout<<time<<" carVl="<<carSpeed.linear<<" carVw="<<carSpeed.rotate<<
        " cbv="<<cbRealSpeed<<" cbvNew="<<cbSpeed<<" cbI="<<current[1]<<
        " reelv="<<reelRealSpeed<<" reelvNew="<<reelSpeed<<" reelI="<<current[0]<<
        " pfv="<<pfRealSpeed<<" pfvNew="<<pfSpeed<<" pfI="<<current[2]<<
        " fhv="<<fhRealSpeed<<" fhvNew="<<fhSpeed<<" fhI="<<current[3]<<endl;
        outFile.open(filename,ios_base::app);
        outFile<<time<<" "<<carSpeed.linear<<" "<<carSpeed.rotate<<
        " "<<cbRealSpeed<<" "<<cbSpeed<<" "<<current[1]<<
        " "<<reelRealSpeed<<" "<<reelSpeed<<" "<<current[0]<<
        " "<<pfRealSpeed<<" "<<pfSpeed<<" "<<current[2]<<
        " "<<fhRealSpeed<<" "<<fhSpeed<<" "<<current[3]<<endl;
        outFile.close();
        if(endFlag)
        {
            usleep(2000);
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
    usleep(3000);
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
    //usleep(2000);
    //voltage=analog/32767.0*1.2*5
    //height=0.2+voltage/5*(3-0.2)
    height=0.2+analog/32767.0*1.2*5.0/5.0*(3-0.2);
    return height;
}
void* getCarSpeedCAN(void* speed)
{
   if(!canInit())
   {
       printf("CAN bus error,program terminated.\n");
       exit(EXIT_FAILURE);
   }
   const int buffSize=2500;
   VCI_CAN_OBJ buff[buffSize];
   while(1)
   {
       int len=VCI_Receive(VCI_USBCAN2,0,0,buff,buffSize,500);
       if(len==0)
       {
           //printf("No message receive.\n");
           usleep(1000*50);
           continue;
       }
       int idx=len-1;
       for(idx;idx>=0;idx--)
       {
           if(buff[idx].ID==0xCFF5188)
           {
               break;
           }
       }
       if(idx<0)
       {
           //printf("no speed message.\n");
           usleep(1000*50);
           continue;
       }
       auto p=buff[idx];
       //cout<<"canid="<<hex<<p.ID<<endl;
       //printf("canid=%d\n",p.ID);
       double v=0.0,w=0.0;
       uint16_t data[8];
       for(int i=0;i<8;i++)
       {
           data[i]=p.Data[i];
       }
       v=(data[1]<<8)|data[0];
       w=(data[3]<<8)|data[2];
       v-=32768;
       w-=32768;
       v/=1000;
       w/=1000;
       carSpeed.linear=v;
       carSpeed.rotate=w;
       usleep(1000*50);//每10ms读取一次车速信号并更新
   }
   pthread_exit(0);
}
bool canInit(void)
{
    int flag=VCI_OpenDevice(VCI_USBCAN2,0,0);
    if(flag==1)
    {
        printf("CAN device open success.\n");
    }
    else
    {
        printf("CAN device open failed.\n");
        return false;
    }
    VCI_INIT_CONFIG config;
    config.AccCode=0x0CFF5188;//接收码
	config.AccMask=0xFFFFFFFF;//屏蔽码
	config.Filter=0;//过滤方式
	config.Timing0=0x00;/*波特率500 Kbps  Timing0=0x0 Timing1= 0x1C*/
	config.Timing1=0x1C;
	config.Mode=0;//正常模式 接收和发送
    flag=VCI_InitCAN(VCI_USBCAN2,0,0,&config);
    if(flag!=1)
    {
        printf("CAN device init failed.\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return false;
    }
    flag=VCI_StartCAN(VCI_USBCAN2,0,0);
    if(flag!=1)
    {
        printf("CAN device start failed.\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
        return false;
    }
    return true;
}