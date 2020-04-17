// for action
#include <ios>
#include <locale>
#include <locale.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "reap_unit_action/ControlReapAction.h"

// for CAN
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "controlcan.h"
#include <ctime>
#include <cstdlib>
#include <cstdio>
#include "unistd.h"
#include <bitset>

// CAN相关函数
VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count=0;//数据列表中，用来存储列表序号。
//reap_unit_action::ControlReapFeedback feedback;


std::string getTime(void)
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
    std::string res="";
    res+='[';
    res+=std::to_string(timeNow->tm_year);
    res+='-';
    res+=std::to_string(timeNow->tm_mon);
    res+='-';
    res+=std::to_string(timeNow->tm_mday);
    res+=' ';
    res+=std::to_string(timeNow->tm_hour);
    res+=':';
    res+=std::to_string(timeNow->tm_min);
    res+=':';
    res+=std::to_string(timeNow->tm_sec);
    res+=':';
    res+=std::to_string(int(ms));
    res+=']';
    return res;
}

typedef actionlib::SimpleActionServer<reap_unit_action::ControlReapAction> Server;
Server* pas;

reap_unit_action::ControlReapFeedback* pfd;

void init_CAN() {// 进行CAN信号发送
    printf(">>start CAN device !\r\n");//指示程序已运行
    if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
    {
        printf(">>open deivce success!\n");//打开设备成功
    }else
    {
        printf(">>open deivce error!\n");
        // exit(1);
    }

    //初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config;
    config.AccCode=0;
    config.AccMask=0xFFFFFFFF;
    config.Filter=1;//接收所有帧
    config.Timing0=0x00;/*波特率1000 Kbps  Timing0=0x00 Timing1= 0x14*/
    config.Timing1=0x14;
    config.Mode=0;//正常模式

    if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)//CAN1
    {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
    }

    if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
    {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2,0);
    }
}

void *receive_func(void* param)  //接收线程,若接受到的信号为速度反馈，则将之反馈会client。
{
	// rec是指一个数据帧，一个rec的类型是VCI_CAN_OBJ。rec.Datelen才是一个数据帧的长度。
	int i,j;
	int ind=0;
	int reclen=0; //接受一次数据之后的实际数据帧数量
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。但是一次接受不一定能够接到3000个数据帧。

	int *run=(int*)param;//线程启动，退出控制。

	while((*run)&0x0f)
	{
		// VCI_Receive(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pReceive,UINT Len,INT WaitTime);
		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,0))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
		    // 上面有一个WaitTime我们可以知道，其实can卡硬件接受的信号频率非常高，只是我们这里过10毫秒来看一次处理一次而已。
		    // 但是为什么没有缓存下来呢？
			for(j=0;j<reclen;j++)
			{
//				printf("RX:%04d  ",count);
//				printf(" ID:%02X ", rec[j].ID);
//				if(rec[j].ExternFlag==0) printf(" Standard ");//帧格式：标准帧
//				if(rec[j].ExternFlag==1) printf(" Extend   ");//帧格式：扩展帧
//				if(rec[j].RemoteFlag==0) printf(" Data   ");//帧类型：数据帧
//				if(rec[j].RemoteFlag==1) printf(" Remote ");//帧类型：远程帧
//				printf("DLC:0x%02X",rec[j].DataLen);//帧长度
//				printf(" data:0x");	//数据
//				for(i = 0; i < rec[j].DataLen; i++)
//				{
//					printf(" %02X", rec[j].Data[i]);
//				}
////				printf(" TimeStamp:0x%08X",rec[j].TimeStamp);//时间标识。
//                printf(" time: ");
//                std::cout<<getTime()<<std::endl;

				if(rec[j].Data[2] == 0x92) //判断是否为速度误差反馈
                {
//				    printf("This is a speed callback msg.\n");

                    unsigned char heigh,low;
                    heigh=rec[j].Data[5];
                    low=rec[j].Data[4];
                    pfd->percent_complete = heigh << 8 | low;

                    pas->publishFeedback(*pfd);
//                    ROS_INFO_STREAM(feedback.percent_complete);
                    ROS_INFO("Receive msg:%04d ID:%02X Data:0x %02X %02X %02X %02X %02X %02X %02X %02X speed_error:%04d",count,rec[j].ID,
                             rec[j].Data[0],rec[j].Data[1],rec[j].Data[2],rec[j].Data[3],
                             rec[j].Data[4],rec[j].Data[5],rec[j].Data[6],rec[j].Data[7],heigh << 8 | low);
                } else
                {
                    ROS_INFO("Receive msg:%04d ID:%02X Data:0x %02X %02X %02X %02X %02X %02X %02X %02X",count,rec[j].ID,
                             rec[j].Data[0],rec[j].Data[1],rec[j].Data[2],rec[j].Data[3],
                             rec[j].Data[4],rec[j].Data[5],rec[j].Data[6],rec[j].Data[7]);
                }

                count++;//序号递增
			}
		}
		//ind=!ind;//变换通道号，以便下次读取另一通道，交替读取。		
	}
	ROS_INFO_STREAM("Exit receive pthread.");//退出接收线程
	pthread_exit(0);
}

void transmit_msg(VCI_CAN_OBJ send[1], char com[10]) //发送函数
{
	if(VCI_Transmit(VCI_USBCAN2,0,0,send,1) == 1)
	{

        ROS_INFO("Send    msg:%04d ID:%02X Data:0x %02X %02X %02X %02X %02X %02X %02X %02X COMMAND:%s",count,send[0].ID,
        send[0].Data[0],send[0].Data[1],send[0].Data[2],send[0].Data[3],
        send[0].Data[4],send[0].Data[5],send[0].Data[6],send[0].Data[7],com);

        count++;
        
//		printf("TX:%04d  ",count);count++;
//        printf(" ID:%02X ",send[0].ID);
//		if(send[0].ExternFlag==0) printf(" Standard ");
//		if(send[0].ExternFlag==1) printf(" Extend   ");
//		if(send[0].RemoteFlag==0) printf(" Data   ");
//		if(send[0].RemoteFlag==1) printf(" Remote ");
//		printf("DLC:0x%02X",send[0].DataLen);
//		printf(" data:0x");
//		for(int i=0;i<send[0].DataLen;i++)
//		{
//			printf(" %02X",send[0].Data[i]);
//		}
//
//		printf(" time: ");
//		std::cout<<getTime()<<std::endl;
//		printf("\n");
	}
}

void set_speed_mode(int num_motor) //驱动第num_motor号电机，速度为speed.
{
    // 设置电机为CAN控制，速度模式
    VCI_CAN_OBJ msg[1];

    msg[0].ID=num_motor;
    msg[0].SendType=0;
    msg[0].RemoteFlag=0;
    msg[0].ExternFlag=0;
    msg[0].DataLen=8;

    msg[0].Data[0] = 0x04;
    msg[0].Data[1] = num_motor;
    msg[0].Data[2] = 0x2A;
    msg[0].Data[3] = 0x00;
    msg[0].Data[4] = 0x00;
    msg[0].Data[5] = 0x00;
    msg[0].Data[6] = 0x00;
    msg[0].Data[7] = 0x00;

    transmit_msg(msg, "set  MODE");
}

void set_motor_speed(int num_motor, int speed) //驱动第num_motor号电机，速度为speed.
{
	// 将速度指令拆分为高低位
	unsigned char heigh,low;
	heigh=(speed>>8)&0xff;
	low=speed&0xff;

	VCI_CAN_OBJ msg[1];

	msg[0].ID = num_motor;
	msg[0].SendType=0;
	msg[0].RemoteFlag=0;
	msg[0].ExternFlag=0;
	msg[0].DataLen=8;

	msg[0].Data[0] = 0x08;
	msg[0].Data[1] = num_motor;
	msg[0].Data[2] = 0x90;
	msg[0].Data[3] = 0x00;
	msg[0].Data[4] = low;
	msg[0].Data[5] = heigh;
	msg[0].Data[6] = 0x00;
	msg[0].Data[7] = 0x00;

	transmit_msg(msg, "set SPEED");
}

void callFeedback(int num_motor) //读取第num_motor号电机的速度误差
{
    VCI_CAN_OBJ msg[1];

    msg[0].ID = num_motor;
    msg[0].SendType=0;
    msg[0].RemoteFlag=0;
    msg[0].ExternFlag=0;
    msg[0].DataLen=8;

    msg[0].Data[0] = 0x04;
    msg[0].Data[1] = num_motor;
    msg[0].Data[2] = 0x92;
    msg[0].Data[3] = 0x00;
    msg[0].Data[4] = 0x00;
    msg[0].Data[5] = 0x00;
    msg[0].Data[6] = 0x00;
    msg[0].Data[7] = 0x00;

    transmit_msg(msg, "call ERRO");
}

// 收到action的goal后调用的回调函数
void execute(const reap_unit_action::ControlReapGoalConstPtr& goal, Server* as)
{
    // 保存server的指针到全局变量
    pas = as;

    // 清理can卡缓冲区,但是感觉没有太大的用处
    VCI_ClearBuffer(VCI_USBCAN2,0,0);

//	ros::Rate r(1);
	reap_unit_action::ControlReapFeedback feedback;
	pfd = &feedback;

	ROS_INFO("Motor %d is working.", goal->dishwasher_id);

//	// 假设洗盘子的进度，并且按照1hz的频率发布进度feedback
//	for(int i=1; i<=3; i++)
//	{
//		feedback.percent_complete = i * 10;
//		as->publishFeedback(feedback);
//		r.sleep();
//	}
//
//	// 当action完成后，向客户端返回结果
//	ROS_INFO("Dishwasher %d finish working.", goal->dishwasher_id);

    // 开启CAN信号接受线程
	int m_run0=1; // 在之后通过m_run0置零可从外部关闭线程
	pthread_t threadid;
	int ret;
	ret=pthread_create(&threadid,NULL,receive_func,&m_run0);

	// 设置电机为速度模式
	set_speed_mode(goal->dishwasher_id);
	// 发送电机驱动指令
    set_motor_speed(goal->dishwasher_id, goal->target_speed);
    usleep(10000);

    // 读取电机速度误差
    callFeedback(goal->dishwasher_id);
    usleep(10000);
    while(feedback.percent_complete != 0)
    {
        callFeedback(goal->dishwasher_id);
        usleep(1000);
    }

    m_run0=0;//线程关闭指令,这里是关闭接受线程
    pthread_join(threadid,NULL);//等待线程关闭。

    as->setSucceeded();


//	usleep(8000000);
//
//	// 停下电机
//    set_motor_speed(goal->dishwasher_id, 0);
//
//	usleep(100000000);//延时单位us，这里设置 100 000 000=10s    100s后关闭接收线程，并退出主程序。
//	m_run0=0;//线程关闭指令,这里是关闭接受线程
//	pthread_join(threadid,NULL);//等待线程关闭。
//	usleep(100000);//延时100ms。
//	VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
//	usleep(100000);//延时100ms。
//	VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
//	usleep(100000);//延时100ms。
//	VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
	//除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
	//goto ext;
	ROS_INFO_STREAM("Once action complete. Wait for next invoke.\n");
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "control_reap_server");
    setlocale( LC_ALL, "" );

    ros::NodeHandle n;

    // 初始化CAN卡
    init_CAN();

	// 定义一个服务器，do_dishes就是topic
	Server server(n, "control_reap", boost::bind(&execute, _1, &server), false);

	// 服务器开始运行
	server.start();

	ros::spin();

	// 停下电机
    set_motor_speed(40, 0);
    set_motor_speed(39, 0);
    set_motor_speed(38, 0);
    set_motor_speed(37, 0);

	usleep(100000);//延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
	usleep(100000);//延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
	usleep(100000);//延时100ms。
	VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
    printf(">>close deivce success!\n");//打开设备成功
    // 除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
    // goto ext;

	return 0;
}
