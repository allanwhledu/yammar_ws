//
// Created by bit on 2020/5/8.
//

#include "CAN_DEVICE.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int64.h"


CAN_DEVICE::CAN_DEVICE(int channel_idx) {
    count = 0;
    m_run0 = 0;
    channel = channel_idx - 1;
}

void CAN_DEVICE::init_CAN() {// 进行CAN信号发送
    if (channel == 0)
    {
        printf(">>start CAN device !\r\n");//指示程序已运行
        if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1)//打开设备
        {
            printf(">>open device success!\n");//打开设备成功
        } else {
            printf(">>open device error!\n");
            exit(1);
        }
    }

    //初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1;//接收所有帧
    config.Timing0 = 0x00;/*波特率1000 Kbps  Timing0=0x00 Timing1= 0x14*/
    config.Timing1 = 0x14;
    config.Mode = 0;//正常模式

    if (VCI_InitCAN(VCI_USBCAN2, 0, channel, &config) != 1)//CAN1
    {
        printf(">>Init CAN error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
        exit(1);
    }

    if (VCI_StartCAN(VCI_USBCAN2, 0, channel) != 1) {
        printf(">>Start CAN error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
        exit(1);
    }
}

void *receive_func(void *param)  //接收线程,若接受到的信号为目标反馈，则将之反馈会client。
{
    CAN_DEVICE *pCAN_DEVICE = (CAN_DEVICE *) param;

    // rec是指一个数据帧，一个rec的类型是VCI_CAN_OBJ。rec.Datelen才是一个数据帧的长度。
    int i, j;
    int ind = 0;
    int reclen = 0; //接受一次数据之后的实际数据帧数量
    VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。但是一次接受不一定能够接到3000个数据帧。

    // 结合while循环，可以从外部修改m_run0来控制while循环
    while (pCAN_DEVICE->m_run0 & 0x0f) {
        // VCI_Receive(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pReceive,UINT Len,INT WaitTime);
        if ((reclen = VCI_Receive(VCI_USBCAN2, 0, pCAN_DEVICE->channel, rec, 3000, 0)) > 0)//调用接收函数，如果有数据，进行数据处理显示。
        {
            // 上面有一个WaitTime我们可以知道，其实can卡硬件接受的信号频率非常高，只是我们这里过10毫秒来看一次处理一次而已。
            for (j = 0; j < reclen; j++) {
                if (rec[j].ID == 0x0181) // 采集卡 channel1 1-5的数据
                {
                    unsigned char heigh1, low1;
                    heigh1 = rec[j].Data[1];
                    low1 = rec[j].Data[0];
                    unsigned char heigh2, low2;
                    heigh2 = rec[j].Data[3];
                    low2 = rec[j].Data[2];
                    if ((heigh1 << 8 | low1) > 60000 || (heigh2 << 8 | low2) > 60000)
                        continue;
                    // 如果为目标信号，则进行相关的操作
                    float vol1 = (heigh1 << 8 | low1);
                    pCAN_DEVICE->angle1 = vol1/2*105/4000+5; //因为输入电压是10v，所以除以2
                    std_msgs::Int64 data_receive1;
                    data_receive1.data = pCAN_DEVICE->angle1;
                    pCAN_DEVICE->pub_c1->publish(data_receive1);

                    float vol2 = (heigh2 << 8 | low2);
                    pCAN_DEVICE->angle2 = vol2/2*105/4000+5;
                    std_msgs::Int64 data_receive2;
                    data_receive2.data = pCAN_DEVICE->angle2;
                    pCAN_DEVICE->pub_c2->publish(data_receive2);

                    ROS_INFO(
                            "Channel %02d Receive msg:%04d ID:%02X Data:0x %02X %02X %02X %02X %02X %02X %02X %02X angle1:%05d angle2:%05d",
                            pCAN_DEVICE->channel+1, pCAN_DEVICE->count, rec[j].ID,
                            rec[j].Data[0], rec[j].Data[1], rec[j].Data[2], rec[j].Data[3],
                            rec[j].Data[4], rec[j].Data[5], rec[j].Data[6], rec[j].Data[7], pCAN_DEVICE->angle1, pCAN_DEVICE->angle2);
                }
                else if (rec[j].ID == 0x0281) { //采集卡 channel2 5-8的数据
                    unsigned char heigh, low;
                    heigh = rec[j].Data[1];
                    low = rec[j].Data[0];

                    if ((heigh << 8 | low) > 60000)
                        continue;
                    // 如果为目标信号，则进行相关的操作
//                    pCAN_DEVICE->angle1 = heigh << 8 | low;
//                    std_msgs::Int64 data_receive;
//                    data_receive.data = heigh << 8 | low;
//                    pCAN_DEVICE->pub_c1->publish(data_receive);

                    ROS_INFO(
                            "Channel %02d Receive msg:%04d ID:%02X Data:0x %02X %02X %02X %02X %02X %02X %02X %02X angle5:%04d",
                            pCAN_DEVICE->channel+1, pCAN_DEVICE->count, rec[j].ID,
                            rec[j].Data[0], rec[j].Data[1], rec[j].Data[2], rec[j].Data[3],
                            rec[j].Data[4], rec[j].Data[5], rec[j].Data[6], rec[j].Data[7], heigh << 8 | low);
                }
                else if (rec[j].ID == 0xCFF5188)
                {
                    double v=0.0,w=0.0;
                    uint16_t data[8];
                    for(int i=0;i<8;i++)
                    {
                        data[i]=rec[j].Data[i];
                    }
                    v=(data[1]<<8)|data[0];
                    w=(data[3]<<8)|data[2];
                    v-=32768;
                    w-=32768;
                    v/=1000;
                    w/=1000;
//                    carSpeed.linear=v;
//                    carSpeed.rotate=w;
                    pCAN_DEVICE->car_speed.data = v;
                    pCAN_DEVICE->pub_c3->publish(pCAN_DEVICE->car_speed);
                }
                else {
                    ROS_INFO("Channel %02d Receive msg:%04d ID:%02X Data:0x %02X %02X %02X %02X %02X %02X %02X %02X",
                            pCAN_DEVICE->channel+1,
                            pCAN_DEVICE->count,
                             rec[j].ID,
                             rec[j].Data[0], rec[j].Data[1], rec[j].Data[2], rec[j].Data[3],
                             rec[j].Data[4], rec[j].Data[5], rec[j].Data[6], rec[j].Data[7]);
                }
                pCAN_DEVICE->count++;//序号递增
            }
        }
    }
    ROS_INFO_STREAM("Exit receive pthread.");//退出接收线程
    pthread_exit(0);
}

void CAN_DEVICE::transmit_msg(VCI_CAN_OBJ send[1], char com[10]) //发送函数
{
    if (VCI_Transmit(VCI_USBCAN2, 0, channel, send, 1) == 1) {

        ROS_INFO("Send    msg:%04d ID:%02X Data:0x %02X %02X %02X %02X %02X %02X %02X %02X COMMAND:%s", count,
                 send[0].ID,
                 send[0].Data[0], send[0].Data[1], send[0].Data[2], send[0].Data[3],
                 send[0].Data[4], send[0].Data[5], send[0].Data[6], send[0].Data[7], com);

        count++;
    }
}

void CAN_DEVICE::set_speed_mode(int num_motor) //驱动第num_motor号电机，速度为speed.
{
    // 设置电机为CAN控制，速度模式
    VCI_CAN_OBJ msg[1];

    msg[0].ID = num_motor;
    msg[0].SendType = 0;
    msg[0].RemoteFlag = 0;
    msg[0].ExternFlag = 0;
    msg[0].DataLen = 8;

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

void CAN_DEVICE::set_motor_speed(int num_motor, int speed) //驱动第num_motor号电机，速度为speed.
{
    // 将速度指令拆分为高低位
    unsigned char heigh, low;
    heigh = (speed >> 8) & 0xff;
    low = speed & 0xff;

    VCI_CAN_OBJ msg[1];

    msg[0].ID = num_motor;
    msg[0].SendType = 0;
    msg[0].RemoteFlag = 0;
    msg[0].ExternFlag = 0;
    msg[0].DataLen = 8;

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

void CAN_DEVICE::callFeedback(int num_motor) //读取第num_motor号电机的速度误差
{
    VCI_CAN_OBJ msg[1];

    msg[0].ID = num_motor;
    msg[0].SendType = 0;
    msg[0].RemoteFlag = 0;
    msg[0].ExternFlag = 0;
    msg[0].DataLen = 8;

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

void CAN_DEVICE::callCurrent(int num_motor) //读取第num_motor号电机的速度误差
{
    VCI_CAN_OBJ msg[1];

    msg[0].ID = num_motor;
    msg[0].SendType = 0;
    msg[0].RemoteFlag = 0;
    msg[0].ExternFlag = 0;
    msg[0].DataLen = 8;

    msg[0].Data[0] = 0x04;
    msg[0].Data[1] = num_motor;
    msg[0].Data[2] = 0xD0;
    msg[0].Data[3] = 0x00;
    msg[0].Data[4] = 0x00;
    msg[0].Data[5] = 0x00;
    msg[0].Data[6] = 0x00;
    msg[0].Data[7] = 0x00;

    transmit_msg(msg, "call CURR");
}

void CAN_DEVICE::open_receive() {
    // 开启CAN信号接受线程
    int ret;
    m_run0 = 1;
    ret = pthread_create(&receive_thread, NULL, receive_func, this);
    ROS_INFO_STREAM("receive_thread_create.");
}

void CAN_DEVICE::check_speed() {
//    // 关闭CAN信号接受线程
//    this->callFeedback(motor);
////    ROS_INFO_STREAM("first call send");
//
//    while(pfd.percent_complete == 10000)
//    {
//        log_error.push_back(pfd.percent_complete);
////        ROS_INFO_STREAM("still wait");
//        this->callFeedback(motor);
//        usleep(2000);
//    }
//
//    while(pfd.percent_complete > 50)
//    {
//        log_error.push_back(pfd.percent_complete);
//        this->callFeedback(motor);
//        usleep(2000);
//    }
//    ROS_INFO_STREAM("ok error is already small");
//
//    this->m_run0 = 0;
////    ROS_INFO_STREAM("wait close");
//    pthread_join(receive_thread, NULL);//等待线程关闭
////    ROS_INFO_STREAM("receive_thread_close.");
}

void CAN_DEVICE::wait_current() {
//    // 关闭CAN信号接受线程
//    this->callCurrent(motor);
////    ROS_INFO_STREAM("call send");
//
//    while(pfd.percent_complete == 10000)
//    {
//        log_error.push_back(pfd.percent_complete);
////        ROS_INFO_STREAM("still wait current feedback");
//        this->callCurrent(motor);
//        usleep(2000);
//    }
//    ROS_INFO_STREAM("current got.");
//
////    while(pfd.percent_complete > 50)
////    {
////        log_error.push_back(pfd.percent_complete);
////        this->callFeedback(motor);
////        usleep(2000);
////    }
////    ROS_INFO_STREAM("ok error is already small");
//
//    this->m_run0 = 0;
////    ROS_INFO_STREAM("wait close");
//    pthread_join(receive_thread, NULL);//等待线程关闭
////    ROS_INFO_STREAM("receive_thread_close.");
}

void CAN_DEVICE::close_receive() {
    this->m_run0 = 0;
    pthread_join(receive_thread, NULL);//等待线程关闭
}

void CAN_DEVICE::closeCAN() {
    usleep(100000);//延时100ms。
    VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
    usleep(100000);//延时100ms。
    VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
    usleep(100000);//延时100ms。
    VCI_CloseDevice(VCI_USBCAN2, 0);//关闭设备。
    printf(">>close deivce success!\n");//打开设备成功
// 除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
// goto ext;
}

void CAN_DEVICE::setMotor(int motor) {
    CAN_DEVICE::motor = motor;
}

void CAN_DEVICE::init_ICAN(){
    VCI_CAN_OBJ msg[1];

    msg[0].ID = 0;
    msg[0].SendType = 0;
    msg[0].RemoteFlag = 0;
    msg[0].ExternFlag = 0;
    msg[0].DataLen = 2;

    msg[0].Data[0] = 0x01;
    msg[0].Data[1] = 0x01;
//    msg[0].Data[2] = 0xD0;
//    msg[0].Data[3] = 0x00;
//    msg[0].Data[4] = 0x00;
//    msg[0].Data[5] = 0x00;
//    msg[0].Data[6] = 0x00;
//    msg[0].Data[7] = 0x00;

    transmit_msg(msg, "init ICAN");
}
