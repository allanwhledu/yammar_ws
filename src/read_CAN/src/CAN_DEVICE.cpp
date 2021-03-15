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

    for (int i =0;i<buffer_length;i++)
    {
        current_buffer0.push_back(0);
        current_buffer1.push_back(0);
        current_buffer2.push_back(0);
        current_buffer3.push_back(0);
        current_buffer4.push_back(0);
    }  // 本来应该是初始化直接有定义的，但是没有成功，所以这样替代做
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
    // 这里，已经改成了500kbps，适应车辆
    config.Timing0 = 0x00;
    config.Timing1 = 0x1C;
    config.Mode = 0;//正常模式

    if (VCI_InitCAN(VCI_USBCAN2, 0, channel, &config) != 1)//CAN1
    {
        printf(">>Init CAN error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
//        exit(1);
    }

    if (VCI_StartCAN(VCI_USBCAN2, 0, channel) != 1) {
        printf(">>Start CAN error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
//        exit(1);
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

                //// 采集卡1：
                if (rec[j].ID == 0x0181) // 0-3接口的数据
                {
                    unsigned char high0, low0;
                    high0 = rec[j].Data[1];
                    low0 = rec[j].Data[0];
                    unsigned char high1, low1;
                    high1 = rec[j].Data[3];
                    low1 = rec[j].Data[2];
                    unsigned char high2, low2;
                    high2 = rec[j].Data[5];
                    low2 = rec[j].Data[4];
                    unsigned char high3, low3;
                    high3 = rec[j].Data[7];
                    low3 = rec[j].Data[6];

                    if ((high0 << 8 | low0) > 60000 || (high1 << 8 | low1) > 60000 || (high2 << 8 | low2) > 60000)
                        continue;
                    // 1号角度传感器-割台
                    int vol1 = (high0 << 8 | low0);
                    ROS_INFO_STREAM(vol1);
                    float vol1_norm = float(vol1)/1000;
                    ROS_INFO_STREAM(vol1_norm);
                    float angle1 = 31.56 - vol1_norm * 31.56/(4.06 - 1);
                    ROS_INFO_STREAM(angle1);
                    pCAN_DEVICE->angle1 = angle1;
                    // pCAN_DEVICE->angle1 = vol1/2*105/4000+5-18; //因为输入电压是10v，所以除以2;-18是修正零漂
                    std_msgs::Int64 data_receive1;
                    data_receive1.data = pCAN_DEVICE->angle1;
                    pCAN_DEVICE->pub_c1->publish(data_receive1);

                    // 2号角度传感器-拨禾论
                    int vol2 = (high1 << 8 | low1);
                    float vol2_norm = float(vol2)/1000;
                    float angle2 = 0 + vol2_norm * 39.13/(3.92 - 0.72);
                    pCAN_DEVICE->angle2 = angle2;
                    // pCAN_DEVICE->angle2 = vol2/2*105/4000+5-18;
                    std_msgs::Int64 data_receive2;
                    data_receive2.data = pCAN_DEVICE->angle2;
                    pCAN_DEVICE->pub_c2->publish(data_receive2);

                    ROS_INFO(
                            "Channel %02d Receive msg:%04d ID:%02X Data:0x %02X %02X %02X %02X %02X %02X %02X %02X angle1:%05d angle2:%05d",
                            pCAN_DEVICE->channel+1, pCAN_DEVICE->count, rec[j].ID,
                            rec[j].Data[0], rec[j].Data[1], rec[j].Data[2], rec[j].Data[3],
                            rec[j].Data[4], rec[j].Data[5], rec[j].Data[6], rec[j].Data[7], pCAN_DEVICE->angle1, pCAN_DEVICE->angle2);
                }
                else if (rec[j].ID == 0x0281) { //4-7接口的数据
                    unsigned char high4, low4;
                    high4 = rec[j].Data[1];
                    low4 = rec[j].Data[0];
                    unsigned char high5, low5;
                    high5 = rec[j].Data[3];
                    low5 = rec[j].Data[2];
                    unsigned char high6, low6;
                    high6 = rec[j].Data[5];
                    low6 = rec[j].Data[4];
                    unsigned char high7, low7;
                    high7 = rec[j].Data[7];
                    low7 = rec[j].Data[6];

                    /// turn号角度传感器
                    if ((high4 << 8 | low4) > 60000 || (high5 << 8 | low5) > 60000)
                        continue;
                    int vol5 = (high4 << 8 | low4);
                    ROS_INFO_STREAM(vol5);
                    pCAN_DEVICE->angle_turn = vol5;
                    std_msgs::Float32 data_receive_turn;
                    data_receive_turn.data = vol5;
                    pCAN_DEVICE->pub_turn_c6->publish(data_receive_turn);

                    // speed号角度传感器
                    int vol6 = (high5 << 8 | low5);
                    ROS_INFO_STREAM(vol6);
                    pCAN_DEVICE->angle_turn = vol6;
                    std_msgs::Float32 data_receive_speed;
                    data_receive_speed.data = vol6;
                    pCAN_DEVICE->pub_speed_c7->publish(data_receive_speed);

                    ROS_INFO(
                            "Channel %02d Receive msg:%04d ID:%02X Data:0x %02X %02X %02X %02X %02X %02X %02X %02X turn:%04f",
                            pCAN_DEVICE->channel+1, pCAN_DEVICE->count, rec[j].ID,
                            rec[j].Data[0], rec[j].Data[1], rec[j].Data[2], rec[j].Data[3],
                            rec[j].Data[4], rec[j].Data[5], rec[j].Data[6], rec[j].Data[7], vol5/1000);
                }

                //// 采集卡2：
                else if (rec[j].ID == 0x0182) // 0-3接口的数据
                {
                    unsigned char high0, low0;
                    high0 = rec[j].Data[1];
                    low0 = rec[j].Data[0];
                    unsigned char high1, low1;
                    high1 = rec[j].Data[3];
                    low1 = rec[j].Data[2];
                    unsigned char high2, low2;
                    high2 = rec[j].Data[5];
                    low2 = rec[j].Data[4];
                    unsigned char high3, low3;
                    high3 = rec[j].Data[7];
                    low3 = rec[j].Data[6];

                    // 力矩传感器
                    float torque = (high0 << 8 | low0);
                    pCAN_DEVICE->torque = torque/10000*100;
                    if(pCAN_DEVICE->torque < 0.05) // 太小的时候过滤一下
                    {
                        pCAN_DEVICE->torque = 0;
                    }
                    std_msgs::Float32 data_receive4;
                    data_receive4.data = pCAN_DEVICE->torque;
                    pCAN_DEVICE->pub_c4->publish(data_receive4);

                    //电流检测（钳流表）
                    int current_cm7290_int = (high1 << 8 | low1);
                    float current_cm7290 = current_cm7290_int;
                    current_cm7290 = current_cm7290/100;
                    std_msgs::Float32 data_current_cm7290;
                    data_current_cm7290.data = current_cm7290;
                    pCAN_DEVICE->pub_c5_cm7290->publish(data_current_cm7290);

                    //电流检测（电流环）
                    int current_int = (high2 << 8 | low2);
                    float current = current_int;
                    float rms = 0.01442504 * pCAN_DEVICE->calculate_rms0(current) - 1.69037332;
                    std_msgs::Float32 data_current_raw;
                    data_current_raw.data = current;
                    pCAN_DEVICE->pub_c5_raw->publish(data_current_raw);
                    std_msgs::Float32 data_current;
                    data_current.data = rms;
                    pCAN_DEVICE->pub_c5->publish(data_current);

                    ROS_INFO(
                            "Channel %02d Receive msg:%04d ID:%02X Data:0x %02X %02X %02X %02X %02X %02X %02X %02X angle1:%05d angle2:%05d",
                            pCAN_DEVICE->channel+1, pCAN_DEVICE->count, rec[j].ID,
                            rec[j].Data[0], rec[j].Data[1], rec[j].Data[2], rec[j].Data[3],
                            rec[j].Data[4], rec[j].Data[5], rec[j].Data[6], rec[j].Data[7], pCAN_DEVICE->angle1, pCAN_DEVICE->angle2);
                }

                else if (rec[j].ID == 0x0282) // 4-7接口的数据
                {
                    unsigned char high4, low4;
                    high4 = rec[j].Data[1];
                    low4 = rec[j].Data[0];
                    unsigned char high5, low5;
                    high5 = rec[j].Data[3];
                    low5 = rec[j].Data[2];
                    unsigned char high6, low6;
                    high6 = rec[j].Data[5];
                    low6 = rec[j].Data[4];
                    unsigned char high7, low7;
                    high7 = rec[j].Data[7];
                    low7 = rec[j].Data[6];

                    //电流检测（电机4接口-暂时没有使用）
                    int current_int4 = (high4 << 8 | low4);
                    if(current_int4>2000)
                        current_int4 = 0;
                    float current4 = current_int4;
                    float rms4 = 0.01442504 * pCAN_DEVICE->calculate_rms1(current4) - 1.69037332;
                    std_msgs::Float32 data_current_raw4;
                    data_current_raw4.data = current4;
                    pCAN_DEVICE->pub_c_motor4_raw->publish(data_current_raw4);
                    std_msgs::Float32 data_current4;
                    data_current4.data = rms4;
                    pCAN_DEVICE->pub_c_motor4->publish(data_current4);
                    //电流检测（电机5接口-对应3号驱动器）
                    int current_int5 = (high5 << 8 | low5);
                    float current5 = current_int5;
                    if(current5>2000)
                        current5 = 0;
                    ROS_INFO_STREAM("current5: "<<current5);
                    float rms5 = 0.01442504 * pCAN_DEVICE->calculate_rms2(current5) - 1.69037332-0.35;
                    std_msgs::Float32 data_current_raw5;
                    data_current_raw5.data = current5;
                    pCAN_DEVICE->pub_c_motor5_raw->publish(data_current_raw5);
                    std_msgs::Float32 data_current5;
                    data_current5.data = rms5;
                    pCAN_DEVICE->pub_c_motor5->publish(data_current5);
                    //电流检测（电机6接口-对应2号驱动器）
                    int current_int6 = (high6 << 8 | low6);
                    float current6 = current_int6;
                    if(current6>2000)
                        current6 = 0;
                    ROS_INFO_STREAM("current6: "<<current6);
                    float rms6 = 0.01442504 * pCAN_DEVICE->calculate_rms3(current6) - 1.69037332+1;
                    std_msgs::Float32 data_current_raw6;
                    data_current_raw6.data = current6;
                    pCAN_DEVICE->pub_c_motor6_raw->publish(data_current_raw6);
                    std_msgs::Float32 data_current6;
                    data_current6.data = rms6;
                    pCAN_DEVICE->pub_c_motor6->publish(data_current6);
                    //电流检测（电机7接口-对应1号驱动器）
                    int current_int7 = (high7 << 8 | low7);
                    float current7 = current_int7;
                    if(current7>2000)
                        current7 = 0;
                    ROS_INFO_STREAM("current7: "<<current7);
                    float rms7 = 0.01442504 * pCAN_DEVICE->calculate_rms4(current7) - 1.69037332+0.75;
                    std_msgs::Float32 data_current_raw7;
                    data_current_raw7.data = current7;
                    pCAN_DEVICE->pub_c_motor7_raw->publish(data_current_raw7);
                    std_msgs::Float32 data_current7;
                    data_current7.data = rms7;
                    pCAN_DEVICE->pub_c_motor7->publish(data_current7);


                    ROS_INFO(
                            "Channel %02d Receive msg:%04d ID:%02X Data:0x %02X %02X %02X %02X %02X %02X %02X %02X angle1:%05d angle2:%05d",
                            pCAN_DEVICE->channel+1, pCAN_DEVICE->count, rec[j].ID,
                            rec[j].Data[0], rec[j].Data[1], rec[j].Data[2], rec[j].Data[3],
                            rec[j].Data[4], rec[j].Data[5], rec[j].Data[6], rec[j].Data[7], pCAN_DEVICE->angle1, pCAN_DEVICE->angle2);
                }

                //// 车速数据
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

float CAN_DEVICE::calculate_rms0(float current_now)
{
    current_buffer1.insert(current_buffer1.begin(),current_now);
    current_buffer1.pop_back();

    float power2sum = 0;
    for(float & iter : current_buffer1)
    {
        power2sum = power2sum + iter*iter;
    }
    float rms = sqrt(power2sum/buffer_length);
//    ROS_INFO_STREAM("rms :"<<rms);
    return rms;
}
float CAN_DEVICE::calculate_rms1(float current_now)
{
    current_buffer1.insert(current_buffer1.begin(),current_now);
    current_buffer1.pop_back();

    float power2sum = 0;
    for(float & iter : current_buffer1)
    {
        power2sum = power2sum + iter*iter;
    }
    float rms = sqrt(power2sum/buffer_length);
//    ROS_INFO_STREAM("rms :"<<rms);
    return rms;
}
float CAN_DEVICE::calculate_rms2(float current_now)
{
    current_buffer2.insert(current_buffer2.begin(),current_now);
    current_buffer2.pop_back();

    float power2sum = 0;
    for(float & iter : current_buffer2)
    {
        power2sum = power2sum + iter*iter;
    }
    float rms = sqrt(power2sum/buffer_length);
//    ROS_INFO_STREAM("rms :"<<rms);
    return rms;
}
float CAN_DEVICE::calculate_rms3(float current_now)
{
    current_buffer3.insert(current_buffer3.begin(),current_now);
    current_buffer3.pop_back();

    float power2sum = 0;
    for(float & iter : current_buffer3)
    {
        power2sum = power2sum + iter*iter;
    }
    float rms = sqrt(power2sum/buffer_length);
//    ROS_INFO_STREAM("rms :"<<rms);
    return rms;
}
float CAN_DEVICE::calculate_rms4(float current_now)
{
    current_buffer4.insert(current_buffer4.begin(),current_now);
    current_buffer4.pop_back();

    float power2sum = 0;
    for(float & iter : current_buffer4)
    {
        power2sum = power2sum + iter*iter;
    }
    float rms = sqrt(power2sum/buffer_length);
//    ROS_INFO_STREAM("rms :"<<rms);
    return rms;
}

void CAN_DEVICE::transmit_msg(VCI_CAN_OBJ send[1], char com[10]) //发送函数
{
    if (VCI_Transmit(VCI_USBCAN2, 0, channel, send, 1) == 1) {

        ROS_INFO("Send msg:%04d ID:%04X Data:0x %02X %02X %02X %02X %02X %02X %02X %02X COMMAND:%s", count,
                 send[0].ID,
                 send[0].Data[0], send[0].Data[1], send[0].Data[2], send[0].Data[3],
                 send[0].Data[4], send[0].Data[5], send[0].Data[6], send[0].Data[7], com);

        count++;
    }
}

void CAN_DEVICE::control_height(int mode) //驱动拨禾轮和割台的高度调节
{
    // 设置电机为CAN控制，速度模式
    VCI_CAN_OBJ msg[1];

    if (mode == 100) {
        msg[0].ID = 0x00000200;
        msg[0].SendType = 0;
        msg[0].RemoteFlag = 0;
        msg[0].ExternFlag = 0;
        msg[0].DataLen = 8;

        msg[0].Data[0] = 0x01;
        msg[0].Data[1] = 0x11;
        msg[0].Data[2] = 0x09;
        msg[0].Data[3] = 0x00;
        msg[0].Data[4] = 0x00;
        msg[0].Data[5] = 0x00;
        msg[0].Data[6] = 0x00;
        msg[0].Data[7] = 0x19;
        transmit_msg(msg, "set  Stady");
    }
    if (mode == 110) // 下降割台（y3口）
    {
        msg[0].ID = 0x00000200;
        msg[0].SendType = 0;
        msg[0].RemoteFlag = 0;
        msg[0].ExternFlag = 0;
        msg[0].DataLen = 8;

        msg[0].Data[0] = 0x01;
        msg[0].Data[1] = 0x11;
        msg[0].Data[2] = 0x09;
        msg[0].Data[3] = 0x00;
        msg[0].Data[4] = 0x04;
        msg[0].Data[5] = 0x00;
        msg[0].Data[6] = 0x00;
        msg[0].Data[7] = 0x1D;

        transmit_msg(msg, "set  down");
    }

    if (mode == 120) // 上升割台（y4）
    {
        msg[0].ID = 0x00000200;
        msg[0].SendType = 0;
        msg[0].RemoteFlag = 0;
        msg[0].ExternFlag = 0;
        msg[0].DataLen = 8;

        msg[0].Data[0] = 0x01;
        msg[0].Data[1] = 0x11;
        msg[0].Data[2] = 0x09;
        msg[0].Data[3] = 0x00;
        msg[0].Data[4] = 0x08;
        msg[0].Data[5] = 0x00;
        msg[0].Data[6] = 0x00;
        msg[0].Data[7] = 0x11;

        transmit_msg(msg, "set  up");
    }

    if (mode == 101) // 下降拨禾轮（y5口）
    {
        msg[0].ID = 0x00000200;
        msg[0].SendType = 0;
        msg[0].RemoteFlag = 0;
        msg[0].ExternFlag = 0;
        msg[0].DataLen = 8;

        msg[0].Data[0] = 0x01;
        msg[0].Data[1] = 0x11;
        msg[0].Data[2] = 0x09;
        msg[0].Data[3] = 0x00;
        msg[0].Data[4] = 0x10;
        msg[0].Data[5] = 0x00;
        msg[0].Data[6] = 0x00;
        msg[0].Data[7] = 0x09;

        transmit_msg(msg, "set  down bh");
    }

    if (mode == 102) // 上升拨禾轮（y6口）
    {
        msg[0].ID = 0x00000200;
        msg[0].SendType = 0;
        msg[0].RemoteFlag = 0;
        msg[0].ExternFlag = 0;
        msg[0].DataLen = 8;

        msg[0].Data[0] = 0x01;
        msg[0].Data[1] = 0x11;
        msg[0].Data[2] = 0x09;
        msg[0].Data[3] = 0x00;
        msg[0].Data[4] = 0x20;
        msg[0].Data[5] = 0x00;
        msg[0].Data[6] = 0x00;
        msg[0].Data[7] = 0x39;

        transmit_msg(msg, "set  up bh");
    }
}

void CAN_DEVICE::open_receive() {
    // 开启CAN信号接受线程
    int ret;
    m_run0 = 1;
    ret = pthread_create(&receive_thread, NULL, receive_func, this);
    ROS_INFO_STREAM("receive_thread_create.");
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

void CAN_DEVICE::init_ICAN(int id) {
    // 使能模拟量转can
    VCI_CAN_OBJ msg[1];

    msg[0].ID = 0;
    msg[0].SendType = 0;
    msg[0].RemoteFlag = 0;
    msg[0].ExternFlag = 0;
    msg[0].DataLen = 2;

    msg[0].Data[0] = 0x01;

    if(id == 1){
        msg[0].Data[1] = 0x01;
    } else if(id == 2){
        msg[0].Data[1] = 0x02;
    }

    transmit_msg(msg, "init ICAN");
}
