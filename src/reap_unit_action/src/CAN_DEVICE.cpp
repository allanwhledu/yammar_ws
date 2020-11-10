//
// Created by bit on 2020/5/8.
//

#include "CAN_DEVICE.h"


CAN_DEVICE::CAN_DEVICE(int channel_idx) {
    count = 0;
    m_run0 = 0;
    channel = channel_idx-1;
}

void CAN_DEVICE::init_CAN() {// 进行CAN信号发送
    printf(">>start CAN device !\r\n");//指示程序已运行
    if (VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1)//打开设备
    {
        printf(">>open device success!\n");//打开设备成功
    } else {
        printf(">>open device error!\n");
        // exit(1);
    }

    //初始化参数，严格参数二次开发函数库说明书。
    VCI_INIT_CONFIG config;
    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Filter = 1;//接收所有帧
    config.Timing0 = 0x03; //波特率125 Kbps
    config.Timing1 = 0x1C;
    config.Mode = 0;//正常模式

    if (VCI_InitCAN(VCI_USBCAN2, 0, channel, &config) != 1)//CAN1-channel
    {
        printf(">>Init CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }

    if (VCI_StartCAN(VCI_USBCAN2, 0, channel) != 1) {
        printf(">>Start CAN1 error\n");
        VCI_CloseDevice(VCI_USBCAN2, 0);
    }
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

void CAN_DEVICE::control_height(int mode) //驱动第num_motor号电机，速度为speed.
{
    // 设置电机为CAN控制，速度模式
    VCI_CAN_OBJ msg[1];

    if (mode == 100) // 静止
    {
        msg[0].ID = 0x0200;
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
    }

    if (mode == 110) // 上升（y3口）
    {
        msg[0].ID = 0x0200;
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
    }

    if (mode == 120) // 下降（y4）
    {
        msg[0].ID = 0x0200;
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
    }

    transmit_msg(msg, "set  height");
}

void CAN_DEVICE::closeCAN() {
    usleep(100000);//延时100ms。
    VCI_ResetCAN(VCI_USBCAN2, 0, channel);//复位CAN1通道。
    usleep(100000);//延时100ms。
    VCI_CloseDevice(VCI_USBCAN2, 0);//关闭设备。
    printf(">>close deivce success!\n");//打开设备成功
// 除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
// goto ext;
}
