//
// Created by bit on 2020/5/8.
//

#ifndef SRC_CAN_DEVICE_H
#define SRC_CAN_DEVICE_H


#include <controlcan.h>
#include <vector>
#include "ros/ros.h"

class CAN_DEVICE {
public:
    VCI_BOARD_INFO pInfo; //用来获取设备信息
    int count; //数据列表中，用来存储列表序号
    int m_run0;
    std::vector<int> log_error; //用于debug的时候记录变量
    std::vector<VCI_CAN_OBJ> speed_error; //速度误差
    int channel;

    CAN_DEVICE(int channel_idx);

    void init_CAN();
    void transmit_msg(VCI_CAN_OBJ *send, char *com);

    void closeCAN();

    void control_height(int mode);
};


#endif //SRC_CAN_DEVICE_H
