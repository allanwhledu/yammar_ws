//
// Created by bit on 2020/5/8.
//

#ifndef SRC_CAN_DEVICE_H
#define SRC_CAN_DEVICE_H


#include <controlcan.h>
#include <vector>
#include <reap_unit_action/ControlReapAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<reap_unit_action::ControlReapAction> Server;

class CAN_DEVICE {
public:
    VCI_BOARD_INFO pInfo; //用来获取设备信息
    int count; //数据列表中，用来存储列表序号
    pthread_t receive_thread;
    int m_run0;
    std::vector<int> log_error; //用于debug的时候记录变量
    std::vector<VCI_CAN_OBJ> speed_error; //速度误差
    int motor;
    int channel;

    void setMotor(int motor);

    Server* pas;
    reap_unit_action::ControlReapFeedback pfd;

    CAN_DEVICE(int channel_idx);

    void init_CAN();
    friend void* receive_func(void* param);
    void transmit_msg(VCI_CAN_OBJ *send, char *com);
    void set_speed_mode(int num_motor);
    void set_motor_speed(int num_motor, int speed);
    void callFeedback(int num_motor);

    void callCurrent(int num_motor);
    void wait_current();

    void open_receive();
    void check_speed();

    void closeCAN();
};


#endif //SRC_CAN_DEVICE_H
