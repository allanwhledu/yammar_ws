#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int64
import requests
import threading


url = "http://81.68.88.148:8099/device/postData"

class Interpreter:
    def __init__(self):
        self.user_name = 'whl'
        self.var0 = 0
        self.var1 = 0
        self.var2 = 0
        self.var3 = 0
        self.var4 = 0
        self.init_time = rospy.get_rostime()

        # 把后边可能用到的 sub, pub 在初始化函数中定义好
        rospy.Subscriber('/current_rms', Float32, self.callback_x)
        rospy.Subscriber('/motor_test_speed', Int64, self.callback_traj_x)
        rospy.Subscriber('/current_cm7290', Float32, self.callback_traj_y)

        add_thread = threading.Thread(target=self.thread_job)
        add_thread.start()

        self.var0 = 1

    def callback_x(self, data):
        msg = data.data
        self.var3 = msg

    def callback_traj_x(self, data):
        msg = float(data.data)
        self.var1 = msg

    def callback_traj_y(self, data):
        msg = data.data
        self.var2 = msg

    def thread_job(self):
        rospy.spin()

    def convertor(self):
        var0, var1, var2, var3, var4 = self.transformer()
        # 字典形式写入数据
        dic = {"user_name": self.user_name, "state": var0,
               "traj_x": var1, "traj_y": var2,
               "var_x": var3, "var_t": var4}
        # 组织成最后的数据包
        payload = {'data': str(dic)}

        files = []

        headers = {}

        response = requests.request("POST", url, headers=headers, data=payload, files=files)

        print(response.text)

    def transformer(self):
        self.var4 = rospy.get_rostime() - self.init_time
        self.var4 = float(self.var4.to_nsec())/1000000
        return self.var0, self.var1, self.var2, self.var3, self.var4


if __name__ == '__main__':
    rospy.init_node('interpreter')
    try:
        convertor = Interpreter()
    except rospy.ROSInterruptException:
        pass
    while not rospy.is_shutdown():
        rospy.sleep(1)
        convertor.convertor()
