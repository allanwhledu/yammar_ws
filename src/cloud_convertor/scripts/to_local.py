#!/usr/bin/env python
# coding=utf-8

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int64
import numpy as np
import requests
import threading


# 数据格式定义：time,car_speed,reel_speed,cb_speed,reel_current,cm7290_current,cb_current

class Interpreter:
    def __init__(self):
        self.init_time = rospy.get_rostime()
        self.time = 0
        self.car_speed = []
        self.reel_speed = []
        self.cb_speed = []
        self.pf_speed = []
        self.reel_current = []
        self.cm7290_current = []
        self.cb_current = []
        self.pf_current = []

        # 把后边可能用到的 sub, pub 在初始化函数中定义好
        # 数据内容：time,car_speed,reel_speed,cb_speed,reel_current,cm7290_current,cb_current
        rospy.Subscriber('/car_speed', Float32, self.callback_car_speed)
        rospy.Subscriber('/REEL_speed', Float32, self.callback_reel_speed)
        rospy.Subscriber('/CB_speed', Float32, self.callback_cb_speed)
        rospy.Subscriber('/FH_speed', Float32, self.callback_pf_speed)  # 注意这里临时改成了FH
        rospy.Subscriber('/REEL_current', Float32, self.callback_reel_current)
        rospy.Subscriber('/current_cm7290', Float32, self.callback_cm7290_current)
        rospy.Subscriber('/CB_current', Float32, self.callback_cb_current)
        rospy.Subscriber('/PF_current', Float32, self.callback_pf_current)

        self.time_thread = threading.Thread(target=self.time_get_job)
        self.time_thread.start()
        self.callback_thread = threading.Thread(target=self.call_back_jobs)
        self.callback_thread.start()

    ## callback functions ##
    def callback_car_speed(self, data):
        msg = data.data
        self.car_speed.append(np.array([self.time, msg]))

    def callback_reel_speed(self, data):
        msg = data.data
        self.reel_speed.append(np.array([self.time, msg]))

    def callback_cb_speed(self, data):
        msg = data.data
        self.cb_speed.append(np.array([self.time, msg]))

    def callback_pf_speed(self, data):
        msg = data.data
        self.pf_speed.append(np.array([self.time, msg]))

    def callback_reel_current(self, data):
        msg = data.data
        self.reel_current.append(np.array([self.time, msg]))

    def callback_cm7290_current(self, data):
        msg = data.data
        self.cm7290_current.append(np.array([self.time, msg]))

    def callback_cb_current(self, data):
        msg = data.data
        self.cb_current.append(np.array([self.time, msg]))

    def callback_pf_current(self, data):
        msg = data.data
        self.pf_current.append(np.array([self.time, msg]))

    ## thread functions ##
    def call_back_jobs(self):
        rospy.spin()

    def time_get_job(self):
        while not rospy.is_shutdown():
            time_duration = rospy.get_rostime() - self.init_time
            self.time = time_duration.to_sec()
            # print 'time:' + str(self.time)

    ## save data ##
    def save_data_to_npy(self):
        # 数据内容：time,car_speed,reel_speed,cb_speed,reel_current,cm7290_current,cb_current
        car_speed_npy = np.stack(self.car_speed)
        np.save("m234_car_speed.npy", car_speed_npy)

        reel_speed_npy = np.stack(self.reel_speed)
        np.save("m234_reel_speed.npy", reel_speed_npy)

        cb_speed_npy = np.stack(self.cb_speed)
        np.save("m234_cb_speed.npy", cb_speed_npy)

        pf_speed_npy = np.stack(self.pf_speed)
        np.save("m234_pf_speed.npy", pf_speed_npy)

        reel_current_npy = np.stack(self.reel_current)
        np.save("m234_reel_current.npy", reel_current_npy)

        cm7290_current_npy = np.stack(self.cm7290_current)
        np.save("m234_cm7290_current.npy", cm7290_current_npy)

        cb_current_npy = np.stack(self.cb_current)
        np.save("m234_cb_current.npy", cb_current_npy)

        pf_current_npy = np.stack(self.pf_current)
        np.save("m234_pf_current.npy", pf_current_npy)


if __name__ == '__main__':
    rospy.init_node('interpreter')
    try:
        convertor = Interpreter()
        while not rospy.is_shutdown():
            print 'Converting...'
            rospy.sleep(1)
        convertor.save_data_to_npy()
        print 'All data have been save as npy files.'
        convertor.callback_thread.join()
        convertor.time_thread.join()
        print 'Convertor exited.'

    except rospy.ROSInterruptException:
        pass
