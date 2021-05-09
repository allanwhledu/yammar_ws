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
        self.play_ratio = 1
        self.init_time = rospy.get_rostime()
        self.time = 0
        self.car_speed = []
        self.m1_speed = []
        self.m2_speed = []
        self.m3_speed = []
        self.m4_speed = []
        self.m5_speed = []
        self.m6_speed = []

        self.cm7290_current = []
        self.m1_current = []
        self.m2_current = []
        self.m3_current = []
        self.m4_current = []
        self.m5_current = []
        self.m6_current = []

        # 把后边可能用到的 sub, pub 在初始化函数中定义好
        # 数据内容：time,car_speed,reel_speed,cb_speed,reel_current,cm7290_current,cb_current
        rospy.Subscriber('/car_speed', Float32, self.callback_car_speed)
        rospy.Subscriber('/motor_1_speed', Float32, self.callback_motor_1_speed)
        rospy.Subscriber('/motor_2_speed', Float32, self.callback_motor_2_speed)
        rospy.Subscriber('/motor_3_speed', Float32, self.callback_motor_3_speed)  # 注意这里临时改成了FH
        rospy.Subscriber('/motor_4_speed', Float32, self.callback_motor_4_speed)
        rospy.Subscriber('/motor_5_speed', Float32, self.callback_motor_5_speed)
        rospy.Subscriber('/motor_6_speed', Float32, self.callback_motor_6_speed)
        rospy.Subscriber('/current_cm7290', Float32, self.callback_cm7290_current)
        rospy.Subscriber('/current1_rms', Float32, self.callback_motor_1_current)
        rospy.Subscriber('/current2_rms', Float32, self.callback_motor_2_current)
        rospy.Subscriber('/current3_rms', Float32, self.callback_motor_3_current)
        rospy.Subscriber('/current4_rms', Float32, self.callback_motor_4_current)
        rospy.Subscriber('/current5_rms', Float32, self.callback_motor_5_current)
        rospy.Subscriber('/current6_rms', Float32, self.callback_motor_6_current)

        self.time_thread = threading.Thread(target=self.time_get_job)
        self.time_thread.start()
        self.callback_thread = threading.Thread(target=self.call_back_jobs)
        self.callback_thread.start()

    ## callback functions ##
    def callback_car_speed(self, data):
        msg = data.data
        self.car_speed.append(np.array([self.time, msg]))

    def callback_motor_1_speed(self, data):
        msg = data.data
        self.m1_speed.append(np.array([self.time, msg]))

    def callback_motor_2_speed(self, data):
        msg = data.data
        self.m2_speed.append(np.array([self.time, msg]))

    def callback_motor_3_speed(self, data):
        msg = data.data
        self.m3_speed.append(np.array([self.time, msg]))

    def callback_motor_4_speed(self, data):
        msg = data.data
        self.m4_speed.append(np.array([self.time, msg]))

    def callback_motor_5_speed(self, data):
        msg = data.data
        self.m5_speed.append(np.array([self.time, msg]))

    def callback_motor_6_speed(self, data):
        msg = data.data
        self.m6_speed.append(np.array([self.time, msg]))

    def callback_cm7290_current(self, data):
        msg = data.data
        self.cm7290_current.append(np.array([self.time, msg]))

    def callback_motor_1_current(self, data):
        msg = data.data
        self.m1_current.append(np.array([self.time, msg]))

    def callback_motor_2_current(self, data):
        msg = data.data
        self.m2_current.append(np.array([self.time, msg]))

    def callback_motor_3_current(self, data):
        msg = data.data
        self.m3_current.append(np.array([self.time, msg]))

    def callback_motor_4_current(self, data):
        msg = data.data
        self.m4_current.append(np.array([self.time, msg]))

    def callback_motor_5_current(self, data):
        msg = data.data
        self.m5_current.append(np.array([self.time, msg]))

    def callback_motor_6_current(self, data):
        msg = data.data
        self.m6_current.append(np.array([self.time, msg]))

    ## thread functions ##
    def call_back_jobs(self):
        rospy.spin()

    def time_get_job(self):
        while not rospy.is_shutdown():
            time_duration = rospy.get_rostime() - self.init_time
            self.time = time_duration.to_sec() / self.play_ratio
            # print 'time:' + str(self.time)

    ## save data ##
    def save_data_to_npy(self):
        # 数据内容：time,car_speed,reel_speed,cb_speed,reel_current,cm7290_current,cb_current
        car_speed_npy = np.stack(self.car_speed)
        np.save("mall_car_speed.npy", car_speed_npy)

        m1_speed_npy = np.stack(self.m1_speed)
        np.save("mall_m1_speed.npy", m1_speed_npy)

        m2_speed_npy = np.stack(self.m2_speed)
        np.save("mall_m2_speed.npy", m2_speed_npy)

        m3_speed_npy = np.stack(self.m3_speed)
        np.save("mall_m3_speed.npy", m3_speed_npy)

        m4_speed_npy = np.stack(self.m4_speed)
        np.save("mall_m4_speed.npy", m4_speed_npy)

        m5_speed_npy = np.stack(self.m5_speed)
        np.save("mall_m5_speed.npy", m5_speed_npy)

        m6_speed_npy = np.stack(self.m6_speed)
        np.save("mall_m6_speed.npy", m6_speed_npy)

        cm7290_current_npy = np.stack(self.cm7290_current)
        np.save("mall_cm7290_current.npy", cm7290_current_npy)

        m1_current_npy = np.stack(self.m1_current)
        np.save("mall_m1_current.npy", m1_current_npy)

        m2_current_npy = np.stack(self.m2_current)
        np.save("mall_m2_current.npy", m2_current_npy)

        m3_current_npy = np.stack(self.m3_current)
        np.save("mall_m3_current.npy", m3_current_npy)

        m4_current_npy = np.stack(self.m4_current)
        np.save("mall_m4_current.npy", m4_current_npy)

        m5_current_npy = np.stack(self.m5_current)
        np.save("mall_m5_current.npy", m5_current_npy)

        m6_current_npy = np.stack(self.m6_current)
        np.save("mall_m6_current.npy", m6_current_npy)


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