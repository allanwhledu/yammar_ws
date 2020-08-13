#!/usr/bin/python
# -*- coding: UTF-8 -*-

import matplotlib.pyplot as plt
import numpy as np

# list_time1 = []
# list_speed1 = []
#
# list_time2 = []
# list_speed2 = []
#
# list_time3 = []
# list_speed3 = []

list_time = [[], [], []]
list_speed = [[], [], []]

with open('/home/sjtu_wanghaili/yammar_ws/speed.txt', 'r') as f:
    for line in f.readlines():
        if len(line.strip()) != 31:
            print "This line msg is not standard."
            continue
        else:
            minutes = int(line.strip()[14:16])
            seconds = int(line.strip()[17:19])
            milliseconds = int(line.strip()[20:23])
            motor_number = int(line.strip()[25])
            speed = int(line.strip()[27:31])

            time = minutes*60*1000 + seconds*1000 + milliseconds

            list_time[motor_number-1].append(time)
            list_speed[motor_number-1].append(speed)

for i in [0, 1, 2]:
    try:
        first_time = list_time[i][0]
        for index in range(len(list_time[i])):
            list_time[i][index] = list_time[i][index] - first_time
    except:
        print "Motor " + str(i) + " has no data."

plt.figure()
plt.subplot(311)
plt.plot(list_time[0], list_speed[0])
plt.subplot(312)
plt.plot(list_time[1], list_speed[1])
plt.subplot(313)
plt.plot(list_time[2], list_speed[2])
plt.show()


