import matplotlib.pyplot as plt
import numpy as np


# move average function
def np_mov_avg(a, n, mode="same"):
    return np.convolve(a, np.ones((n,)) / n, mode=mode)


# rmse
def windows_rms(a, window_size):
    a2 = np.power(a, 2)
    window = np.ones((window_size, )) / window_size
    return np.sqrt(np.convolve(a2, window, mode="same"))


input_txt = '[2021-1-14 13:44:39:056]NoLoad_3000.txt'

rev = []
motor_current = []
motor_current_true = []

file = open(input_txt)

for line in file:
    line = line.strip('\n')
    line = line.split(' ')

    rev.append(float(line[-3]))
    motor_current.append(float(line[-2]))
    motor_current_true.append(float(line[-1]))
# time_cost_M1_FH = 9 * 60 + 49
# time_cost_M7_FR = 6 * 60 + 44
time_cost_no_load = 60
time_axis = np.linspace(0, time_cost_no_load, len(rev))
file.close()

np_motor_current = np.array(motor_current)
np_motor_current_true = np.array(motor_current_true)
# np_motor_current = (9.9 / 820) * np_motor_current
np_motor_current_true = np_motor_current_true / 100
np_avg_current = np_mov_avg(np_motor_current, 50)
np_motor_current_rms = windows_rms(np_motor_current, 50)
np_motor_current_true_rms = windows_rms(np_motor_current_true, 50)

plt.subplot(311)
plt.title('Gripper')
plt.plot(time_axis, np_motor_current_true, color="green", label="raw")
plt.plot(time_axis, np_motor_current_true_rms, color="red", label="rms")
plt.ylabel('Curent /A')

plt.subplot(312)
plt.title('Loop sensor')
plt.plot(time_axis, np_motor_current, color="green", label="true")
plt.plot(time_axis, np_motor_current_rms, color="red", label="rms")
plt.ylabel('Current /A')

plt.subplot(313)
plt.title('Gripper  - LoopSensor (RMS)')
plt.plot(time_axis, np_motor_current_true_rms - np_motor_current_rms, color="blue")
plt.xlabel('time /s')
plt.draw()
plt.legend()
plt.show()
plt.savefig("motor_no_load_3000.png")

time_long = len(time_axis)
mean_motor_current = np.average(np_motor_current_rms[int(time_long/3):int(time_long/3*2)])
mean_motor_current_true = np.average(np_motor_current_true_rms[int(time_long/3):int(time_long/3*2)])
print("current and true current : ", mean_motor_current, " ", mean_motor_current_true)
