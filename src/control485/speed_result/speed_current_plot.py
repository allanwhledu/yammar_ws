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


input_txt = '[2021-1-15 14:26:54:005]m1_3000.txt'

rev = []
motor_mVol_raw = []
motor_current_true = []

motor_current_rms = []
motor_current_cm720 = []

file = open(input_txt)

for line in file:
    line = line.strip('\n')
    line = line.split(' ')

    rev.append(float(line[-4]))
    motor_mVol_raw.append(float(line[-3]))
    motor_current_rms.append(float(line[-2]))
    motor_current_cm720.append(float(line[-1]))
# time_cost_M1_FH = 9 * 60 + 49
# time_cost_M7_FR = 6 * 60 + 44
time_cost_no_load = 41
time_axis = np.linspace(0, time_cost_no_load, len(rev))
file.close()

# np_motor_current = np.array(motor_current)
# np_motor_current_true = np.array(motor_current_true)
# # np_motor_current = (9.9 / 820) * np_motor_current
# np_motor_current_true = np_motor_current_true / 100
# np_avg_current = np_mov_avg(np_motor_current, 50)
# np_motor_current_rms = windows_rms(np_motor_current, 50)
# np_motor_current_true_rms = windows_rms(np_motor_current_true, 50)
motor_mVol_np = np.array(motor_mVol_raw)
motor_current_raw_np = 0.01442504 * motor_mVol_np - 1.69037332
motor_current_rms_np = np.array(motor_current_rms)
motor_current_cm720_np = np.array(motor_current_cm720)

plt.subplot(211)
plt.title('No-load Current-Time')
plt.plot(time_axis, motor_current_rms_np, color="green", label="loop rms")
plt.plot(time_axis, motor_current_cm720_np, color="red", label="cm7290")
plt.ylabel('Current /A')

plt.subplot(212)
plt.plot(time_axis, motor_current_raw_np, color="green", label="loop raw")
plt.plot(time_axis, motor_current_cm720_np, color="red", label="cm7290")
plt.ylabel('Current /A')
plt.xlabel('time /s')

# plt.subplot(312)
# plt.title('Loop sensor')
# plt.plot(time_axis, np_motor_current, color="green", label="true")
# plt.plot(time_axis, np_motor_current_rms, color="red", label="rms")
# plt.ylabel('Current /A')
#
# plt.subplot(313)
# plt.title('Gripper  - LoopSensor (RMS)')
# plt.plot(time_axis, np_motor_current_true_rms - np_motor_current_rms, color="blue")
# plt.xlabel('time /s')
plt.draw()
plt.legend()
plt.show()
# plt.savefig("motor_no_load_3000.png")

# time_long = len(time_axis)
# mean_motor_current = np.average(np_motor_current_rms[int(time_long/3):int(time_long/3*2)])
# mean_motor_current_true = np.average(np_motor_current_true_rms[int(time_long/3):int(time_long/3*2)])
# print("current and true current : ", mean_motor_current, " ", mean_motor_current_true)
