import matplotlib.pyplot as plt
import numpy as np

input_txt = '[2020-11-11 18:0:58:688]_speed.txt'

rev = []
motor_current = []

file = open(input_txt)

for line in file:
    line = line.strip('\n')
    line = line.split(' ')

    rev.append(float(line[-2]))
    motor_current.append(float(line[-1]))

time_axis = np.arange(0, len(rev))
file.close()

# Draw plot
fig, ax1 = plt.subplots()
ax2 = ax1.twinx()

ax1.plot(time_axis, motor_current, color="red")
ax2.plot(time_axis, rev, color="green")

ax1.set_xlabel('time')
ax1.set_ylabel('Motor current', color="red")
ax2.set_ylabel('Rev', color="green")
plt.draw()
plt.savefig("M7.png")
plt.show()
