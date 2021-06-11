#!/usr/bin/env python
# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt

# 数据格式定义：time,car_speed,reel_speed,cb_speed,reel_current,cm7290_current,cb_current


def down_sample(sample_data, scale=50):
    length = sample_data.shape[0]
    to_del = []
    for i in range(length):
        if i % scale != 0:
            to_del.append(i)
    sample_data = np.delete(sample_data, to_del, axis=0)
    return sample_data


def remove_end_part(sample_data, how_long=100):
    length = sample_data.shape[0]
    to_del = np.arange(length-how_long, length, 1)
    sample_data = np.delete(sample_data, to_del, axis=0)
    return sample_data

def remove_error(sample_data):
    length = sample_data.shape[0]
    for line_index in range(length):
        if sample_data[line_index, 1] > 500:
            sample_data[line_index, 1] = 0


if __name__ == '__main__':

    # x1 = [1.2,3.2,5.5,7.3,9.5]
    # y1 = [10,10,10,10,10]
    # x2 = [2.4,4.2,6.6,8.3,10.4]
    # y2 = [11,11,11,11,11]
    # plt.plot(x1,y1,'r')
    # plt.plot(x2,y2,'b')
    # plt.show()

    # reel_speed = reel_speed[0:2225, ...]
    # reel_speed = np.insert(reel_speed, 657, [159.42242, 0], 0)
    # cb_speed = cb_speed[0:2225, ...]
    # cb_speed = np.delete(cb_speed, (1666, 1915, 2154), 0)
    # cb_speed = np.delete(cb_speed, 656, 0)

    draw_what = 'speed'  # speed or current
    if draw_what == 'speed':
        car_speed = np.load("mall_car_speed.npy", allow_pickle=True)
        m1_speed = np.load("mall_m1_speed.npy", allow_pickle=True)
        m2_speed = np.load("mall_m2_speed.npy", allow_pickle=True)
        m3_speed = np.load("mall_m3_speed.npy", allow_pickle=True)
        m4_speed = np.load("mall_m4_speed.npy", allow_pickle=True)
        m5_speed = np.load("mall_m5_speed.npy", allow_pickle=True)
        m11_speed = np.load("mall_m11_speed.npy", allow_pickle=True)
        m7_speed = np.load("mall_m7_speed.npy", allow_pickle=True)
        m8_speed = np.load("mall_m8_speed.npy", allow_pickle=True)
        m9_speed = np.load("mall_m9_speed.npy", allow_pickle=True)
        m10_speed = np.load("mall_m10_speed.npy", allow_pickle=True)

        # 修改数据
        m3_speed[0:79, 1] = 0


        m1_speed_freq = round((m1_speed[-1,0] - m1_speed[0,0]) / m1_speed.shape[0], 2)
        m2_speed_freq = round((m2_speed[-1,0] - m2_speed[0,0]) / m2_speed.shape[0], 2)
        m3_speed_freq = round((m3_speed[-1,0] - m3_speed[0,0]) / m3_speed.shape[0], 2)
        m4_speed_freq = round((m4_speed[-1,0] - m4_speed[0,0]) / m4_speed.shape[0], 2)
        m5_speed_freq = round((m5_speed[-1,0] - m5_speed[0,0]) / m5_speed.shape[0], 2)
        m11_speed_freq = round((m11_speed[-1,0] - m11_speed[0,0]) / m11_speed.shape[0], 2)
        m7_speed_freq = round((m7_speed[-1,0] - m7_speed[0,0]) / m7_speed.shape[0], 2)
        m8_speed_freq = round((m8_speed[-1,0] - m8_speed[0,0]) / m8_speed.shape[0], 2)
        m9_speed_freq = round((m9_speed[-1,0] - m9_speed[0,0]) / m9_speed.shape[0], 2)
        m10_speed_freq = round((m10_speed[-1,0] - m10_speed[0,0]) / m10_speed.shape[0], 2)

        fig1 = plt.figure(1)
        ax1 = plt.subplot(211)
        plt.plot(m1_speed[..., 0], m1_speed[..., 1], label='m1_speed freq:' + str(m1_speed_freq))
        plt.plot(m2_speed[..., 0], m2_speed[..., 1], label='m2_speed freq:' + str(m2_speed_freq))
        plt.plot(m3_speed[..., 0], m3_speed[..., 1], label='m3_speed freq:' + str(m3_speed_freq))
        plt.plot(m4_speed[..., 0], m4_speed[..., 1], label='m4_speed freq:' + str(m4_speed_freq))
        plt.plot(m5_speed[..., 0], m5_speed[..., 1], label='m5_speed freq:' + str(m5_speed_freq))
        plt.plot(m11_speed[..., 0], m11_speed[..., 1], label='m11_speed freq:' + str(m11_speed_freq))
        plt.plot(m7_speed[..., 0], m7_speed[..., 1], label='m7_speed freq:' + str(m7_speed_freq))
        plt.plot(m8_speed[..., 0], m8_speed[..., 1], label='m8_speed freq:' + str(m8_speed_freq))
        plt.plot(m9_speed[..., 0], m9_speed[..., 1], label='m9_speed freq:' + str(m9_speed_freq))
        plt.plot(m10_speed[..., 0], m10_speed[..., 1], label='m10_speed freq:' + str(m10_speed_freq))
        plt.title('Control motors based on car speed.\npriority and tracking', fontsize=30)
        # 设置坐标刻度大小
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        ax1.set_xlabel('time s', fontsize=20)
        ax1.set_ylabel('speed n/min', fontsize=20)
        # 设置图例字体大小
        ax1.legend(loc='center left', fontsize=20)
        ax1.set_xlim(50,150)

        ax2 = plt.subplot(212, sharex=ax1)  # 这里设置了坐标轴共享
        plt.step(car_speed[..., 0], car_speed[..., 1], 'o', where='post', label='car_speed')
        # 设置坐标标签字体大小
        ax2.set_xlabel('time s', fontsize=20)
        ax2.set_ylabel('speed m/s', fontsize=20)
        # 设置坐标刻度大小
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置图例字体大小
        ax2.legend(loc='center right', fontsize=20)
        ax2.set_xlim(50, 150)
        # plt.show()

        # ax11 = fig1.add_axes([0.2, 0.65, 0.15, 0.15])  # inside axes
        # ax11.plot(reel_speed[650:670, 0], reel_speed[650:670, 1], 'go-', label='reel_speed')
        # ax11.plot(cb_speed[650:670, 0], cb_speed[650:670, 1], 'bo-', label='cb_speed')
        # ax11.plot(pf_speed[650:670, 0], pf_speed[650:670, 1], 'bo-', label='pf_speed')
        # ax11.set_xlabel('time s')
        # ax11.set_ylabel('speed m/s')
        # ax11.set_title('Zoom in start point.')
        # # 去掉边框
        # ax11.spines['top'].set_visible(False)
        # ax11.spines['right'].set_visible(False)
        # # ax11.spines['bottom'].set_visible(False)
        # # ax11.spines['left'].set_visible(False)

        plt.show()

    elif draw_what == 'current':
        # fig2 = plt.figure(2)
        # ax3 = fig2.add_subplot(1, 1, 1)
        # ax3.plot(reel_speed[650:670, 0], reel_speed[650:670, 1], 'go-', label='reel_speed')
        # ax3.plot(cb_speed[650:670, 0], cb_speed[650:670, 1], 'bo-', label='cb_speed')
        # ax3.plot(pf_speed[650:670, 0], pf_speed[650:670, 1], 'bo-', label='pf_speed')
        # # 设置坐标刻度大小
        # plt.xticks(fontsize=20)
        # plt.yticks(fontsize=20)
        # # 设置坐标标签字体大小
        # ax3.set_xlabel('time s', fontsize=20)
        # ax3.set_ylabel('speed n/min', fontsize=20)
        # # 设置图例字体大小
        # ax3.legend(fontsize=20)
        # # plt.show()

        car_speed = np.load("mall_car_speed.npy", allow_pickle=True)
        m1_current = np.load("mall_m1_current.npy", allow_pickle=True)
        m2_current = np.load("mall_m2_current.npy", allow_pickle=True)
        m3_current = np.load("mall_m3_current.npy", allow_pickle=True)
        m4_current = np.load("mall_m4_current.npy", allow_pickle=True)
        m5_current = np.load("mall_m5_current.npy", allow_pickle=True)
        m11_current = np.load("mall_m11_current.npy", allow_pickle=True)
        m7_current = np.load("mall_m7_current.npy", allow_pickle=True)
        m8_current = np.load("mall_m8_current.npy", allow_pickle=True)
        m9_current = np.load("mall_m9_current.npy", allow_pickle=True)
        m10_current = np.load("mall_m10_current.npy", allow_pickle=True)
        cm7290_current = np.load("mall_cm7290_current.npy", allow_pickle=True)

        # m1_current = down_sample(m1_current, scale=50)
        # m2_current = down_sample(m2_current, scale=50)
        # m3_current = down_sample(m3_current, scale=50)
        # m4_current = down_sample(m4_current, scale=50)
        # m5_current = down_sample(m5_current, scale=50)
        # m11_current = down_sample(m11_current, scale=50)
        #
        # m1_current = remove_end_part(m1_current, how_long=200)
        # m2_current = remove_end_part(m2_current, how_long=200)
        # m3_current = remove_end_part(m3_current, how_long=200)
        # m4_current = remove_end_part(m4_current, how_long=200)
        # m5_current = remove_end_part(m5_current, how_long=200)
        # m11_current = remove_end_part(m11_current, how_long=200)

        # remove_error(cm7290_current)


        fig3 = plt.figure(2)
        ax4 = plt.subplot(811)
        plt.plot(m3_current[..., 0], m3_current[..., 1], 'r', label='m3_current')
        plt.title('Control motors based on car speed.\nCurrent compared monitoring', fontsize=30)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小speed
        ax4.set_ylabel('current A', fontsize=20)
        # 设置图例字体大小
        ax4.legend(fontsize=20, loc='upper right')
        # 设置坐标刻度大小

        ax5 = plt.subplot(812)
        plt.plot(m4_current[..., 0], m4_current[..., 1], 'g', label='m4_current')
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        # ax5.set_xlabel('time s', fontsize=20)
        ax5.set_ylabel('current A', fontsize=20)
        # 设置图例字体大小
        ax5.legend(fontsize=20, loc='upper right')

        ax6 = plt.subplot(813)
        plt.plot(m2_current[..., 0], m2_current[..., 1], 'b', label='m2_current')
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        # ax6.set_xlabel('time s', fontsize=20)
        ax6.set_ylabel('current A', fontsize=20)
        # 设置图例字体大小
        ax6.legend(fontsize=20, loc='upper right')

        ax7 = plt.subplot(814)
        plt.plot(m1_current[..., 0], m1_current[..., 1], 'y', label='m1_current')
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        # ax7.set_xlabel('time s', fontsize=20)
        ax7.set_ylabel('current A', fontsize=20)
        # 设置图例字体大小
        ax7.legend(fontsize=20, loc='upper right')

        ax8 = plt.subplot(815)
        plt.plot(m5_current[..., 0], m5_current[..., 1], 'k', label='m5_current')
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        # ax8.set_xlabel('time s', fontsize=20)
        ax8.set_ylabel('current A', fontsize=20)
        # 设置图例字体大小
        ax8.legend(fontsize=20, loc='upper right')

        ax9 = plt.subplot(816)
        plt.plot(m7_current[..., 0], m7_current[..., 1], 'b', label='m7_current')
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        # ax9.set_xlabel('time s', fontsize=20)
        ax9.set_ylabel('current A', fontsize=20)
        # 设置图例字体大小
        ax9.legend(fontsize=20, loc='upper right')

        ax10 = plt.subplot(817)
        plt.plot(m8_current[..., 0], m8_current[..., 1], 'r', label='m8_current')
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        # ax10.set_xlabel('time s', fontsize=20)
        ax10.set_ylabel('current A', fontsize=20)
        # 设置图例字体大小
        ax10.legend(fontsize=20, loc='upper right')

        ax7 = plt.subplot(818, sharex=ax4)
        ax7.step(car_speed[..., 0], car_speed[..., 1], 'o', where='post', label='car_speed')
        # 设置坐标刻度大小
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        ax7.set_xlabel('time s', fontsize=20)
        ax7.set_ylabel('current A', fontsize=20)
        # 设置图例字体大小
        ax7.legend(fontsize=20, loc='upper right')


        plt.show()

    elif draw_what == 'speed-single':

        car_speed = np.load("mall_car_speed.npy", allow_pickle=True)
        m1_speed = np.load("mall_m1_speed.npy", allow_pickle=True)
        m2_speed = np.load("mall_m2_speed.npy", allow_pickle=True)
        m3_speed = np.load("mall_m3_speed.npy", allow_pickle=True)
        m4_speed = np.load("mall_m4_speed.npy", allow_pickle=True)
        m5_speed = np.load("mall_m5_speed.npy", allow_pickle=True)
        m11_speed = np.load("mall_m11_speed.npy", allow_pickle=True)
        m7_speed = np.load("mall_m7_speed.npy", allow_pickle=True)
        m8_speed = np.load("mall_m8_speed.npy", allow_pickle=True)
        m9_speed = np.load("mall_m9_speed.npy", allow_pickle=True)
        m10_speed = np.load("mall_m10_speed.npy", allow_pickle=True)

        # xlim = [300,800]
        # m2_current = down_sample(m2_current, scale=50)
        fig3 = plt.figure(2)
        ax4 = plt.subplot(211)
        plt.plot(m3_speed[..., 0], m3_speed[..., 1], 'r', label='m3_speed')
        # plt.plot(car_speed[..., 0], car_speed[..., 1], 'b', label='car_speed')
        plt.title('Control motors based on car speed.\nCurrent compared monitoring', fontsize=30)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        ax4.set_ylabel('Speed', fontsize=20)
        # 设置图例字体大小
        ax4.legend(fontsize=20, loc='upper right')
        # plt.xlim(xlim)

        ax4 = plt.subplot(212)
        # plt.plot(m3_speed[..., 0], m3_speed[..., 1], 'r', label='m3_speed')
        plt.plot(car_speed[..., 0], car_speed[..., 1], 'b', label='car_speed')
        # plt.title('Control motors based on car speed.\nCurrent compared monitoring', fontsize=30)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        ax4.set_ylabel('Speed', fontsize=20)
        # 设置图例字体大小
        ax4.legend(fontsize=20, loc='upper right')
        # plt.xlim(xlim)

        plt.show()

    elif draw_what == 'current-single':

        car_speed = np.load("mall_car_speed.npy", allow_pickle=True)
        m1_current = np.load("mall_m1_current.npy", allow_pickle=True)
        m2_current = np.load("mall_m2_current.npy", allow_pickle=True)
        m3_current = np.load("mall_m3_current.npy", allow_pickle=True)
        m4_current = np.load("mall_m4_current.npy", allow_pickle=True)
        m5_current = np.load("mall_m5_current.npy", allow_pickle=True)
        m11_current = np.load("mall_m11_current.npy", allow_pickle=True)
        m7_current = np.load("mall_m7_current.npy", allow_pickle=True)
        m8_current = np.load("mall_m8_current.npy", allow_pickle=True)
        m9_current = np.load("mall_m9_current.npy", allow_pickle=True)
        m10_current = np.load("mall_m10_current.npy", allow_pickle=True)
        cm7290_current = np.load("mall_cm7290_current.npy", allow_pickle=True)

        # xlim = [300,800]
        # m2_current = down_sample(m2_current, scale=50)
        fig3 = plt.figure(2)
        ax4 = plt.subplot(211)
        plt.plot(m3_current[..., 0], m3_current[..., 1], 'r', label='m3_current')
        # plt.plot(car_speed[..., 0], car_speed[..., 1], 'b', label='car_speed')
        plt.title('Control motors based on car speed.\nCurrent compared monitoring', fontsize=30)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        ax4.set_ylabel('Speed', fontsize=20)
        # 设置图例字体大小
        ax4.legend(fontsize=20, loc='upper right')
        # plt.xlim(xlim)

        ax4 = plt.subplot(212)
        # plt.plot(m3_speed[..., 0], m3_speed[..., 1], 'r', label='m3_speed')
        plt.plot(car_speed[..., 0], car_speed[..., 1], 'b', label='car_speed')
        # plt.title('Control motors based on car speed.\nCurrent compared monitoring', fontsize=30)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        ax4.set_ylabel('Speed', fontsize=20)
        # 设置图例字体大小
        ax4.legend(fontsize=20, loc='upper right')
        # plt.xlim(xlim)

        plt.show()
