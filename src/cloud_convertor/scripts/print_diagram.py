#!/usr/bin/env python
# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt

# 数据格式定义：time,car_speed,reel_speed,cb_speed,reel_current,cm7290_current,cb_current


if __name__ == '__main__':
    car_speed = np.load("m234_car_speed.npy", allow_pickle=True)
    reel_speed = np.load("m234_reel_speed.npy", allow_pickle=True)
    cb_speed = np.load("m234_cb_speed.npy", allow_pickle=True)
    pf_speed = np.load("m234_pf_speed.npy", allow_pickle=True)
    reel_current = np.load("m234_reel_current.npy", allow_pickle=True)
    cb_current = np.load("m234_cb_current.npy", allow_pickle=True)
    pf_current = np.load("m234_pf_current.npy", allow_pickle=True)
    cm7290_current = np.load("m234_cm7290_current.npy", allow_pickle=True)

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
        fig1 = plt.figure(1)
        ax1 = plt.subplot(211)
        plt.plot(reel_speed[..., 0], reel_speed[..., 1], 'g', label='reel_speed')
        plt.plot(cb_speed[..., 0], cb_speed[..., 1], 'b', label='cb_speed')
        plt.plot(pf_speed[..., 0], pf_speed[..., 1], 'r', label='pf_speed')
        plt.title('Control motors based on car speed.\npriority and tracking', fontsize=30)
        # 设置坐标刻度大小
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        ax1.set_xlabel('time s', fontsize=20)
        ax1.set_ylabel('speed n/min', fontsize=20)
        # 设置图例字体大小
        ax1.legend(loc='center right', fontsize=20)

        ax2 = plt.subplot(212, sharex=ax1)
        plt.step(car_speed[..., 0], car_speed[..., 1], 'o', where='post', label='car_speed')
        # 设置坐标标签字体大小
        ax2.set_xlabel('time s', fontsize=20)
        ax2.set_ylabel('speed m/s', fontsize=20)
        # 设置坐标刻度大小
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置图例字体大小
        ax2.legend(loc='center right', fontsize=20)
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

        fig3 = plt.figure(2)
        ax4 = plt.subplot(411)
        plt.plot(reel_current[..., 0], (reel_current[..., 1] + 1.8)*8, 'r', label='reel_current')
        # plt.plot(cm7290_current[..., 0], cm7290_current[..., 1], 'y', label='cm7290_current')
        plt.title('Control motors based on car speed.\nCurrent compared monitoring', fontsize=30)
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        ax4.set_ylabel('current A', fontsize=20)
        # 设置图例字体大小
        ax4.legend(fontsize=20, loc='upper right')
        # 设置坐标刻度大小

        ax5 = plt.subplot(412)
        plt.plot(cb_current[..., 0], cb_current[..., 1]+0.4, 'g', label='cb_current')
        # plt.plot(pf_current[..., 0], pf_current[..., 1]+0.4, 'b', label='pf_current')
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        ax5.set_xlabel('time s', fontsize=20)
        ax5.set_ylabel('current A', fontsize=20)
        # 设置图例字体大小
        ax5.legend(fontsize=20, loc='upper right')

        ax6 = plt.subplot(413)
        # plt.plot(cb_current[..., 0], cb_current[..., 1]+0.4, 'g', label='cb_current')
        plt.plot(pf_current[..., 0], pf_current[..., 1]+0.4, 'b', label='pf_current')
        plt.xticks(fontsize=20)
        plt.yticks(fontsize=20)
        # 设置坐标标签字体大小
        ax6.set_xlabel('time s', fontsize=20)
        ax6.set_ylabel('current A', fontsize=20)
        # 设置图例字体大小
        ax6.legend(fontsize=20, loc='upper right')

        ax7 = plt.subplot(414, sharex=ax4)
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
