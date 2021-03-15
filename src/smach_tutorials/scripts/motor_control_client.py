#!/usr/bin/env python
# coding=utf-8

import rospy
import smach
import smach_ros
import threading

from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Int32

from smach_ros import SimpleActionState
from control485.msg import DriveMotorAction

# 同调率
cbCof = 1.2
reelCof = 1.6
pfCof = 4.44
fhCof = 3.94

# 减速比
cbRatio = 5
reelRatio = 64
pfRatio = 15
fhRatio = 10

# 电机序号
reel = 1
cb = 2
pf = 3
fh = 0

# 模式
mode = 'stop'

# 设定电机序号
# motors = [cb, reel]
# motors = [pf, cb, reel]
motors = [reel, cb, pf]
motor_goal = list()

for i in motors:
    motor_client = DriveMotorAction()
    motor_client.action_goal.goal.motor_id = i
    motor_goal.append(motor_client)

target_speed = Int32()
target_speed.data = 0
last_target = -1000

# for publish result
pub_result = rospy.Publisher('smach_fback', Int32, queue_size=1)


class end(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end_succeeded'])

    def execute(self, userdata):
        msg = Int32()
        msg.data = 1
        pub_result.publish(msg)
        return 'end_succeeded'


class Mode_classification(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_or_stop', 'operating'])

    def execute(self, userdata):
        global mode
        if mode == 'start' or mode == 'stop':
            rospy.loginfo('Start or Stop')
            return 'start_or_stop'
        else:
            return 'operating'


def monitor_cb(self, msg):
    global last_target
    global mode

    # reel_ta = reelRatio * min(50.0, min(21.23 * reelCof * msg.data + 12.3, 21.23 * 1.0 * msg.data + 21.23))
    # cb_ta = 0.5 * cbRatio * min(467.0, min(398.09 * cbCof * msg.data + 131.37, 398.09 * 1.0 * msg.data + 238.85))
    # pf_ta = pfRatio * min(187.0, min(39.16 * pfCof * msg.data + 52.47, 39.16 * 3.0 * msg.data + 90.07))
    # fh_ta = fhRatio * min(187.0, min(39.16 * fhCof * msg.data + 52.47, 39.16 * 3.0 * msg.data + 90.07))
    reel_ta = 2000 * msg.data
    cb_ta = 500
    pf_ta = 500
    fh_ta = 500

    if reel_ta > 2800:
        reel_ta = 2800
    if cb_ta > 2800:
        cb_ta = 2800
    if pf_ta > 2800:
        pf_ta = 2800
    if fh_ta > 2800:
        fh_ta = 2800

    if abs(msg.data - last_target) < 0.025:  # 恒定
        mode = 'steady'
        motor_goal[0].action_goal.goal.motor_id = pf
        motor_goal[1].action_goal.goal.motor_id = cb
        motor_goal[2].action_goal.goal.motor_id = reel
        # motor_goal[3].action_goal.goal.motor_id = fh

        # motor_goal[0].action_goal.goal.motor_id = cb
        # motor_goal[1].action_goal.goal.motor_id = cb
        # motor_goal[2].action_goal.goal.motor_id = cb
        motor_goal[0].action_goal.goal.target_speed = pf_ta
        motor_goal[1].action_goal.goal.target_speed = cb_ta
        motor_goal[2].action_goal.goal.target_speed = reel_ta
        # motor_goal[3].action_goal.goal.target_speed = reel_ta

        for motor in motor_goal:
            print motor.action_goal.goal.motor_id, ' ', motor.action_goal.goal.target_speed

        # last_target = msg.data
        return False

    elif msg.data <= 0:  # 停机
        mode = 'stop'
        motor_goal[0].action_goal.goal.motor_id = reel
        motor_goal[1].action_goal.goal.motor_id = cb
        motor_goal[2].action_goal.goal.motor_id = pf
        # motor_goal[3].action_goal.goal.motor_id = fh

        # motor_goal[0].action_goal.goal.motor_id = cb
        # motor_goal[1].action_goal.goal.motor_id = cb
        # motor_goal[2].action_goal.goal.motor_id = cb
        motor_goal[0].action_goal.goal.target_speed = 0
        motor_goal[1].action_goal.goal.target_speed = 0
        motor_goal[2].action_goal.goal.target_speed = 0
        # motor_goal[3].action_goal.goal.target_speed = 0

        for motor in motor_goal:
            print motor.action_goal.goal.motor_id, ' ', motor.action_goal.goal.target_speed

        last_target = msg.data
        return False

    elif mode == 'stop' and msg.data - last_target >= 0.025:  # 启动
        mode = 'start'
        motor_goal[0].action_goal.goal.motor_id = pf
        motor_goal[1].action_goal.goal.motor_id = cb
        motor_goal[2].action_goal.goal.motor_id = reel
        # motor_goal[3].action_goal.goal.motor_id = reel
        # motor_goal[0].action_goal.goal.motor_id = cb
        # motor_goal[1].action_goal.goal.motor_id = cb
        # motor_goal[2].action_goal.goal.motor_id = cb
        motor_goal[0].action_goal.goal.target_speed = pf_ta
        motor_goal[1].action_goal.goal.target_speed = cb_ta
        motor_goal[2].action_goal.goal.target_speed = reel_ta
        # motor_goal[3].action_goal.goal.target_speed = reel_ta

        for motor in motor_goal:
            print motor.action_goal.goal.motor_id, ' ', motor.action_goal.goal.target_speed

        last_target = msg.data
        return False

    elif last_target < msg.data and msg.data - last_target >= 0.025:  # 加速
        mode = 'speedup'
        motor_goal[0].action_goal.goal.motor_id = reel # 加速这个地方比较特殊，只有reel变化
        motor_goal[1].action_goal.goal.motor_id = cb
        motor_goal[2].action_goal.goal.motor_id = pf
        # motor_goal[3].action_goal.goal.motor_id = reel
        # motor_goal[0].action_goal.goal.motor_id = cb
        # motor_goal[1].action_goal.goal.motor_id = cb
        # motor_goal[2].action_goal.goal.motor_id = cb
        motor_goal[0].action_goal.goal.target_speed = reel_ta
        motor_goal[1].action_goal.goal.target_speed = cb_ta
        motor_goal[2].action_goal.goal.target_speed = pf_ta
        # motor_goal[3].action_goal.goal.target_speed = reel_ta

        for motor in motor_goal:
            print motor.action_goal.goal.motor_id, ' ', motor.action_goal.goal.target_speed

        last_target = msg.data
        return False

    elif last_target > msg.data and last_target - msg.data >= 0.025:  # 减速
        mode = 'speeddown'
        motor_goal[0].action_goal.goal.motor_id = reel # 减速也特殊
        motor_goal[1].action_goal.goal.motor_id = cb
        motor_goal[2].action_goal.goal.motor_id = pf
        # motor_goal[3].action_goal.goal.motor_id = fh
        # motor_goal[0].action_goal.goal.motor_id = cb
        # motor_goal[1].action_goal.goal.motor_id = cb
        # motor_goal[2].action_goal.goal.motor_id = cb
        motor_goal[0].action_goal.goal.target_speed = reel_ta
        motor_goal[1].action_goal.goal.target_speed = cb_ta
        motor_goal[2].action_goal.goal.target_speed = pf_ta
        # motor_goal[3].action_goal.goal.target_speed = fh_ta
        for motor in motor_goal:
            print motor.action_goal.goal.motor_id, ' ', motor.action_goal.goal.target_speed

        last_target = msg.data
        return False

    else:
        return True


def main():
    global motor_client

    rospy.init_node("preemption_example")

    # # 配置第1个并行容器
    # foo_concurrence = smach.Concurrence(outcomes=['foo_reset', 'foo_done'],
    #                                     default_outcome='foo_reset',
    #                                     child_termination_cb=child_term_cb,
    #                                     outcome_cb=out_cb)
    # with foo_concurrence:
    #     smach.Concurrence.add('SPEEDUP_FH', foo())
    #     smach.Concurrence.add('CHECK_FH', smach_ros.MonitorState("/sm_reset", Empty, monitor_cb))
    #
    # # 配置第2个并行容器
    # foo_concurrence2 = smach.Concurrence(outcomes=['foo_reset', 'foo_done'],
    #                                      default_outcome='foo_reset',
    #                                      child_termination_cb=child_term_cb2,
    #                                      outcome_cb=out_cb2)
    # with foo_concurrence2:
    #     smach.Concurrence.add('SPEEDUP_PF', foo())
    #     smach.Concurrence.add('CHECK_PF', smach_ros.MonitorState("/sm_reset2", Empty, monitor_cb))

    # 配置状态机
    sm = smach.StateMachine(outcomes=['DONE'])
    with sm:
        smach.StateMachine.add('WAIT', smach_ros.MonitorState("/modified_car_speed", Float32, monitor_cb),
                               transitions={'invalid': 'MOTOR1', 'valid': 'WAIT', 'preempted': 'WAIT'})
        smach.StateMachine.add('MOTOR1',
                               SimpleActionState('control485',
                                                 DriveMotorAction,
                                                 goal=motor_goal[0].action_goal.goal),
                               transitions={'succeeded': 'Mode_classification',
                                            'preempted': 'MOTOR1',
                                            'aborted': 'MOTOR1'})
        smach.StateMachine.add('Mode_classification', Mode_classification(),
                               transitions={'start_or_stop': 'MOTOR2',  # 直达end就是单电机控制
                                            'operating': 'END'})

        smach.StateMachine.add('MOTOR2',
                               SimpleActionState('control485',
                                                 DriveMotorAction,
                                                 goal=motor_goal[1].action_goal.goal),
                               transitions={'succeeded': 'MOTOR3',
                                            'preempted': 'MOTOR2',
                                            'aborted': 'MOTOR2'})

        smach.StateMachine.add('MOTOR3',
                               SimpleActionState('control485',
                                                 DriveMotorAction,
                                                 goal=motor_goal[2].action_goal.goal),
                               transitions={'succeeded': 'END',
                                            'preempted': 'MOTOR3',
                                            'aborted': 'MOTOR3'})
        # smach.StateMachine.add('MOTOR4',
        #                        SimpleActionState('control485',
        #                                          DriveMotorAction,
        #                                          goal=motor_goal[3].action_goal.goal),
        #                        transitions={'succeeded': 'END',
        #                                     'preempted': 'MOTOR4',
        #                                     'aborted': 'MOTOR4'})

        smach.StateMachine.add('END',
                               end(), transitions={'end_succeeded': 'WAIT'})

        # # 第一个节点为普通节点，调用init节点函数，若节点函数返回值为init_done则转移到下一个节点FH
        # smach.StateMachine.add('TRY_SPEEDUP_FH', init(), transitions={'init_done': 'FH'})
        # # 第二个节点为并行节点，调用foo_concurrence节点函数，同样定义状态转移规则
        # smach.StateMachine.add('FH', foo_concurrence,
        #                        transitions={'foo_reset': 'TRY_AGAIN_FH', 'foo_done': 'TRY_SPEEDUP_PF'})
        # smach.StateMachine.add('TRY_AGAIN_FH', foo(),
        #                        transitions={'foo_succeeded': 'FH', 'preempted': 'TRY_SPEEDUP_FH'})
        # smach.StateMachine.add('TRY_SPEEDUP_PF', init(), transitions={'init_done': 'PF'})
        # smach.StateMachine.add('PF', foo_concurrence2, transitions={'foo_reset': 'TRY_AGAIN_PF', 'foo_done': 'DONE'})
        # smach.StateMachine.add('TRY_AGAIN_PF', foo(),
        #                        transitions={'foo_succeeded': 'PF', 'preempted': 'TRY_SPEEDUP_PF'})

    # 插入内部状态监控器
    sis = smach_ros.IntrospectionServer('smach_server', sm, '/INPUT_COMMAND')

    # 运行状态机
    sis.start()

    # 创建线程用于接受ctrl+c
    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()

    # Request the container to preempt
    sm.request_preempt()

    # Block until everything is preempted
    # (you could do something more complicated to get the execution outcome if you want it)
    smach_thread.join()
    sis.stop()


if __name__ == "__main__":
    main()
