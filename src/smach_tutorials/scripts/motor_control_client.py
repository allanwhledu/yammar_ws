# -*- coding: utf-8 -*-
# !/usr/bin/env python

import rospy
import smach
import smach_ros

from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Int32

from smach_ros import SimpleActionState
from control485.msg import DriveMotorAction

motors = [1, 1, 1, 1]
motor_goal = list()

for i in motors:
    motor_client = DriveMotorAction()
    motor_client.action_goal.goal.motor_id = i
    motor_goal.append(motor_client)

target_speed = Int32()
target_speed.data = 0

last_target = -1000


# i = Int32()
# i.data = 3

# # 这里首先定义状态节点
# class init(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['init_done'])
#
#     def execute(self, userdata):
#         rospy.sleep(2)
#         return 'init_done'
#
#
# class foo(smach.State):
#     def __init__(self):
#         smach.State.__init__(self, outcomes=['foo_succeeded', 'preempted'])
#
#     def execute(self, userdata):
#         for idx in range(5):
#             if self.preempt_requested():
#                 print "state foo is being preempted!!!"
#                 self.service_preempt()
#                 return 'preempted'
#             rospy.sleep(0.2)
#         return 'foo_succeeded'
#
#
# # 这里定义的是判断转移的函数
# def child_term_cb(outcome_map):
#     if outcome_map['SPEEDUP_FH'] == 'foo_succeeded':
#         return True
#     elif outcome_map['CHECK_FH'] == 'invalid':
#         return True
#     else:
#         return False
#
#
# def out_cb(outcome_map):
#     if outcome_map['CHECK_FH'] == 'invalid':
#         return 'foo_done'
#     elif outcome_map['SPEEDUP_FH'] == 'foo_succeeded':
#         return 'foo_reset'
#     else:
#         return 'foo_reset'
#
#
# def child_term_cb2(outcome_map):
#     if outcome_map['SPEEDUP_PF'] == 'foo_succeeded':
#         return True
#     elif outcome_map['CHECK_PF'] == 'invalid':
#         return True
#     else:
#         return False
#
#
# def out_cb2(outcome_map):
#     if outcome_map['CHECK_PF'] == 'invalid':
#         return 'foo_done'
#     elif outcome_map['SPEEDUP_PF'] == 'foo_succeeded':
#         return 'foo_reset'
#     else:
#         return 'foo_done'

# for publish result
pub = rospy.Publisher('smach_fback', Int32, queue_size=1)
class end(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end_succeeded'])

    def execute(self, userdata):
        msg = Int32()
        msg.data = 1
        pub.publish(msg)
        # rospy.sleep(3.0)
        # msg.data = 0
        # pub.publish(msg)
        return 'end_succeeded'

def monitor_cb(self, msg):

    global last_target
    if last_target != msg.data:

        motor_goal[0].action_goal.goal.target_speed = 2000 * msg.data
        motor_goal[1].action_goal.goal.target_speed = 4000 * msg.data
        motor_goal[2].action_goal.goal.target_speed = 5000 * msg.data
        motor_goal[3].action_goal.goal.target_speed = 6000 * msg.data

        for motor in motor_goal:
            print motor.action_goal.goal.motor_id, ' ', motor.action_goal.goal.target_speed

        last_target = msg.data
        return False

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
                               transitions={'succeeded': 'MOTOR2',
                                            'preempted': 'MOTOR1',
                                            'aborted': 'MOTOR1'})

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
                               transitions={'succeeded': 'MOTOR4',
                                            'preempted': 'MOTOR3',
                                            'aborted': 'MOTOR3'})

        smach.StateMachine.add('MOTOR4',
                               SimpleActionState('control485',
                                                 DriveMotorAction,
                                                 goal=motor_goal[3].action_goal.goal),
                               transitions={'succeeded': 'END',
                                            'preempted': 'MOTOR4',
                                            'aborted': 'MOTOR4'})
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
    sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
