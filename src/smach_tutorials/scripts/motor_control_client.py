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
m1 = 5
m2 = 7
m3 = 8
m4 = 11
m5 = 9
m6 = 10

# 模式
mode = 'stop'

# 设定电机序号
# motors = [cb, reel]
# motors = [pf, cb, reel]
motors = [m1, m2, m3, m4, m5, m6]
# motors = [reel, cb]
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

car_speed_now = 0
car_speed_last = 0


class Topic_monitor:
    def __init__(self):
        self.car_speed = 0

        rospy.Subscriber('/modified_car_speed', Float32, self.callback_car_speed)

        self.callback_thread = threading.Thread(target=self.call_back_jobs)
        self.callback_thread.start()

    ## callback functions ##
    def callback_car_speed(self, data):
        global car_speed_now
        self.car_speed = data.data
        car_speed_now = data.data

    ## thread functions ##
    def call_back_jobs(self):
        rospy.spin()

class end(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['end_succeeded'])

    def execute(self, userdata):
        # msg = Int32()
        # msg.data = 1
        # pub_result.publish(msg)
        return 'end_succeeded'

# define state Foo
class Car_speed_monitor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['speedup', 'speeddown', 'steady'])
        self.counter = 0

    def execute(self, userdata):
        global car_speed_now
        global car_speed_last
        result = None
        global last_target
        global mode

        '''
        motor target speed describtion
        '''
        motor_speed_dict = {
            'M1': None,
            'M2': None,
            'M3': None,
            'M4': None,
            'M5': 1324 / 1.4,
            'M6': None,
            'M7': 487 / 0.25,
            'M8': 933 / 0.33,
            'M9': 1193 / 1,
            'M10': 408 / 0.14,
            'M11': 2235.8 / 1,
        }

        # m1_ta = reelRatio * min(50.0, min(21.23 * reelCof * msg.data + 12.3, 21.23 * 1.0 * msg.data + 21.23))
        # m2_ta = 0.5 * cbRatio * min(467.0, min(398.09 * cbCof * msg.data + 131.37, 398.09 * 1.0 * msg.data + 238.85))
        # m3_ta = pfRatio * min(187.0, min(39.16 * pfCof * msg.data + 52.47, 39.16 * 3.0 * msg.data + 90.07))
        # m4_ta = fhRatio * min(187.0, min(39.16 * fhCof * msg.data + 52.47, 39.16 * 3.0 * msg.data + 90.07))
        m1_ta = 1000
        m2_ta = motor_speed_dict['M7']
        m3_ta = motor_speed_dict['M8']
        m4_ta = motor_speed_dict['M11']
        m5_ta = motor_speed_dict['M9']
        m6_ta = motor_speed_dict['M10']

        if m1_ta > 3000:
            m1_ta = 3000
        if m2_ta > 3000:
            m2_ta = 3000
        if m3_ta > 3000:
            m3_ta = 3000
        if m4_ta > 200:
            m4_ta = 200
        if m5_ta > 3000:
            m5_ta = 3000
        if m6_ta > 3000:
            m6_ta = 3000

        motor_goal[0].action_goal.goal.motor_id = m1
        motor_goal[1].action_goal.goal.motor_id = m2
        motor_goal[2].action_goal.goal.motor_id = m3
        motor_goal[3].action_goal.goal.motor_id = m4
        motor_goal[4].action_goal.goal.motor_id = m5
        motor_goal[5].action_goal.goal.motor_id = m6

        motor_goal[0].action_goal.goal.target_speed = m1_ta
        motor_goal[1].action_goal.goal.target_speed = m5_ta
        motor_goal[2].action_goal.goal.target_speed = m4_ta
        motor_goal[3].action_goal.goal.target_speed = m3_ta
        motor_goal[4].action_goal.goal.target_speed = m2_ta
        motor_goal[5].action_goal.goal.target_speed = m1_ta

        # for motor in motor_goal:
        #     print motor.action_goal.goal.motor_id, ' ', motor.action_goal.goal.target_speed

        # rospy.loginfo('Monitor car speed ...')
        if car_speed_now > car_speed_last:
            result = 'speedup'
        elif car_speed_now < car_speed_last:
            result = 'speeddown'
        elif car_speed_now == car_speed_last:
            result = 'steady'

        car_speed_last = car_speed_now
        return result

class failed(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['speed_control_failed'])

    def execute(self, userdata):
        global car_speed_now
        global car_speed_last
        result = None
        global last_target
        global mode

        # m1_ta = reelRatio * min(50.0, min(21.23 * reelCof * msg.data + 12.3, 21.23 * 1.0 * msg.data + 21.23))
        # m2_ta = 0.5 * cbRatio * min(467.0, min(398.09 * cbCof * msg.data + 131.37, 398.09 * 1.0 * msg.data + 238.85))
        # m3_ta = pfRatio * min(187.0, min(39.16 * pfCof * msg.data + 52.47, 39.16 * 3.0 * msg.data + 90.07))
        # m4_ta = fhRatio * min(187.0, min(39.16 * fhCof * msg.data + 52.47, 39.16 * 3.0 * msg.data + 90.07))
        m1_ta = 0
        m2_ta = 0
        m3_ta = 0
        m4_ta = 0
        m5_ta = 0
        m6_ta = 0

        motor_goal[0].action_goal.goal.motor_id = m1
        motor_goal[1].action_goal.goal.motor_id = m2
        motor_goal[2].action_goal.goal.motor_id = m3
        motor_goal[3].action_goal.goal.motor_id = m4
        motor_goal[4].action_goal.goal.motor_id = m5
        motor_goal[5].action_goal.goal.motor_id = m6

        motor_goal[0].action_goal.goal.target_speed = m1_ta
        motor_goal[1].action_goal.goal.target_speed = m2_ta
        motor_goal[2].action_goal.goal.target_speed = m2_ta
        motor_goal[3].action_goal.goal.target_speed = m4_ta
        motor_goal[4].action_goal.goal.target_speed = m5_ta
        motor_goal[5].action_goal.goal.target_speed = m6_ta

        # for motor in motor_goal:
        #     print motor.action_goal.goal.motor_id, ' ', motor.action_goal.goal.target_speed

        # rospy.loginfo('Monitor car speed ...')
        result = 'speed_control_failed'

        car_speed_last = car_speed_now
        return result

def main():
    global motor_client

    rospy.init_node("preemption_example")

    topic_monitor = Topic_monitor()

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
        # smach.StateMachine.add('WAIT', smach_ros.MonitorState("/modified_car_speed", Float32, monitor_cb),
        #                        transitions={'invalid': 'MOTOR1', 'valid': 'WAIT', 'preempted': 'WAIT'})
        smach.StateMachine.add('WAIT', Car_speed_monitor(),
                               transitions={'speedup': 'MOTOR6',
                                            'speeddown': 'R_MOTOR1',
                                            'steady': 'WAIT'})
        smach.StateMachine.add('MOTOR1',
                               SimpleActionState('control485',
                                                 DriveMotorAction,
                                                 goal=motor_goal[0].action_goal.goal),
                               transitions={'succeeded': 'END',
                                            'preempted': 'MOTOR1',
                                            'aborted': 'MOTOR1'})

        smach.StateMachine.add('MOTOR2',
                               SimpleActionState('control485',
                                                 DriveMotorAction,
                                                 goal=motor_goal[1].action_goal.goal),
                               transitions={'succeeded': 'MOTOR1',
                                            'preempted': 'MOTOR2',
                                            'aborted': 'MOTOR2'})

        smach.StateMachine.add('MOTOR3',
                               SimpleActionState('control485',
                                                 DriveMotorAction,
                                                 goal=motor_goal[2].action_goal.goal),
                               transitions={'succeeded': 'MOTOR2',
                                            'preempted': 'MOTOR3',
                                            'aborted': 'MOTOR3'})
        smach.StateMachine.add('MOTOR4',
                               SimpleActionState('control485_2',
                                                 DriveMotorAction,
                                                 goal=motor_goal[3].action_goal.goal),
                               transitions={'succeeded': 'MOTOR3',
                                            'preempted': 'MOTOR4',
                                            'aborted': 'MOTOR4'})

        smach.StateMachine.add('MOTOR5',
                               SimpleActionState('control485_2',
                                                 DriveMotorAction,
                                                 goal=motor_goal[4].action_goal.goal),
                               transitions={'succeeded': 'MOTOR4',
                                            'preempted': 'MOTOR5',
                                            'aborted': 'MOTOR5'})

        smach.StateMachine.add('MOTOR6',
                               SimpleActionState('control485_2',
                                                 DriveMotorAction,
                                                 goal=motor_goal[5].action_goal.goal),
                               transitions={'succeeded': 'MOTOR5',
                                            'preempted': 'MOTOR6',
                                            'aborted': 'MOTOR6'})

        smach.StateMachine.add('R_MOTOR1',
                               SimpleActionState('control485',
                                                 DriveMotorAction,
                                                 goal=motor_goal[0].action_goal.goal),
                               transitions={'succeeded': 'R_MOTOR2',
                                            'preempted': 'R_MOTOR1',
                                            'aborted': 'R_MOTOR1'})

        smach.StateMachine.add('R_MOTOR2',
                               SimpleActionState('control485',
                                                 DriveMotorAction,
                                                 goal=motor_goal[1].action_goal.goal),
                               transitions={'succeeded': 'R_MOTOR3',
                                            'preempted': 'R_MOTOR2',
                                            'aborted': 'R_MOTOR2'})

        smach.StateMachine.add('R_MOTOR3',
                               SimpleActionState('control485',
                                                 DriveMotorAction,
                                                 goal=motor_goal[2].action_goal.goal),
                               transitions={'succeeded': 'R_MOTOR4',
                                            'preempted': 'R_MOTOR3',
                                            'aborted': 'R_MOTOR3'})
        smach.StateMachine.add('R_MOTOR4',
                               SimpleActionState('control485_2',
                                                 DriveMotorAction,
                                                 goal=motor_goal[3].action_goal.goal),
                               transitions={'succeeded': 'R_MOTOR5',
                                            'preempted': 'R_MOTOR4',
                                            'aborted': 'R_MOTOR4'})

        smach.StateMachine.add('R_MOTOR5',
                               SimpleActionState('control485_2',
                                                 DriveMotorAction,
                                                 goal=motor_goal[4].action_goal.goal),
                               transitions={'succeeded': 'R_MOTOR6',
                                            'preempted': 'R_MOTOR5',
                                            'aborted': 'R_MOTOR5'})

        smach.StateMachine.add('R_MOTOR6',
                               SimpleActionState('control485_2',
                                                 DriveMotorAction,
                                                 goal=motor_goal[5].action_goal.goal),
                               transitions={'succeeded': 'END',
                                            'preempted': 'R_MOTOR6',
                                            'aborted': 'R_MOTOR6'})

        smach.StateMachine.add('FAILED',
                               failed(), transitions={'speed_control_failed': 'R_MOTOR1'})

        smach.StateMachine.add('END',
                               end(), transitions={'end_succeeded': 'WAIT'})

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
