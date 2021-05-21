#!/usr/bin/env python
# coding=utf-8

import rospy
import smach
import smach_ros
import threading

from std_msgs.msg import Empty
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Int16

from smach_ros import SimpleActionState
from control485.msg import DriveMotorAction
import actionlib

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
m1 = 3
m2 = 4
m3 = 2
m4 = 1
m5 = 5
m6 = 7
m7 = 8
m8 = 11
m9 = 9
m10 = 10

# 模式
mode = 'stop'

# 设定电机序号
# motors = [cb, reel]
# motors = [pf, cb, reel]
motors = [m1, m2, m3, m4, m5, m6, m7, m8, m9, m10]
motor_target_speed = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
# motors = [reel, cb]

client_1 = actionlib.SimpleActionClient('control485', DriveMotorAction)
client_2 = actionlib.SimpleActionClient('control485_2', DriveMotorAction)
client_3 = actionlib.SimpleActionClient('control485_3', DriveMotorAction)
motor_now = 0

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
is_stop = 0
is_stop_last = 0
motor_controlled_id = 0


class Topic_monitor:
    def __init__(self):
        self.car_speed = 0

        rospy.Subscriber('/modified_car_speed', Float32, self.callback_car_speed)
        rospy.Subscriber('/stop', Int16, self.callback_stop_msg)
        # rospy.Subscriber('/motor_controlled', Int16, self.callback_motor_id)

        self.callback_thread = threading.Thread(target=self.call_back_jobs)
        self.callback_thread.start()

    ## callback functions ##
    def callback_car_speed(self, data):
        global car_speed_now
        self.car_speed = data.data
        car_speed_now = data.data

    def callback_stop_msg(self, data):
        global is_stop
        is_stop = data.data

    # def callback_motor_id(self, data):
    #     global motor_controlled_id
    #     motor_controlled_id = data.data

    ## thread functions ##
    def call_back_jobs(self):
        rospy.spin()

class client_motor_1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCEEDED', 'ABORTED'])

    def execute(self, userdata):
        global client_1
        global motor_goal
        # msg = Int32()
        # msg.data = 1
        # pub_result.publish(msg)
        goal = motor_goal[0].action_goal.goal
        action_result = client_1.send_goal_and_wait(goal)
        # client_1.wait_for_result()
        # action_result = client_1
        # print 'motor10 goal state:'
        # print action_result
        if action_result == 3:
            action_result = "SUCCEEDED"
        elif action_result == 4:
            action_result = 'ABORTED'

        if action_result == 'ABORTED':
            for index in range(len(motor_goal)):
                motor_goal[index].action_goal.goal.target_speed = 0
        return action_result

class client_motor_2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCEEDED', 'ABORTED'])

    def execute(self, userdata):
        global client_1
        global motor_goal
        # msg = Int32()
        # msg.data = 1
        # pub_result.publish(msg)
        goal = motor_goal[1].action_goal.goal
        action_result = client_1.send_goal_and_wait(goal)
        # client_1.wait_for_result()
        # action_result = client_1
        # print 'motor10 goal state:'
        # print action_result
        if action_result == 3:
            action_result = "SUCCEEDED"
        elif action_result == 4:
            action_result = 'ABORTED'

        if action_result == 'ABORTED':
            for index in range(len(motor_goal)):
                motor_goal[index].action_goal.goal.target_speed = 0
        return action_result

class client_motor_3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCEEDED', 'ABORTED'])

    def execute(self, userdata):
        global client_1
        global motor_goal
        # msg = Int32()
        # msg.data = 1
        # pub_result.publish(msg)
        goal = motor_goal[2].action_goal.goal
        action_result = client_1.send_goal_and_wait(goal)
        # client_1.wait_for_result()
        # action_result = client_1
        # print 'motor10 goal state:'
        # print action_result
        if action_result == 3:
            action_result = "SUCCEEDED"
        elif action_result == 4:
            action_result = 'ABORTED'

        if action_result == 'ABORTED':
            for index in range(len(motor_goal)):
                motor_goal[index].action_goal.goal.target_speed = 0
        return action_result

class client_motor_4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCEEDED', 'ABORTED'])

    def execute(self, userdata):
        global client_1
        global motor_goal
        # msg = Int32()
        # msg.data = 1
        # pub_result.publish(msg)
        goal = motor_goal[3].action_goal.goal
        action_result = client_1.send_goal_and_wait(goal)
        # client_1.wait_for_result()
        # action_result = client_1
        # print 'motor10 goal state:'
        # print action_result
        if action_result == 3:
            action_result = "SUCCEEDED"
        elif action_result == 4:
            action_result = 'ABORTED'

        if action_result == 'ABORTED':
            for index in range(len(motor_goal)):
                motor_goal[index].action_goal.goal.target_speed = 0
        return action_result

class client_motor_5(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCEEDED', 'ABORTED'])

    def execute(self, userdata):
        global client_2
        global motor_goal
        # msg = Int32()
        # msg.data = 1
        # pub_result.publish(msg)
        goal = motor_goal[4].action_goal.goal
        action_result = client_2.send_goal_and_wait(goal)
        # client_2.wait_for_result()
        # action_result = client_2
        # print 'motor10 goal state:'
        # print action_result
        if action_result == 3:
            action_result = "SUCCEEDED"
        elif action_result == 4:
            action_result = 'ABORTED'

        if action_result == 'ABORTED':
            for index in range(len(motor_goal)):
                motor_goal[index].action_goal.goal.target_speed = 0
        return action_result

class client_motor_6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCEEDED', 'ABORTED'])

    def execute(self, userdata):
        global client_2
        global motor_goal
        # msg = Int32()
        # msg.data = 1
        # pub_result.publish(msg)
        goal = motor_goal[5].action_goal.goal
        action_result = client_2.send_goal_and_wait(goal)
        # client_2.wait_for_result()
        # action_result = client_2
        # print 'motor10 goal state:'
        # print action_result
        if action_result == 3:
            action_result = "SUCCEEDED"
        elif action_result == 4:
            action_result = 'ABORTED'

        if action_result == 'ABORTED':
            for index in range(len(motor_goal)):
                motor_goal[index].action_goal.goal.target_speed = 0
        return action_result

class client_motor_7(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCEEDED', 'ABORTED'])

    def execute(self, userdata):
        global client_2
        global motor_goal
        # msg = Int32()
        # msg.data = 1
        # pub_result.publish(msg)
        goal = motor_goal[6].action_goal.goal
        action_result = client_2.send_goal_and_wait(goal)
        # client_2.wait_for_result()
        # action_result = client_2
        # print 'motor10 goal state:'
        # print action_result
        if action_result == 3:
            action_result = "SUCCEEDED"
        elif action_result == 4:
            action_result = 'ABORTED'

        if action_result == 'ABORTED':
            for index in range(len(motor_goal)):
                motor_goal[index].action_goal.goal.target_speed = 0
        return action_result

class client_motor_8(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCEEDED', 'ABORTED'])

    def execute(self, userdata):
        global client_3
        global motor_goal
        # msg = Int32()
        # msg.data = 1
        # pub_result.publish(msg)
        goal = motor_goal[7].action_goal.goal
        action_result = client_3.send_goal_and_wait(goal)
        # client_3.wait_for_result()
        # action_result = client_3
        # print 'motor10 goal state:'
        # print action_result
        if action_result == 3:
            action_result = "SUCCEEDED"
        elif action_result == 4:
            action_result = 'ABORTED'

        if action_result == 'ABORTED':
            for index in range(len(motor_goal)):
                motor_goal[index].action_goal.goal.target_speed = 0
        return action_result

class client_motor_9(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCEEDED', 'ABORTED'])

    def execute(self, userdata):
        global client_3
        global motor_goal
        # msg = Int32()
        # msg.data = 1
        # pub_result.publish(msg)
        goal = motor_goal[8].action_goal.goal
        action_result = client_3.send_goal_and_wait(goal)
        # client_3.wait_for_result()
        # action_result = client_3
        # print 'motor10 goal state:'
        # print action_result
        if action_result == 3:
            action_result = "SUCCEEDED"
        elif action_result == 4:
            action_result = 'ABORTED'

        if action_result == 'ABORTED':
            for index in range(len(motor_goal)):
                motor_goal[index].action_goal.goal.target_speed = 0
        return action_result

class client_motor_10(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['SUCCEEDED', 'ABORTED'])

    def execute(self, userdata):
        global client_3
        global motor_goal
        # msg = Int32()
        # msg.data = 1
        # pub_result.publish(msg)
        goal = motor_goal[9].action_goal.goal
        action_result = client_3.send_goal_and_wait(goal)
        # client_3.wait_for_result()
        # action_result = client_3
        # print 'motor10 goal state:'
        # print action_result
        if action_result == 3:
            action_result = "SUCCEEDED"
        elif action_result == 4:
            action_result = 'ABORTED'

        if action_result == 'ABORTED':
            for index in range(len(motor_goal)):
                motor_goal[index].action_goal.goal.target_speed = 0
        return action_result


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
        smach.State.__init__(self, outcomes=['speedup', 'speeddown', 'steady', 'stop', 'start'])
        self.counter = 0

    def execute(self, userdata):
        global car_speed_now
        global car_speed_last
        global is_stop
        global is_stop_last
        result = None
        global last_target
        global mode

        global motors
        global motor_target_speed
        global motor_goal

        '''
        motor target speed describtion
        '''
        motor_speed_dict = {
            # todo 增加1-4的减速比（3 4 2 1）
            'M1': 44 / 1,
            'M2': 467 / 1,
            'M3': 187 / 1,
            'M4': 324 / 1,
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

        # motor_target_speed[0] = 1000  # motor_speed_dict['M3']
        # motor_target_speed[1] = motor_speed_dict['M3']
        # motor_target_speed[2] = motor_speed_dict['M2']
        # motor_target_speed[3] = motor_speed_dict['M1']
        # motor_target_speed[4] = motor_speed_dict['M5']
        # motor_target_speed[5] = motor_speed_dict['M7']
        # motor_target_speed[6] = motor_speed_dict['M8']
        # motor_target_speed[7] = motor_speed_dict['M11']
        # motor_target_speed[8] = motor_speed_dict['M9']
        # motor_target_speed[9] = motor_speed_dict['M10']

        motor_target_speed[0] = 1500 + 1000 * car_speed_now  # motor_speed_dict['M3']
        motor_target_speed[1] = 500
        motor_target_speed[2] = 500
        motor_target_speed[3] = 800
        motor_target_speed[4] = 500
        motor_target_speed[5] = 300
        motor_target_speed[6] = 500
        motor_target_speed[7] = 200
        motor_target_speed[8] = 500
        motor_target_speed[9] = 500

        for index in range(len(motor_target_speed)):
            if motor_target_speed[index] > 3000:
                motor_target_speed[index] = 3000

        for index in range(len(motor_goal)):
            motor_goal[index].action_goal.goal.motor_id = motors[index]
            motor_goal[index].action_goal.goal.target_speed = motor_target_speed[index]

        if is_stop == 1:
            for index in range(len(motor_goal)):
                motor_goal[index].action_goal.goal.target_speed = 0
        # for motor in motor_goal:
        #     print motor.action_goal.goal.motor_id, ' ', motor.action_goal.goal.target_speed

        # rospy.loginfo('Monitor car speed ...')
        if is_stop == 1 and is_stop_last == 0:
            is_stop_last = is_stop
            result = 'stop'
        elif is_stop == 0 and is_stop_last == 1:
            is_stop_last = is_stop
            result = 'start'
        elif car_speed_now > car_speed_last and car_speed_last == 0:
            result = 'start'
        elif car_speed_now > car_speed_last:
            result = 'speedup'
        elif car_speed_now < car_speed_last and car_speed_last != 0:
            result = 'speeddown'
        elif car_speed_now < car_speed_last and car_speed_last == 0:
            result = 'stop'
        elif car_speed_now == car_speed_last:
            result = 'steady'

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
                               transitions={'start': 'START_MOTOR10',
                                            'speeddown': 'SPEEDCHANGE_MOTOR1',
                                            'steady': 'WAIT',
                                            'stop': 'SPEEDDOWN_MOTOR1',
                                            'speedup': 'SPEEDCHANGE_MOTOR1'})

        smach.StateMachine.add('SPEEDCHANGE_MOTOR1', client_motor_1(),
                               transitions={'SUCCEEDED': 'END',
                                            'ABORTED': 'SPEEDDOWN_MOTOR1'})

        smach.StateMachine.add('START_MOTOR1', client_motor_1(),
                               transitions={'SUCCEEDED': 'END',
                                            'ABORTED': 'SPEEDDOWN_MOTOR1'})

        smach.StateMachine.add('START_MOTOR2', client_motor_2(),
                               transitions={'SUCCEEDED': 'START_MOTOR1',
                                            'ABORTED': 'SPEEDDOWN_MOTOR2'})

        smach.StateMachine.add('START_MOTOR3', client_motor_3(),
                               transitions={'SUCCEEDED': 'START_MOTOR2',
                                            'ABORTED': 'SPEEDDOWN_MOTOR3'})

        smach.StateMachine.add('START_MOTOR4', client_motor_4(),
                               transitions={'SUCCEEDED': 'START_MOTOR3',
                                            'ABORTED': 'SPEEDDOWN_MOTOR4'})

        smach.StateMachine.add('START_MOTOR5', client_motor_5(),
                               transitions={'SUCCEEDED': 'START_MOTOR4',
                                            'ABORTED': 'SPEEDDOWN_MOTOR5'})

        smach.StateMachine.add('START_MOTOR6', client_motor_6(),
                               transitions={'SUCCEEDED': 'START_MOTOR5',
                                            'ABORTED': 'SPEEDDOWN_MOTOR6'})

        smach.StateMachine.add('START_MOTOR7', client_motor_7(),
                               transitions={'SUCCEEDED': 'START_MOTOR6',
                                            'ABORTED': 'SPEEDDOWN_MOTOR7'})

        smach.StateMachine.add('START_MOTOR8', client_motor_8(),
                               transitions={'SUCCEEDED': 'START_MOTOR7',
                                            'ABORTED': 'SPEEDDOWN_MOTOR8'})

        smach.StateMachine.add('START_MOTOR9', client_motor_9(),
                               transitions={'SUCCEEDED': 'START_MOTOR8',
                                            'ABORTED': 'SPEEDDOWN_MOTOR9'})

        smach.StateMachine.add('START_MOTOR10', client_motor_10(),
                               transitions={'SUCCEEDED': 'START_MOTOR9',
                                            'ABORTED': 'SPEEDDOWN_MOTOR10'})

        smach.StateMachine.add('SPEEDDOWN_MOTOR1', client_motor_1(),
                               transitions={'SUCCEEDED': 'SPEEDDOWN_MOTOR2',
                                            'ABORTED': 'SPEEDDOWN_MOTOR1'})

        smach.StateMachine.add('SPEEDDOWN_MOTOR2', client_motor_2(),
                               transitions={'SUCCEEDED': 'SPEEDDOWN_MOTOR3',
                                            'ABORTED': 'SPEEDDOWN_MOTOR2'})

        smach.StateMachine.add('SPEEDDOWN_MOTOR3', client_motor_3(),
                               transitions={'SUCCEEDED': 'SPEEDDOWN_MOTOR4',
                                            'ABORTED': 'SPEEDDOWN_MOTOR3'})

        smach.StateMachine.add('SPEEDDOWN_MOTOR4', client_motor_4(),
                               transitions={'SUCCEEDED': 'SPEEDDOWN_MOTOR5',
                                            'ABORTED': 'SPEEDDOWN_MOTOR4'})

        smach.StateMachine.add('SPEEDDOWN_MOTOR5', client_motor_5(),
                               transitions={'SUCCEEDED': 'SPEEDDOWN_MOTOR6',
                                            'ABORTED': 'SPEEDDOWN_MOTOR5'})

        smach.StateMachine.add('SPEEDDOWN_MOTOR6', client_motor_6(),
                               transitions={'SUCCEEDED': 'SPEEDDOWN_MOTOR7',
                                            'ABORTED': 'SPEEDDOWN_MOTOR6'})

        smach.StateMachine.add('SPEEDDOWN_MOTOR7', client_motor_7(),
                               transitions={'SUCCEEDED': 'SPEEDDOWN_MOTOR8',
                                            'ABORTED': 'SPEEDDOWN_MOTOR7'})

        smach.StateMachine.add('SPEEDDOWN_MOTOR8', client_motor_8(),
                               transitions={'SUCCEEDED': 'SPEEDDOWN_MOTOR9',
                                            'ABORTED': 'SPEEDDOWN_MOTOR8'})

        smach.StateMachine.add('SPEEDDOWN_MOTOR9', client_motor_9(),
                               transitions={'SUCCEEDED': 'SPEEDDOWN_MOTOR10',
                                            'ABORTED': 'SPEEDDOWN_MOTOR9'})

        smach.StateMachine.add('SPEEDDOWN_MOTOR10', client_motor_10(),
                               transitions={'SUCCEEDED': 'END',
                                            'ABORTED': 'SPEEDDOWN_MOTOR10'})

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
