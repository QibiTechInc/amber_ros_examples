#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time

from std_msgs.msg import Empty as msgEmtpy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState
from amber_ros_driver.srv import (
    SetInt8Array,
    SetJointTrajectory,
    SetJointNo
)


class AmberImpedanceControl(object):

    _srv_setcontrolmode = '/set_control_mode'
    _srv_servoallon = '/servo_all_on'
    _srv_servoon = '/servo_on'
    _srv_setjointtrajectory = '/set_joint_trajectory'
    _srv_waitinterpolation = '/wait_interpolation'
    _srv_servoalloff = '/servo_all_off'
    _srv_servooff = '/servo_off'
    _srv_restposition = '/go_to_rest_position'

    def __init__(self):

        lr = self._LR = rospy.get_param('~amber_lr', default='amber_right')
        self._IMPD_P_GAIN = float(rospy.get_param('pgain', default=-2))
        self._IMPD_D_GAIN = float(rospy.get_param('dgain', default=-3))

        self._ENTER_KEY = False
        self._ACT_FLAG = False
        self._IMPD_ACTIVE = False
        self._INTP_MINJERK = 1
        self._NOMASK = [0, 0, 0, 0, 0, 0, 0]
        self._CURRENTS_DATA = {}
        self._POSITIONS_DATA = {}
        self._SERVICE_DICT = {}
        self._START_CURRENTS = {}
        self._START_POSITIONS = {}
        self._SUBSCRIBERS = []
        srv_dict = {}

        # set control mode
        srv_name = lr+self._srv_setcontrolmode
        rospy.wait_for_service(srv_name)
        srv_dict[self._srv_setcontrolmode] = rospy.ServiceProxy(
            srv_name,
            SetInt8Array
        )

        # servo all on
        srv_name = lr+self._srv_servoallon
        rospy.wait_for_service(srv_name)
        srv_dict[self._srv_servoallon] = rospy.ServiceProxy(
            srv_name,
            Empty
        )

        # servo on
        srv_name = lr+self._srv_servoon
        rospy.wait_for_service(srv_name)
        srv_dict[self._srv_servoon] = rospy.ServiceProxy(
            srv_name,
            SetJointNo
        )

        # set joint trajectory
        srv_name = lr+self._srv_setjointtrajectory
        rospy.wait_for_service(srv_name)
        srv_dict[self._srv_setjointtrajectory] = rospy.ServiceProxy(
            srv_name,
            SetJointTrajectory
        )

        # wait interpolation
        srv_name = lr+self._srv_waitinterpolation
        rospy.wait_for_service(srv_name)
        srv_dict[self._srv_waitinterpolation] = rospy.ServiceProxy(
            srv_name,
            Empty
        )

        # servo all off
        srv_name = lr+self._srv_servoalloff
        rospy.wait_for_service(srv_name)
        srv_dict[self._srv_servoalloff] = rospy.ServiceProxy(
            srv_name,
            Empty
        )

        # servo off
        srv_name = lr+self._srv_servooff
        rospy.wait_for_service(srv_name)
        srv_dict[self._srv_servooff] = rospy.ServiceProxy(
            srv_name,
            SetJointNo
        )

        # go to rest position
        srv_name = lr+self._srv_restposition
        rospy.wait_for_service(srv_name)
        srv_dict[self._srv_restposition] = rospy.ServiceProxy(
            srv_name,
            Empty
        )

        self._SERVICE_DICT[lr] = srv_dict

        self._SUBSCRIBERS.append(
            rospy.Subscriber(lr+'/joint_currents',
                             Float64MultiArray,
                             self._joint_currents_cb,
                             queue_size=1,
                             callback_args=lr)
        )
        self._SUBSCRIBERS.append(
            rospy.Subscriber(lr+'/joint_states',
                             JointState,
                             self._joint_states_cb,
                             queue_size=1,
                             callback_args=lr)
        )
        self._SUBSCRIBERS.append(
            rospy.Subscriber('scenario_key',
                             msgEmtpy,
                             self._scenario_key_cb,
                             queue_size=1)
        )


    def _joint_currents_cb(self, data, lr):
        self._CURRENTS_DATA[lr] = data.data


    def _joint_states_cb(self, data, lr):
        self._POSITIONS_DATA[lr] = data.position


    def _scenario_key_cb(self, data):
        self._ENTER_KEY = True


    def _wait_key_enter(self):
        while not rospy.is_shutdown() and not self._ENTER_KEY:
            time.sleep(1)
        self._ENTER_KEY = False


    def _get_current_average(self, duration):
        ret_data = {}
        for lr in self._CURRENTS_DATA.keys():
            currents_data = self._CURRENTS_DATA[lr]
            sum_list = [0]*len(currents_data)
            count = 0
            r = rospy.Rate(50)
            start = rospy.Time.now()
            while rospy.Time.now()-start < rospy.Duration(duration):
                sum_list = [i+j for i,j in zip(sum_list, currents_data)]
                count+=1
                r.sleep()
            ret_data[lr] = [i/count for i in sum_list]
        return ret_data


    def _close_hand_until_contact(self,
                                  lr,
                                  close_angle=0.5,
                                  close_time=3.0,
                                  close_offset=0.3,
                                  contact_current=0.5):
        start_currents = self._get_current_average(1.0)
        self._SERVICE_DICT[lr][self._srv_setjointtrajectory](
            [0, 0, 0, 0, 0, close_angle, close_angle],
            close_time,
            [1, 1, 1, 1, 1, 0, 0],
            self._INTP_MINJERK,
            False
        )
        diff_c = [i-j for i,j in zip(self._CURRENTS_DATA[lr], start_currents[lr])]
        # diff_c = self._CURRENTS_DATA[lr]
        while (abs(diff_c[5]) < contact_current \
               or abs(diff_c[6]) < contact_current) \
               and not rospy.is_shutdown():
            diff_c = self._CURRENTS_DATA[lr]
            print(diff_c[5:])
            time.sleep(0.01)
        self._SERVICE_DICT[lr][self._srv_setjointtrajectory](
            [0, 0, 0, 0, 0, 0, 0],
            0.01,
            [1, 1, 1, 1, 1, 0, 0],
            self._INTP_MINJERK,
            True
        )
        self._SERVICE_DICT[lr][self._srv_waitinterpolation]()
        time.sleep(1)
        rospy.loginfo('grasp offset')
        self._SERVICE_DICT[lr][self._srv_setjointtrajectory](
            [0, 0, 0, 0, 0, close_offset, close_offset],
            1.0,
            [1, 1, 1, 1, 1, 0, 0],
            self._INTP_MINJERK,
            True
        )
        self._SERVICE_DICT[lr][self._srv_waitinterpolation]()


    def close_hand_test(self):
        rospy.loginfo("- control mode: POSITION mode")
        self._SERVICE_DICT[self._LR][self._srv_setcontrolmode]([1, 1, 1, 1, 1, 1, 1])
        time.sleep(0.2)

        # servo all on
        rospy.loginfo("- servo ALL ON")
        self._SERVICE_DICT[self._LR][self._srv_servoallon]()
        time.sleep(0.5)

        # set joint trajectory by position mode
        rospy.loginfo("- joint trajectory")
        self._SERVICE_DICT[self._LR][self._srv_setjointtrajectory](
            [0.0, -0.055, -1.57, 0.0, 1.57, -0.5, -0.5],
            3.0,
            self._NOMASK,
            self._INTP_MINJERK,
            False
        )

        # wait interpolation
        rospy.loginfo("- wait interpolation...")
        self._SERVICE_DICT[self._LR][self._srv_waitinterpolation]()
        rospy.loginfo("- reached")

        # set plate
        rospy.loginfo("- ")
        rospy.loginfo('After setting plate on hand, please press the "Enter" key')
        self._wait_key_enter()

        # grasp
        self._close_hand_until_contact(self._LR)


    def _impedance_control(self, event):
        lr = self._LR
        if self._IMPD_ACTIVE:
            pos_diff = [i-j for i,j in zip(self._POSITIONS_DATA[lr], self._START_POSITIONS[lr])]
            pos_dd = [i-j for i,j in zip(pos_diff, self._PRE_POS_DIFF[lr])]
            target_currents = [0]*len(self._START_CURRENTS[lr])
            target_currents[5] = self._grasp_angle
            target_currents[6] = self._grasp_angle
            for i in range(3):
                target_currents[i] = self._START_CURRENTS[lr][i] \
                    + pos_diff[i]*self._IMPD_P_GAIN + pos_dd[i]*self._IMPD_D_GAIN
            rospy.loginfo(target_currents)
            self._SERVICE_DICT[lr][self._srv_setjointtrajectory](
                target_currents,
                0.01,
                [0, 0, 0, 1, 1, 1, 1],
                self._INTP_MINJERK,
                False
            )
            self._PRE_POS_DIFF[lr] = pos_diff


    def test(self):
        rospy.loginfo("sceen1")
        self._wait_key_enter()
        rospy.loginfo("sceen2")
        self._wait_key_enter()
        rospy.loginfo("sceen3")
        self._wait_key_enter()
        rospy.loginfo("sceen4")
        self._wait_key_enter()


    def run(self):
        # set control mode to position
        rospy.loginfo("- control mode: POSITION mode")
        self._SERVICE_DICT[self._LR][self._srv_setcontrolmode]([1, 1, 1, 1, 1, 1, 1])
        time.sleep(0.2)

        # servo all on
        rospy.loginfo("- servo ALL ON")
        self._SERVICE_DICT[self._LR][self._srv_servoallon]()
        # self._SERVICE_DICT[self._LR][self._srv_servoon](0)
        time.sleep(0.5)

        # set joint trajectory by position mode
        rospy.loginfo("- joint trajectory")
        self._SERVICE_DICT[self._LR][self._srv_setjointtrajectory](
            [0.0, -0.055, -1.57, 0.0, 1.57, -0.5, -0.5],
            3.0,
            self._NOMASK,
            self._INTP_MINJERK,
            False
        )

        # wait interpolation
        rospy.loginfo("- wait interpolation...")
        self._SERVICE_DICT[self._LR][self._srv_waitinterpolation]()
        rospy.loginfo("- reached")

        # set plate
        rospy.loginfo("- ")
        rospy.loginfo('After setting plate on hand, please press the "Enter" key')
        self._wait_key_enter()

        # set joint trajectory by position mode
        rospy.loginfo("- graspping...")
        # self._close_hand_until_contact(self._LR)
        self._grasp_angle = 0.3
        self._SERVICE_DICT[self._LR][self._srv_setjointtrajectory](
            [0.0, -0.055, -1.57, 0.0, 1.57, self._grasp_angle, self._grasp_angle],
            3.0,
            self._NOMASK,
            self._INTP_MINJERK,
            False
        )
        rospy.loginfo("- wait interpolation...")
        self._SERVICE_DICT[self._LR][self._srv_waitinterpolation]()
        rospy.loginfo("- grasp")

        # get now sensor data
        rospy.loginfo("- get data...")
        time.sleep(1)
        self._START_CURRENTS = self._get_current_average(2)
        print(self._START_CURRENTS[self._LR])
        self._START_POSITIONS = self._POSITIONS_DATA.copy()
        self._PRE_POS_DIFF = self._START_POSITIONS.copy()
        for k,v in self._PRE_POS_DIFF.items():
            self._PRE_POS_DIFF[k] = [0]*len(v)
        rospy.loginfo("- ok")

        # wait input enter key
        rospy.loginfo("- ")
        rospy.loginfo("After grabbing the plate, please press the enter key")
        self._wait_key_enter()

        # change control mode to current, then set joint trajectory
        rospy.loginfo("- change control mode to CURRENT mode from joint 0 to 2")
        self._START_CURRENTS[5] = self._grasp_angle
        self._START_CURRENTS[6] = self._grasp_angle
        self._SERVICE_DICT[self._LR][self._srv_setcontrolmode]([3, 3, 3, 1, 1, 1, 1])
        self._SERVICE_DICT[self._LR][self._srv_setjointtrajectory](
            self._START_CURRENTS[self._LR],
            0.1,
            [0, 0, 0, 1, 1, 1, 1],
            self._INTP_MINJERK,
            False
        )
        self._SERVICE_DICT[self._LR][self._srv_waitinterpolation]()
        rospy.loginfo("- ok")

        # start impedance control
        self._IMPD_ACTIVE = True
        impd_timers = []
        impd_timers.append(rospy.Timer(rospy.Duration(0.1), self._impedance_control))
        rospy.loginfo("- start impedance control")

        # wait input enter key
        rospy.loginfo("- ")
        rospy.loginfo("finish by pressing the enter key")
        self._wait_key_enter()
        self._IMPD_ACTIVE = False
        for it in impd_timers:
            it.shutdown()
        rospy.loginfo("- timer end")

        # change control mode to position, then set joint trajectory
        rospy.loginfo("- change control mode to POSITION mode from joint 0 to 2")
        self._SERVICE_DICT[self._LR][self._srv_servooff](2)
        time.sleep(0.1)
        self._SERVICE_DICT[self._LR][self._srv_setcontrolmode]([1, 1, 1, 1, 1, 1, 1])
        time.sleep(0.1)
        self._SERVICE_DICT[self._LR][self._srv_servoon](2)
        time.sleep(0.1)
        rospy.loginfo("- ok")

        # rest position
        rospy.loginfo("- move rest position")
        self._SERVICE_DICT[self._LR][self._srv_restposition]()

        # servo all off
        rospy.loginfo("- servo ALL OFF")
        self._SERVICE_DICT[self._LR][self._srv_servoalloff]()



class AmberImpedanceControlDemoScenario(object):
    def __init__(self):
        self._pub = rospy.Publisher('scenario_key', msgEmtpy, queue_size=1)

    def _wait_key_enter(self):
        input('')

    def run(self):
        while not rospy.is_shutdown():
            self._wait_key_enter()
            self._pub.publish()
