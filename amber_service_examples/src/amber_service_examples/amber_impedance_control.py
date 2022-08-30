#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import sys

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

    _SRV_SETCONTROLMODE = '/set_control_mode'
    _SRV_SERVOALLON = '/servo_all_on'
    _SRV_SERVOON = '/servo_on'
    _SRV_SETJOINTTRAJECTORY = '/set_joint_trajectory'
    _SRV_WAITINTERPOLATION = '/wait_interpolation'
    _SRV_SERVOALLOFF = '/servo_all_off'
    _SRV_SERVOOFF = '/servo_off'
    _SRV_RESETPOSITION = '/go_to_rest_position'

    def __init__(self):

        self._LR = rospy.get_param('~amber_lr', default='amber_right')
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

        # register services
        service_list = [self._SRV_SETCONTROLMODE,
                        self._SRV_SERVOALLON,
                        self._SRV_SERVOON,
                        self._SRV_SETJOINTTRAJECTORY,
                        self._SRV_WAITINTERPOLATION,
                        self._SRV_SERVOALLOFF,
                        self._SRV_SERVOOFF,
                        self._SRV_RESETPOSITION]
        srv_dict = {}
        for sv in service_list:
            srv_dict = self._register_service(self, sv, srv_dict)
        self._SERVICE_DICT[self._LR] = srv_dict

        self._SUBSCRIBERS.append(
            rospy.Subscriber(self._LR+'/joint_currents',
                             Float64MultiArray,
                             self._joint_currents_cb,
                             queue_size=1,
                             callback_args=self._LR)
        )
        self._SUBSCRIBERS.append(
            rospy.Subscriber(self._LR+'/joint_states',
                             JointState,
                             self._joint_states_cb,
                             queue_size=1,
                             callback_args=self._LR)
        )
        self._SUBSCRIBERS.append(
            rospy.Subscriber('scenario_key',
                             msgEmtpy,
                             self._scenario_key_cb,
                             queue_size=1)
        )


    def _register_service(self, name, srv_dict):
        srv_name = self._LR + name
        rospy.wait_for_service(srv_name)
        srv_dict[name] = rospy.ServiceProxy(
            srv_name,
            Empty
        )
        return srv_dict


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
                sum_list = [i+j for i, j in zip(sum_list, currents_data)]
                count += 1
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
        self._SERVICE_DICT[lr][self._SRV_SETJOINTTRAJECTORY](
            [0, 0, 0, 0, 0, close_angle, close_angle],
            close_time,
            [1, 1, 1, 1, 1, 0, 0],
            self._INTP_MINJERK,
            False
        )
        diff_c = [i-j for i, j in zip(self._CURRENTS_DATA[lr], start_currents[lr])]
        while (abs(diff_c[5]) < contact_current \
               or abs(diff_c[6]) < contact_current) \
               and not rospy.is_shutdown():
            diff_c = [i-j for i, j in zip(self._CURRENTS_DATA[lr], start_currents[lr])]
            time.sleep(0.01)
        self._SERVICE_DICT[lr][self._SRV_SETJOINTTRAJECTORY](
            [0, 0, 0, 0, 0, 0, 0],
            0.01,
            [1, 1, 1, 1, 1, 0, 0],
            self._INTP_MINJERK,
            True
        )
        self._SERVICE_DICT[lr][self._SRV_WAITINTERPOLATION]()
        time.sleep(1)
        rospy.loginfo('-- grasp offset')
        self._SERVICE_DICT[lr][self._SRV_SETJOINTTRAJECTORY](
            [0, 0, 0, 0, 0, close_offset, close_offset],
            1.0,
            [1, 1, 1, 1, 1, 0, 0],
            self._INTP_MINJERK,
            True
        )
        self._SERVICE_DICT[lr][self._SRV_WAITINTERPOLATION]()


    def _impedance_control(self, event):
        if self._IMPD_ACTIVE:
            pos_diff = [i-j for i, j in zip(self._POSITIONS_DATA[self._LR],
                                            self._START_POSITIONS[self._LR])]
            pos_dd = [i-j for i, j in zip(pos_diff, self._PRE_POS_DIFF[self._LR])]
            target_currents = [0]*len(self._START_CURRENTS[self._LR])
            for i in range(3):
                target_currents[i] = self._START_CURRENTS[self._LR][i] \
                    + pos_diff[i]*self._IMPD_P_GAIN + pos_dd[i]*self._IMPD_D_GAIN
            rospy.logdebug(target_currents)
            self._SERVICE_DICT[self._LR][self._SRV_SETJOINTTRAJECTORY](
                target_currents,
                0.01,
                [0, 0, 0, 1, 1, 1, 1],
                self._INTP_MINJERK,
                False
            )
            self._PRE_POS_DIFF[self._LR] = pos_diff


    def run(self):
        # set control mode to position
        rospy.loginfo("- all control mode: POSITION mode")
        self._SERVICE_DICT[self._LR][self._SRV_SETCONTROLMODE]([1, 1, 1, 1, 1, 1, 1])
        time.sleep(1)
        rospy.loginfo("ok")

        # servo all on
        rospy.loginfo("- servo ALL ON")
        self._SERVICE_DICT[self._LR][self._SRV_SERVOALLON]()
        time.sleep(1)
        rospy.loginfo("ok")

        # set joint trajectory by position mode
        rospy.loginfo("- set joint trajectory to start pose")
        self._SERVICE_DICT[self._LR][self._SRV_SETJOINTTRAJECTORY](
            [0.0, -0.055, -1.57, 0.0, 1.57, -0.5, -0.5],
            3.0,
            self._NOMASK,
            self._INTP_MINJERK,
            False
        )

        # wait interpolation
        rospy.loginfo("- wait interpolation...")
        self._SERVICE_DICT[self._LR][self._SRV_WAITINTERPOLATION]()
        rospy.loginfo("ok")

        # wait for plate to be set
        rospy.loginfo('***')
        rospy.loginfo('After setting plate on hand, please press the "Enter" key')
        self._wait_key_enter()

        # grasp plate
        rospy.loginfo("- graspping...")
        self._close_hand_until_contact(self._LR)
        rospy.loginfo("grasp")

        # get now sensor data
        rospy.loginfo("- get data...")
        time.sleep(1)
        self._START_CURRENTS = self._get_current_average(2)
        self._START_POSITIONS = self._POSITIONS_DATA.copy()
        self._PRE_POS_DIFF = self._START_POSITIONS.copy()
        for k, v in self._PRE_POS_DIFF.items():
            self._PRE_POS_DIFF[k] = [0]*len(v)
        rospy.loginfo("ok")

        # change upper arm control mode to current
        rospy.loginfo("- upper arm control mode: CURRENT mode")
        self._SERVICE_DICT[self._LR][self._SRV_SETCONTROLMODE]([3, 3, 3, 1, 1, 1, 1])
        self._SERVICE_DICT[self._LR][self._SRV_SETJOINTTRAJECTORY](
            self._START_CURRENTS[self._LR],
            0.1,
            [0, 0, 0, 1, 1, 1, 1],
            self._INTP_MINJERK,
            False
        )
        self._SERVICE_DICT[self._LR][self._SRV_WAITINTERPOLATION]()
        rospy.loginfo("ok")

        # start impedance control
        self._IMPD_ACTIVE = True
        impd_timers = []
        impd_timers.append(rospy.Timer(rospy.Duration(0.1), self._impedance_control))
        rospy.loginfo("- <<< Start impedance control >>>")

        # wait input enter key
        rospy.loginfo('***')
        rospy.loginfo("Finish by pressing the enter key")
        self._wait_key_enter()
        self._IMPD_ACTIVE = False
        for it in impd_timers:
            it.shutdown()
        rospy.loginfo("- timer end")

        # change upper arm control mode to position
        rospy.loginfo("- control mode: POSITION mode")
        self._SERVICE_DICT[self._LR][self._SRV_SETCONTROLMODE]([1, 1, 1, 1, 1, 1, 1])
        time.sleep(1)
        rospy.loginfo("ok")

        # release plate
        rospy.loginfo("- open hand")
        self._SERVICE_DICT[self._LR][self._SRV_SETJOINTTRAJECTORY](
            [0, 0, 0, 0, 0, -0.5, -0.5],
            3.0,
            [1, 1, 1, 1, 1, 0, 0],
            self._INTP_MINJERK,
            False
        )
        self._SERVICE_DICT[self._LR][self._SRV_WAITINTERPOLATION]()
        rospy.loginfo("ok")

        # rest position
        rospy.loginfo("- move rest position")
        self._SERVICE_DICT[self._LR][self._SRV_RESETPOSITION]()
        rospy.loginfo("ok")

        # servo all off
        rospy.loginfo("- servo ALL OFF")
        self._SERVICE_DICT[self._LR][self._SRV_SERVOALLOFF]()
        rospy.loginfo("ok")

        rospy.loginfo("*** Demo finish ***")
        rospy.loginfo("Please 'Ctrl-c' and press 'Enter'")



class AmberImpedanceControlDemoScenario(object):
    def __init__(self):
        self._pub = rospy.Publisher('scenario_key', msgEmtpy, queue_size=1)


    def _wait_key_enter(self):
        python_ver = sys.version_info.major
        if python_ver == 2:
            raw_input('')
        else:
            input('')


    def run(self):
        try:
            while not rospy.is_shutdown():
                self._wait_key_enter()
                self._pub.publish()
        finally:
            rospy.loginfo("<< Please press 'Enter' to exit >>")
