#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import sys
from std_srvs.srv import Empty
from amber_ros_driver.srv import (
    SetInt8Array,
    SetJointTrajectory
)


def wait_key_enter():
    python_ver = sys.version_info.major
    if python_ver == 2:
        raw_input('')
    else:
        input('')


class ChangeModePos2CurExampleNode(object):
    def __init__(self):
        self._INTP_MINJERK = 1
        self._NOMASK = [0, 0, 0, 0, 0, 0, 0]
        self._SRV_R_SETCONTROLMODE = '/amber_right/set_control_mode'
        self._SRV_R_SERVOALLON = '/amber_right/servo_all_on'
        self._SRV_R_SETJOINTTRAJECTORY = '/amber_right/set_joint_trajectory'
        self._SRV_R_WAITINTERPOLATION = '/amber_right/wait_interpolation'
        self._SRV_R_SERVOALLOFF = '/amber_right/servo_all_off'

        rospy.init_node('mode_change_pos2cur_example_node')

        # set control mode
        rospy.wait_for_service(self._SRV_R_SETCONTROLMODE)
        self._set_control_mode = rospy.ServiceProxy(
            self._SRV_R_SETCONTROLMODE,
            SetInt8Array
        )
        # servo all on
        rospy.wait_for_service(self._SRV_R_SERVOALLON)
        self._servo_allon = rospy.ServiceProxy(
            self._SRV_R_SERVOALLON,
            Empty
        )
        # set joint trajectory
        rospy.wait_for_service(self._SRV_R_SETJOINTTRAJECTORY)
        self._set_joint_trajectory = rospy.ServiceProxy(
            self._SRV_R_SETJOINTTRAJECTORY,
            SetJointTrajectory
        )
        # wait interpolation
        rospy.wait_for_service(self._SRV_R_WAITINTERPOLATION)
        self._wait_interpolation = rospy.ServiceProxy(
            self._SRV_R_WAITINTERPOLATION,
            Empty
        )
        # servo all off
        rospy.wait_for_service(self._SRV_R_SERVOALLOFF)
        self._servo_alloff = rospy.ServiceProxy(
            self._SRV_R_SERVOALLOFF,
            Empty
        )

    def run(self):
        # set control mode to position
        rospy.loginfo("- control mode: POSITION mode")
        self._set_control_mode([1, 1, 1, 1, 1, 1, 1])
        time.sleep(0.2)

        # servo all on
        rospy.loginfo("- servo ALL ON")
        self._servo_allon()
        time.sleep(0.5)

        # set joint trajectory by position mode
        rospy.loginfo("- joint trajectory")
        self._set_joint_trajectory(
            [0.0, -0.055, -2.3, 0.0, 0.0, 0.0, 0.0],
            3.0,
            self._NOMASK,
            self._INTP_MINJERK,
            False
        )
        # wait interpolation
        rospy.loginfo("- wait interpolation...")
        self._wait_interpolation()
        rospy.loginfo("- reached")

        # wait input enter key
        rospy.loginfo("- ")
        rospy.loginfo("After grabbing a right arm of the robot, please press the enter key")
        wait_key_enter()
        rospy.loginfo("- ")

        # change control mode to current, then set joint trajectory
        rospy.loginfo("- change control mode to CURRENT mode from joint 0 to 2")
        self._set_control_mode([3, 3, 3, 1, 1, 1, 1])
        self._set_joint_trajectory(
            [0.136, -0.6, -1.36, 0.0, 0.0, 0.0, 0.0],
            1.0,
            self._NOMASK,
            self._INTP_MINJERK,
            False
        )
        self._wait_interpolation()
        rospy.loginfo("- ok")

        # wait input enter key
        rospy.loginfo("- ")
        rospy.loginfo("finish by pressing the enter key")
        wait_key_enter()

        # set joint trajectory to safety position
        self._set_joint_trajectory(
            [0.0, -0.3, -0.3, 0.0, 0.0, 0.0, 0.0],
            3.0,
            self._NOMASK,
            self._INTP_MINJERK,
            False
        )
        self._wait_interpolation()

        # servo all off
        self._servo_alloff()


try:
    node = ChangeModePos2CurExampleNode()
    node.run()
except rospy.ROSInterruptException:
    pass
