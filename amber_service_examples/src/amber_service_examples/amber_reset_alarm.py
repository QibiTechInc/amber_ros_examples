#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from amber_ros_driver.srv import SetJointNo


class ResetAlarmExampleNode(object):
    def __init__(self):
        rospy.init_node('reset_alarm_example_node')

        self._service_name_right = '/amber_right/alarm_reset'

        rospy.wait_for_service(self._service_name_right)
        self._reset_alarm_right = rospy.ServiceProxy(
            self._service_name_right,
            SetJointNo
        )

    def run(self):
        self._reset_alarm_right(0)
        self._reset_alarm_right(1)
        self._reset_alarm_right(2)
        self._reset_alarm_right(3)
        self._reset_alarm_right(4)
        self._reset_alarm_right(5)
        self._reset_alarm_right(6)


try:
    node = ResetAlarmExampleNode()
    node.run()
except rospy.ROSInterruptException:
    pass
