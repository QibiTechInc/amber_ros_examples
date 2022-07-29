#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Empty


class ServoAllOnExampleNode(object):
    def __init__(self):
        rospy.init_node('servo_allon_example_node')

        self._service_name_right = '/amber_right/servo_all_on'

        rospy.wait_for_service(self._service_name_right)
        self._servo_allon_right = rospy.ServiceProxy(
            self._service_name_right,
            Empty
        )

    def run(self):
        self._servo_allon_right()


try:
    node = ServoAllOnExampleNode()
    node.run()
except rospy.ROSInterruptException:
    pass
