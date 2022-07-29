#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_srvs.srv import Empty


class ServoAllOffExampleNode(object):
    def __init__(self):
        rospy.init_node('servo_alloff_example_node')

        self._service_name_right = '/amber_right/servo_all_off'

        rospy.wait_for_service(self._service_name_right)
        self._servo_alloff_right = rospy.ServiceProxy(
            self._service_name_right,
            Empty
        )

    def run(self):
        self._servo_alloff_right()


try:
    node = ServoAllOffExampleNode()
    node.run()
except rospy.ROSInterruptException:
    pass
