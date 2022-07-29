# -*- coding: utf-8 -*-
import rospy
import actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal


class GripperExampleNode(object):
    def __init__(self):
        rospy.init_node('gripper_example_node')
        self._right_gripper_client = actionlib.SimpleActionClient(
            '/amber_righthand/gripper_cmd',
            GripperCommandAction
        )
        self._right_gripper_client.wait_for_server()
        self._left_gripper_client = actionlib.SimpleActionClient(
            '/amber_lefthand/gripper_cmd',
            GripperCommandAction
        )
        self._left_gripper_client.wait_for_server()

    def open_hands(self):
        goal_r = GripperCommandGoal()
        goal_r.command.position = 0.1
        goal_r.command.max_effort = 0
        self._right_gripper_client.send_goal(goal_r)
        goal_l = GripperCommandGoal()
        goal_l.command.position = 0.1
        goal_l.command.max_effort = 0
        self._left_gripper_client.send_goal(goal_l)
        self._right_gripper_client.wait_for_result()
        self._left_gripper_client.wait_for_result()

    def close_hands(self):
        goal_r = GripperCommandGoal()
        goal_r.command.position = 0.01
        goal_r.command.max_effort = 0.0
        self._right_gripper_client.send_goal(goal_r)
        goal_l = GripperCommandGoal()
        goal_l.command.position = 0.01
        goal_l.command.max_effort = 0.0
        self._left_gripper_client.send_goal(goal_l)
        self._right_gripper_client.wait_for_result()
        self._left_gripper_client.wait_for_result()

    def run(self):
        rospy.loginfo('Start example')

        rospy.loginfo('Open hands')
        self.open_hands()

        rospy.loginfo('Close hands')
        self.close_hands()

        rospy.loginfo('Finish example')
        rospy.spin()
