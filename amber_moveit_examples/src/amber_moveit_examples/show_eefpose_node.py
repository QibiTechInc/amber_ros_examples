# -*- coding: utf-8 -*-
import rospy
import moveit_commander
import sys


class ShowEefposeNode(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('amber_moveit_show_eefpose_node')

        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()

        group_name_left = "left_arm"
        self._move_group_left = moveit_commander.MoveGroupCommander(group_name_left)
        group_name_right = "right_arm"
        self._move_group_right = moveit_commander.MoveGroupCommander(group_name_right)

    def _get_move_group(self, lr):
        if lr == 'l':
            return self._move_group_left
        elif lr == 'r':
            return self._move_group_right
        else:
            raise RuntimeError('Invalid l/r specification')

    def get_eefpose(self, lr):
        move_group = self._get_move_group(lr)
        current_pose = move_group.get_current_pose().pose
        return current_pose

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            pose_r = self.get_eefpose('r')
            pose_l = self.get_eefpose('l')
            rospy.loginfo('Current pose(r): ' + str(pose_r))
            rospy.loginfo('Current pose(l): ' + str(pose_l))
            r.sleep()
