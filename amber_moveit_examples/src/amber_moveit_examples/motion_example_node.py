# -*- coding: utf-8 -*-
import rospy
import copy
import moveit_commander
import sys


class MotionExampleNode(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('amber_moveit_motion_example_node')

        group_name_right = "right_arm"
        self._move_group_right = moveit_commander.MoveGroupCommander(group_name_right)
        group_name_left = "left_arm"
        self._move_group_left = moveit_commander.MoveGroupCommander(group_name_left)

    def _get_move_group(self, lr):
        if lr == 'l':
            return self._move_group_left
        elif lr == 'r':
            return self._move_group_right
        else:
            raise RuntimeError('Invalid l/r specification')

    def move_arm_by_joint_angle(self, lr, arm_joints,
                                acceleration_scale=1.0,
                                velocity_scale=1.0,
                                wait=True):
        move_group = self._get_move_group(lr)
        move_group.set_max_acceleration_scaling_factor(acceleration_scale)
        move_group.set_max_velocity_scaling_factor(velocity_scale)
        joint_goal = move_group.get_current_joint_values()
        arm_joints.extend(joint_goal[5:])
        move_group.go(arm_joints, wait=wait)

        current_joints = move_group.get_current_joint_values()
        rospy.loginfo('Current joints: ' + str(current_joints))
        move_group.stop()

    def move_arm_by_pose(self, lr, eef_pose,
                         acceleration_scale=1.0,
                         velocity_scale=1.0,
                         wait=True):
        move_group = self._get_move_group(lr)
        move_group.set_max_acceleration_scaling_factor(acceleration_scale)
        move_group.set_max_velocity_scaling_factor(velocity_scale)
        move_group.set_joint_value_target(eef_pose, True)
        move_group.go(wait=wait)
        move_group.clear_pose_targets()

        current_pose = move_group.get_current_pose().pose
        rospy.loginfo('Current pose: ' + str(current_pose))
        move_group.stop()

    def plan_and_execute_motion_by_pose(self, lr, eef_pose,
                                        acceleration_scale=1.0,
                                        velocity_scale=1.0,
                                        wait=True):
        move_group = self._get_move_group(lr)
        move_group.set_max_acceleration_scaling_factor(acceleration_scale)
        move_group.set_max_velocity_scaling_factor(velocity_scale)
        waypoints = []
        waypoints.append(copy.deepcopy(eef_pose))
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        move_group.execute(plan, wait=wait)

        current_pose = move_group.get_current_pose().pose
        rospy.loginfo('Current pose: ' + str(current_pose))
        move_group.stop()

    def goto_init_pose(self):
        init_pose = [0.0, -0.057, -2.35, 0.0, 0.0]
        self.move_arm_by_joint_angle('r', init_pose)
        self.move_arm_by_joint_angle('l', init_pose)

    def run(self):
        rospy.loginfo('Start example')
        rospy.loginfo('Go to initial pose')
        self.goto_init_pose()

        rospy.loginfo('Move arms by joint angles')
        target_joints_r = [-0.3, -0.35, -1.2, 1.41, 0.0]
        target_joints_l = [0.3, -0.35, -1.2, 1.41, 0.0]

        self.move_arm_by_joint_angle('r',
                                     target_joints_r,
                                     velocity_scale=0.1,
                                     wait=True)
        self.move_arm_by_joint_angle('l',
                                     target_joints_l,
                                     velocity_scale=0.1,
                                     wait=True)

        rospy.loginfo('Move arms by eef poses')
        target_pose_r = self._move_group_right.get_current_pose().pose
        target_pose_r.position.x = 0.2571589194651918
        target_pose_r.position.y = -0.25342840253637927
        target_pose_r.position.z = 0.3543721703683328
        target_pose_l = self._move_group_left.get_current_pose().pose
        target_pose_l.position.x = 0.19242117784777
        target_pose_l.position.y = 0.2797439261551028
        target_pose_l.position.z = 0.47574641133309103

        self.move_arm_by_pose('r',
                              target_pose_r,
                              velocity_scale=0.1,
                              wait=True)
        self.plan_and_execute_motion_by_pose('l',
                                             target_pose_l,
                                             velocity_scale=0.1)

        rospy.loginfo('Return to initial pose')
        self.goto_init_pose()

        rospy.loginfo('Finish example')
        rospy.spin()
