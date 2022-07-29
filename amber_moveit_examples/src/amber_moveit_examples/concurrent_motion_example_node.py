# -*- coding: utf-8 -*-
import rospy
import copy
from amber_ros_driver.srv import SetJointTrajectory, SetJointTrajectoryRequest
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from moveit_msgs.msg import Constraints, PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK


class ConcurrentMotionExampleNode(object):
    def __init__(self):
        rospy.init_node('amber_concurrent_motion_example_node')

        # parameters
        self._namespace_left = rospy.get_param('~namespace_left', 'amber_left')
        self._namespace_right = rospy.get_param('~namespace_right', 'amber_right')

        # attributes
        self._arm_mask = [0, 0, 0, 0, 0, 1, 1]
        self._gripper_mask = [1, 1, 1, 1, 1, 0, 0]

    def set_joint_trajectory_srv_client(self,
                                        namespace,
                                        target_angles,
                                        goal_time,
                                        mask,
                                        method,
                                        relative):
        srv_name = namespace + '/set_joint_trajectory'
        rospy.wait_for_service(srv_name)
        try:
            set_joint_trajectory = rospy.ServiceProxy(srv_name,
                                                      SetJointTrajectory)
            set_joint_trajectory(target_angles, goal_time, mask, method, relative)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def wait_interpolation_srv_client(self, namespace):
        srv_name = namespace + '/wait_interpolation'
        rospy.wait_for_service(srv_name)
        try:
            wait_interpolation = rospy.ServiceProxy(srv_name, Empty)
            wait_interpolation()
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

    def compute_ik_srv_client(self, group_name, target_pose):
        srv_name = '/compute_ik'
        rospy.wait_for_service(srv_name)

        joint_state = rospy.wait_for_message('joint_states', JointState)
        robot_state = RobotState()
        constraints = Constraints()
        robot_state.joint_state = joint_state

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'body'
        pose_stamped.pose = target_pose
        try:
            request = PositionIKRequest()
            request.group_name = group_name
            request.robot_state = robot_state
            request.constraints = constraints
            request.pose_stamped = pose_stamped

            compute_ik = rospy.ServiceProxy(srv_name, GetPositionIK)
            response = compute_ik(request)
            if group_name == 'left_arm':
                position = response.solution.joint_state.position[0:7]
            else:
                position = response.solution.joint_state.position[7:]
            return position
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            return None

    def _get_namespace(self, lr):
        if lr == 'l':
            return self._namespace_left
        elif lr == 'r':
            return self._namespace_right
        else:
            raise RuntimeError('Invalid l/r specification')

    def _get_groupname(self, lr):
        if lr == 'l':
            return 'left_arm'
        elif lr == 'r':
            return 'right_arm'
        else:
            raise RuntimeError('Invalid l/r specification')

    def move_arm_by_joint_angle(self,
                                lr,
                                arm_joints,
                                goal_time):
        namespace = self._get_namespace(lr)
        target_joint_angle = copy.copy(arm_joints)
        target_joint_angle.extend([0.0, 0.0])
        self.set_joint_trajectory_srv_client(namespace,
                                             target_joint_angle,
                                             goal_time,
                                             self._arm_mask,
                                             SetJointTrajectoryRequest.MINJERK,
                                             False)

    def calc_ik_from_pose(self, lr, eef_pose):
        group_name = self._get_groupname(lr)
        target_angles = list(self.compute_ik_srv_client(group_name, eef_pose))
        if len(target_angles) == 7:
            # 最後の回転軸の結果は使わない
            target_angles[4] = 0.0
            return target_angles[:5]
        else:
            rospy.logwarn('Failed to calculate ik')
            return None

    def wait_interpolation(self, lr):
        namespace = self._get_namespace(lr)
        self.wait_interpolation_srv_client(namespace)

    def goto_init_pose(self):
        init_pose = [0.0, -0.057, -2.4, 0.0, 0.0]
        self.move_arm_by_joint_angle('r', init_pose, 3.0)
        self.move_arm_by_joint_angle('l', init_pose, 3.0)
        self.wait_interpolation('r')
        self.wait_interpolation('l')

    def run(self):
        rospy.loginfo('Start example')
        rospy.loginfo('Go to initial pose')
        self.goto_init_pose()

        rospy.loginfo('Move arms by joint angles')
        target_joints_r = [-0.3, -0.35, -1.2, 1.41, 0.0]
        target_joints_l = [0.3, -0.35, -1.2, 1.41, 0.0]

        self.move_arm_by_joint_angle('r',
                                     target_joints_r,
                                     3.0)
        self.move_arm_by_joint_angle('l',
                                     target_joints_l,
                                     3.0)
        self.wait_interpolation('r')
        self.wait_interpolation('l')

        rospy.loginfo('Move arms by eef poses')
        # body座標系でhand座標系の姿勢を指示
        target_pose_r = Pose()
        target_pose_r.position.x = 0.2792974995496962
        target_pose_r.position.y = -0.2651829016841067
        target_pose_r.position.z = 0.5305198023694585
        target_pose_l = Pose()
        target_pose_l.position.x = 0.2074024403073674
        target_pose_l.position.y = 0.1661532906837873
        target_pose_l.position.z = 0.5315360912459752
        target_joints_r = self.calc_ik_from_pose('r', target_pose_r)
        target_joints_l = self.calc_ik_from_pose('l', target_pose_l)
        if target_joints_r is not None:
            self.move_arm_by_joint_angle('r',
                                         target_joints_r,
                                         3.0)
        if target_joints_l is not None:
            self.move_arm_by_joint_angle('l',
                                         target_joints_l,
                                         3.0)
        self.wait_interpolation('r')
        self.wait_interpolation('l')

        rospy.loginfo('Return to initial pose')
        self.goto_init_pose()

        rospy.loginfo('Finish example')
        rospy.spin()
