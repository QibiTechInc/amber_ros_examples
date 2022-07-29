cmake_minimum_required(VERSION 3.0.2)
project(amber_moveit_examples)

find_package(catkin REQUIRED COMPONENTS
  rospy actionlib geometry_msgs moveit_core moveit_msgs moveit_ros_planning moveit_ros_planning_interface sensor_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy actionlib geometry_msgs moveit_core moveit_msgs moveit_ros_planning moveit_ros_planning_interface sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/motion_example_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/show_eefpose_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/gripper_example_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(PROGRAMS
  scripts/concurrent_motion_example_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
