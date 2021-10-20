cmake_minimum_required(VERSION 3.0.2)
project(pybullet_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  sensor_msgs
  franka_panda_description
)

catkin_python_setup()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS scripts/pybullet_ros DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
