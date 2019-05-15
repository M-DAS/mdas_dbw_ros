cmake_minimum_required(VERSION 2.8.3)
project(mdas_dbw_joystick_demo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  joy
  mdas_dbw_msgs
)

catkin_package(
    CATKIN_DEPENDS
    rospy
    joy
    std_msgs
    mdas_dbw_msgs
)

catkin_python_setup()

catkin_install_python(PROGRAMS bin/joystick_demo
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
