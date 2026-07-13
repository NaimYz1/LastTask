cmake_minimum_required(VERSION 3.0.2)
project(fyp_jackal_navigation)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/mid360_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch params maps rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
