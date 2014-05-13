cmake_minimum_required(VERSION 2.8.3)
project(perception)

catkin_python_setup() 
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib_msgs actionlib tabletop_object_segmentation_online perception_msgs)


catkin_package(CATKIN_DEPENDS 
  tabletop_object_segmentation_online 
  actionlib_msgs
  actionlib
  perception_msgs
)

install(PROGRAMS nodes/detect_objects_server nodes/detect_objects_client
         DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


