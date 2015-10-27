# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale, Matteo Taiana, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.3)
project(pf3d_tracker)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  std_msgs
  geometry_msgs
  message_generation
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  Estimates.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES foveated_stereo_ros
#  CATKIN_DEPENDS cv_bridge geometry_msgs message_filters roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
)


#common commands for building c++ executables and libraries

INCLUDE_DIRECTORIES(${GSL_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(./include)

SUBDIRS(src)



