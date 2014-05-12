cmake_minimum_required(VERSION 2.8.3)
project(perception)

catkin_python_setup() 
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS actionlib_msgs actionlib message_generation tabletop_object_segmentation_online bottomup_msgs)

add_action_files(
  DIRECTORY action
  FILES DetectObjects.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs tabletop_object_segmentation_online bottomup_msgs# Or other packages containing msgs
)

catkin_package(CATKIN_DEPENDS 
  tabletop_object_segmentation_online 
  actionlib_msgs
  actionlib
  message_generation
  bottomup_msgs
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


