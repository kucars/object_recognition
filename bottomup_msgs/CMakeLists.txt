cmake_minimum_required(VERSION 2.8.3)
project(bottomup_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# Do not just add this line to your CMakeLists.txt, modify the existing line
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs geometry_msgs message_generation tabletop_object_segmentation_online) #object_recognition_msgs
include_directories(include)


#uncomment if you have defined messages
## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES
   Region.msg
   Object.msg
   Hypothesis.msg
   bottomup_msg.msg
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs tabletop_object_segmentation_online)#object_recognition_msgs
catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs std_msgs tabletop_object_segmentation_online)#object_recognition_msgs

