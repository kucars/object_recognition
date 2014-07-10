cmake_minimum_required(VERSION 2.8.3)
project(tabletop_object_segmentation_online)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# Do not just add this line to your CMakeLists.txt, modify the existing line
find_package(catkin REQUIRED COMPONENTS 
	std_msgs
 	pcl_ros 
	sensor_msgs 
	geometry_msgs 
	message_generation 
	object_recognition_msgs 
	pcl_conversions 
	tf)

find_package(PCL REQUIRED)

include_directories(include   
	${catkin_INCLUDE_DIRS} 
	${PCL_INCLUDE_DIRS}
)

#uncomment if you have defined messages
## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES
   Table.msg
   TableRegion.msg
)

add_service_files(
   DIRECTORY srv
   FILES
   TabletopSegmentation.srv
)


generate_messages(DEPENDENCIES sensor_msgs shape_msgs)
catkin_package(CATKIN_DEPENDS 
    message_runtime
    shape_msgs
    roscpp
    sensor_msgs
    std_msgs
    pcl_conversions
    pcl_ros
    sensor_msgs
    tf
)


add_library(marker_generator src/marker_generator.cpp)

add_executable(tabletop_segmentation_online_param_up src/tabletop_segmentation_online_param_up.cpp)
add_dependencies(tabletop_segmentation_online_param_up ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(tabletop_segmentation_online_param_up  marker_generator ${PCL_LIBRARIES} ${catkin_LIBRARIES})	
