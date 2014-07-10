cmake_minimum_required(VERSION 2.8.3)
project(objectrecognition)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  rospy
  std_msgs
  bottomup_msgs
  rosconsole
  tabletop_object_segmentation_online
  tf
  pcl_conversions
  perception_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_move_group
  moveit_msgs
  shape_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
find_package(VTK REQUIRED)

include_directories(include   
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    objectrecognition_sv 
  CATKIN_DEPENDS
    roscpp
    bottomup_msgs
    rosconsole
    tf
    pcl_conversions
    pcl_ros
    perception_msgs
  DEPENDS
    Eigen
)

add_library(objectrecognition_sv 
	src/${PROJECT_NAME}/point_pair.cpp  
	src/${PROJECT_NAME}/object_model.cpp
	src/${PROJECT_NAME}/pose_estimation.cpp
	src/${PROJECT_NAME}_sv/object_model.cpp 
	src/${PROJECT_NAME}_sv/pose_estimation.cpp
	src/${PROJECT_NAME}_sv/point_pair.cpp  
	)



#add_library(objectrecognition_si 
#	src/objectrecognition_si/object_model.cpp 
#	src/objectrecognition_si/pose_estimation.cpp 
#	)

add_executable(object_detector_sv src/ros_objectrecognition_sv/household_object_detector.cpp)
target_link_libraries(object_detector_sv  objectrecognition_sv  -lpq ${Boost_LIBRARIES}  ${PCL_LIBRARIES} vtkCommon vtkGraphics vtkIO vtkFiltering ${catkin_LIBRARIES})

add_executable(object_recognition_pose_estimation_server src/ros_objectrecognition_sv/household_object_detector_server.cpp)
target_link_libraries(object_recognition_pose_estimation_server  objectrecognition_sv  -lpq ${Boost_LIBRARIES}  ${PCL_LIBRARIES} vtkCommon vtkGraphics vtkIO vtkFiltering ${catkin_LIBRARIES})

#target_link_libraries(object_detector_sv ${catkin_LIBRARIES} objectrecognition_sv vtkCommon vtkGraphics vtkIO vtkFiltering  -lpq ${Boost_LIBRARIES} ${PCL_LIBRARIES})
#add_executable(object_graspable_points src/ros_objectrecognition_sv/graspable_points.cpp)
#target_link_libraries(object_graspable_points objectrecognition_sv -lboost_serialization vtkCommon vtkGraphics vtkIO vtkFiltering  -lpq ${PCL_LIBRARIES})

#add_executable(object_detector_si src/ros_objectrecognition_si/household_object_detector.cpp)
#target_link_libraries(object_detector_si objectrecognition_si objectrecognition_sv -lboost_serialization ${VTK_LIBRARIES} -lpq)

#add_executable(object_detector_sv_openmp src/ros_objectrecognition_sv/household_object_detector_openmp.cpp)
#target_link_libraries(object_detector_sv_openmp objectrecognition_sv -lboost_serialization vtkCommon vtkGraphics vtkIO vtkFiltering -lpq)

add_executable(obj_to_ply src/obj_to_ply.c src/ply_io.c )
target_link_libraries(obj_to_ply  -lm ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable(upsampling src/upsampling.cpp)
target_link_libraries(upsampling  ${Boost_LIBRARIES} ${PCL_LIBRARIES} -lpq)


add_executable(particle_filter src/ros_objectrecognition_sv/particle_filter.cpp)
target_link_libraries(particle_filter ${Boost_LIBRARIES}  ${PCL_LIBRARIES} ${catkin_LIBRARIES})

