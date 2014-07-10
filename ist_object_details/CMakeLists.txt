cmake_minimum_required(VERSION 2.8.3)
project(ist_object_details)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros eigen_conversions tf_conversions tf perception_msgs std_srvs)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV)

include_directories(include   
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${VTK_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${VTK_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)

catkin_package(CATKIN_DEPENDS 
  pcl_ros eigen_conversions tf_conversions tf perception_msgs std_srvs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES object_details
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

#include_directories(/usr/include/vtk-5.6)
add_library(object_details src/object_details/object_details.cpp)
add_executable(ros_object_details src/ros_object_details/ros_object_details.cpp)
target_link_libraries(ros_object_details object_details libvtkCommon.so libvtkFiltering.so libvtkRendering.so  ${Eigen_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES}   ${OpenCV_LIBRARIES})



