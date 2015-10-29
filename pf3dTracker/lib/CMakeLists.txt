# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale, Matteo Taiana, Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
find_package(OpenCV REQUIRED)
file(GLOB folder_source *.cpp)
file(GLOB folder_header ${CMAKE_CURRENT_SOURCE_DIR}/include/iCub/*.hpp)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_library(pf3d_tracker_lib ${folder_source} ${folder_header})
