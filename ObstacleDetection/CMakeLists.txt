# add the include directory of the filter
include_directories(../include)

set(FILTER_NAME obstacle_detection)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})


adtf_add_filter(${FILTER_NAME}
    stdafx.h

    ObstacleDetection.h
    ObstacleDetection.cpp
)


target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

adtf_set_folder(${FILTER_NAME} aadcuser/obstacledetection) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
