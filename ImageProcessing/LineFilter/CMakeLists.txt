set(FILTER_NAME line_filter)

# This is required by CMake
cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
set (CMAKE_CXX_STANDARD 11)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})


adtf_add_filter(${FILTER_NAME}
    stdafx.h

    LineFilterAdapter.h
    LineFilterAdapter.cpp
    LineFilter.h
    LineFilter.cpp
)


target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

adtf_set_folder(${FILTER_NAME} aadctraffic)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
