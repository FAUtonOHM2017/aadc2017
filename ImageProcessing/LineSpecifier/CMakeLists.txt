set(FILTER_NAME line_specifier)

# This is required by CMake
cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})


adtf_add_filter(${FILTER_NAME}
    stdafx.h

    LineSpecifierAdapter.h
    LineSpecifierAdapter.cpp
    LineSpecifier.h
    LineSpecifier.cpp
    LineTemplateGenerator.h
    LineTemplateGenerator.cpp
    LineTemplate.h
    LineTemplate.cpp
    stop/StopLineDetector.h
    stop/StopLineDetector.cpp
    lines/LineDetectorHorizontal.cpp
    lines/LineDetectorHorizontal.h
    lines/LineDetectorVertical.cpp
    lines/LineDetectorVertical.h
    parking/ParkingSpotDetector.cpp
    parking/ParkingSpotDetector.h
    crossing/IntersectionDetector.cpp
    crossing/IntersectionDetector.h
    cache/DetectionCache.cpp
    cache/DetectionCache.h
    speed/SpeedRecommender.cpp
    speed/SpeedRecommender.h
    PoseCache.h
)


target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

adtf_set_folder(${FILTER_NAME} aadctraffic)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
