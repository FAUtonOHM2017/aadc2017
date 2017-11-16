cmake_minimum_required(VERSION 2.8.4 FATAL_ERROR)

cmake_policy(SET CMP0002 NEW)
cmake_policy(SET CMP0003 NEW)
cmake_policy(SET CMP0011 NEW)

# THIS HAS TO COME BEFORE THE PROJECT LINE
set(CMAKE_C_COMPILER "gcc-4.8")
set(CMAKE_CXX_COMPILER "g++-4.8")

# The project and cmake vars must only be defined in case that only the base
# filter should be build.
if(NOT AADC_BUILD_ALL_IN_ONE)
    # define a project name
    project(aadc_user)

    # CMAKE_BUILD_TYPE is empty on Unix systems, but needed by ADTF. Otherwise the ADTF libs
    # can't be found and linked.
    if(UNIX AND NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
                "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
                FORCE)
    endif(UNIX AND NOT CMAKE_BUILD_TYPE)


endif(NOT AADC_BUILD_ALL_IN_ONE)

find_package(ADTF REQUIRED)
#-------General------------

set(AADC_FOUND FALSE)
set(AADC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../)
find_package(AADC REQUIRED)
if (AADC_FOUND)
   message("-- AADC found")
else(AADC_FOUND)
   message(FATAL_ERROR "-- AADC not found")
endif (AADC_FOUND)

#--------------add private macros-----------------
include(${AADC_DIR}/AADC_PRIVATE.cmake)
include(${AADC_DIR}/AADCConfigVersion.cmake)

#--------------sources----------------------------
include_directories(${AADC_DIR}/include)

add_subdirectory(ActionStopFilter)
add_subdirectory(FTO_CarPose)
add_subdirectory(FTO_MarkerDetector)
add_subdirectory(FTO_MarkerPos)
add_subdirectory(FTO_MoveToPoint)
add_subdirectory(FTO_ObjectSpecifier)
add_subdirectory(FTO_OutputLimiter)
add_subdirectory(FTO_PIDController)
add_subdirectory(ImageProcessing)
add_subdirectory(LightControl)
add_subdirectory(ObstacleDetection)
add_subdirectory(StateControl)
add_subdirectory(TimerFilter)
add_subdirectory(UltrasonicACC)
add_subdirectory(UltrasonicCheck)
add_subdirectory(UltrasonicEmergencyActiveBreak)
add_subdirectory(UltrasonicFilter)
add_subdirectory(UltrasonicStructSplit)

