# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME FTO_CarPose)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    cCarPose.h
    cCarPose.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} CarPose) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
