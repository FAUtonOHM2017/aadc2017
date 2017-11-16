# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME user_TimerFilter)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    TimerFilter.h
    TimerFilter.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/timerfilter) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})