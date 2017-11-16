# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME user_UltrasonicEmergencyActiveBreak)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    UltrasonicEmergencyActiveBreak.h
    UltrasonicEmergencyActiveBreak.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/ultrasonic_emergency_stop_activeBreak) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})