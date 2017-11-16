include_directories(../include)
# declare our filter
set(FILTER_NAME user_UltrasonicCheck)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    UltrasonicCheck.h
    UltrasonicCheck.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/ultrasonic_check) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})