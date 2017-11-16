include_directories(../include)
# declare our filter
set(FILTER_NAME user_UltrasonicACC)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    UltrasonicACC.h
    UltrasonicACC.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/ultrasonic_acc) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})