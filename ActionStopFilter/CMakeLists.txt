# declare our filter
set(FILTER_NAME user_ActionStopFilter)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    stdafx.cpp

    ActionStop.h
    ActionStop.cpp
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/action_stop_filter) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
