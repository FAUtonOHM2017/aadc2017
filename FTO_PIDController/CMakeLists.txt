# add the include directory of the filter
include_directories(../../include)
# declare our filter
set(FILTER_NAME FTO_PIDcontroller)
 
# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                stdafx.cpp     
                stdafx.h
                PIDController.h
                PIDController.cpp
		TSignalValue.h
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} pidcontroller) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
