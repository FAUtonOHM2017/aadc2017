# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME aadc_StateControlManagementSLIM)

# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
              
            stdafx.h
            StateControlManagementSlim.cpp
            StateControlManagementSlim.h
            StateControl.h
           	${HEADER_MOC}
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/StateControlManagementSLIM)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
