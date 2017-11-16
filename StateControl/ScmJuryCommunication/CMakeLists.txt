# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME aadc_ScmJuryCommunication)

# Specify the filter module target
adtf_add_filter(${FILTER_NAME}

            stdafx.h
            ScmJuryCommunication.cpp
          	ScmJuryCommunication.h
         	  Maneuverlist.h
          	StateControl.h
           	${HEADER_MOC}
)

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/ScmJuryCommunication)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
