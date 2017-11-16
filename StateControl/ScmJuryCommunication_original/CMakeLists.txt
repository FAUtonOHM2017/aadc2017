# add the include directory of the filter
include_directories(../include)
set(FILTER_NAME aadc_ScmJuryCommunication)

   
adtf_add_filter(${FILTER_NAME}
            stdafx.cpp
            stdafx.h
            juryEnums.h
            ScmJuryCommunication.cpp            
            ScmJuryCommunication.h
            Maneuverlist.h
            StateControl.h
            ${HEADER_MOC}
)


# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/ScmJuryCommunication) 

adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})    




