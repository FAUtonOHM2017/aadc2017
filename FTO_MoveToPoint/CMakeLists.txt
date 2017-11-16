# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME FTO_Move_To_Point)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    bezier.h
    bezier.cpp
    cMoveToPoint.h
    cMoveToPoint.cpp
    tinyxml2.h
    tinyxml2.cpp
)

adtf_set_folder(${FILTER_NAME} FTO_Move_To_Point)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
