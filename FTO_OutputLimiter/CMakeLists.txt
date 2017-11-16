# add the include directory of the filter
include_directories(../include)
# declare our filter
set(FILTER_NAME FTO_Output_Limiter)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    cOutputLimiter.h
    cOutputLimiter.cpp
)

adtf_set_folder(${FILTER_NAME} OutputLimiter)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
