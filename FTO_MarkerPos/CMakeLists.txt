set(FILTER_NAME FTO_markerPos)
include_directories(../include)

# include directories
include_directories(${OpenCV_INCLUDE_DIR})

# Specify the filter module target
adtf_add_filter(${FILTER_NAME}
                cMarkerPos.cpp
                
)

target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} FTO_position)

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
