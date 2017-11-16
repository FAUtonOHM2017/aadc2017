# add the include directory of the filter
include_directories(../../include)

set(FILTER_NAME user_roadSignDetection)

#set include directories
include_directories(${aruco_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIR}) 

# add the source
adtf_add_filter(${FILTER_NAME}
				stdafx.h
				RoadSignDetectionAdapter.h
				RoadSignDetectionAdapter.cpp
				TTrafficSign.h
				TSignalValue.h
				TPoseStruct.h
				TFeedbackStruct.h
				TActionStruct.h
				aruco_helpers.h
		)

# linking libs
target_link_libraries(${FILTER_NAME} ${OpenCV_LIBS} ${aruco_LIBS})

# set folder (only needed and available for MSVC)
adtf_set_folder(${FILTER_NAME} aadcuser/imageproc) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})	
