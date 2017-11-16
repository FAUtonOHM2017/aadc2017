add_subdirectory(ThriftExtIf)

set (CMAKE_CXX_STANDARD 11)

# declare our filter
set(FILTER_NAME fto_ObjectSpecifier)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}
    ${Boost_INCLUDE_DIR} 
    ${Thrift_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	../../../include
	ThriftExtIf/gen-cpp
)

adtf_add_filter(${FILTER_NAME}
    stdafx.h
    ObjectSpecifier.h
    ObjectSpecifier.cpp
)

target_link_libraries(${FILTER_NAME}
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${Thrift_LINK_LIBS}
    ${OpenSSL_LINK_LIBS}
    ThriftExtIf
)

adtf_set_folder(${FILTER_NAME} FTO_ObjectSpecifier) 

# Specify where it should be installed to
adtf_install_plugin(${FILTER_NAME} ${CMAKE_INSTALL_BINARY})
