cmake_minimum_required(VERSION 2.8)

#Name of the Project (name that appears in IDE)
project(ThriftExtIf)
set(folder_name "libraries")

#SET Source Files
set(SOURCE
	gen-cpp/ExtIf_constants.cpp
	gen-cpp/ExtIf_types.cpp
	gen-cpp/ExtService.cpp
	#GenInterfaceService.cpp
	#GenInterfaceService_server.skeleton.cpp	
)

set(HEADER 
	gen-cpp/ExtIf_constants.h
	gen-cpp/ExtIf_types.h
	gen-cpp/ExtService.h
	
)

include_directories(
	${Thrift_INCLUDE_DIR}
	${Boost_INCLUDE_DIR} 
	)
	
link_directories(
	
	)

add_library(${PROJECT_NAME}  
    ${SOURCE}
    ${HEADER}
)

set_target_properties(${PROJECT_NAME}
        PROPERTIES
        DEBUG_POSTFIX "d"
        RELEASE_POSTFIX ""
        RELWITHDEBINFO_POSTFIX ""
		FOLDER "${folder_name}"
    )

target_link_libraries(${PROJECT_NAME} 
    ${Thrift_LINK_LIBS}
)

a_post_build_install(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_LIBRARY})
install(FILES ${HEADER} DESTINATION "${CMAKE_INSTALL_INCLUDE}/${PROJECT_NAME}")
