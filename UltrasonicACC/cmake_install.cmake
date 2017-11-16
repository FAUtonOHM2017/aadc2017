# Install script for directory: /home/aadc/ADTF/aadc2017/src/aadcUser/UltrasonicACC

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    if(EXISTS "$ENV{DESTDIR}/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/debug/user_UltrasonicACC.plb" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/debug/user_UltrasonicACC.plb")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/debug/user_UltrasonicACC.plb"
           RPATH "")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/debug/user_UltrasonicACC.plb")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/debug" TYPE MODULE FILES "/home/aadc/ADTF/aadc2017/src/aadcUser/UltrasonicACC/user_UltrasonicACC.plb")
    if(EXISTS "$ENV{DESTDIR}/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/debug/user_UltrasonicACC.plb" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/debug/user_UltrasonicACC.plb")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/debug/user_UltrasonicACC.plb"
           OLD_RPATH "/opt/adtf/2.14.0/lib:/usr/local/cuda/lib64:/home/aadc/ADTF/aadc2017/src/aadcUser/drawerlib:/home/aadc/ADTF/aadc2017/src/aadcUser/mixinlib:/home/aadc/ADTF/aadc2017/src/aadcUser/mixinlib_qtext:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/debug/user_UltrasonicACC.plb")
      endif()
    endif()
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee]|[Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    if(EXISTS "$ENV{DESTDIR}/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/user_UltrasonicACC.plb" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/user_UltrasonicACC.plb")
      file(RPATH_CHECK
           FILE "$ENV{DESTDIR}/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/user_UltrasonicACC.plb"
           RPATH "")
    endif()
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/user_UltrasonicACC.plb")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin" TYPE MODULE FILES "/home/aadc/ADTF/aadc2017/src/aadcUser/UltrasonicACC/user_UltrasonicACC.plb")
    if(EXISTS "$ENV{DESTDIR}/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/user_UltrasonicACC.plb" AND
       NOT IS_SYMLINK "$ENV{DESTDIR}/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/user_UltrasonicACC.plb")
      file(RPATH_CHANGE
           FILE "$ENV{DESTDIR}/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/user_UltrasonicACC.plb"
           OLD_RPATH "/opt/adtf/2.14.0/lib:/usr/local/cuda/lib64:/home/aadc/ADTF/aadc2017/src/aadcUser/drawerlib:/home/aadc/ADTF/aadc2017/src/aadcUser/mixinlib:/home/aadc/ADTF/aadc2017/src/aadcUser/mixinlib_qtext:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/home/aadc/ADTF/aadc2017/src/aadcUser/../../_install/linux64/bin/user_UltrasonicACC.plb")
      endif()
    endif()
  endif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee]|[Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
endif()

