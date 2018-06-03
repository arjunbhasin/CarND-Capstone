# Install script for directory: /home/arjun/workspace/SelfDrivingCars/CarND-Capstone/ros/src/darknet_ros/darknet_ros

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/arjun/workspace/SelfDrivingCars/CarND-Capstone/ros/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/arjun/workspace/SelfDrivingCars/CarND-Capstone/ros/build/darknet_ros/darknet_ros/catkin_generated/installspace/darknet_ros.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/darknet_ros/cmake" TYPE FILE FILES
    "/home/arjun/workspace/SelfDrivingCars/CarND-Capstone/ros/build/darknet_ros/darknet_ros/catkin_generated/installspace/darknet_rosConfig.cmake"
    "/home/arjun/workspace/SelfDrivingCars/CarND-Capstone/ros/build/darknet_ros/darknet_ros/catkin_generated/installspace/darknet_rosConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/darknet_ros" TYPE FILE FILES "/home/arjun/workspace/SelfDrivingCars/CarND-Capstone/ros/src/darknet_ros/darknet_ros/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/arjun/workspace/SelfDrivingCars/CarND-Capstone/ros/build/darknet_ros/darknet_ros/CMakeFiles/CMakeRelink.dir/libdarknet_ros_lib.so")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/darknet_ros" TYPE EXECUTABLE FILES "/home/arjun/workspace/SelfDrivingCars/CarND-Capstone/ros/build/darknet_ros/darknet_ros/CMakeFiles/CMakeRelink.dir/darknet_ros")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/darknet_ros" TYPE DIRECTORY FILES "/home/arjun/workspace/SelfDrivingCars/CarND-Capstone/ros/src/darknet_ros/darknet_ros/include/darknet_ros/" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

