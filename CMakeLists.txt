# Copyright: (C)  Alessandro Settimi, Mirko Ferrati, Arash Ajoudani, Jinoh Lee
# Authors: Alessandro Settimi, Mirko Ferrati
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 OLD)
    cmake_policy(SET CMP0005 NEW)
endif(COMMAND cmake_policy)
include(ExternalProject)
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules/")

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(drc_shared REQUIRED)
find_package(kdl_codyco REQUIRED)
find_package(TinyXML REQUIRED)
find_package(paramHelp REQUIRED)
find_package(iDynTree REQUIRED)

###########
## Build ##
###########
add_subdirectory(app)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${YARP_INCLUDE_DIRS}  ${iDynTree_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${drc_shared_INCLUDE_DIRS} ${paramHelp_INCLUDE_DIRS})

FILE(GLOB HEADER_FILES include/*.h)

add_executable(trajectory_generator
       src/trajectory_generator.cpp
       src/main.cpp
       ${HEADER_FILES}
       )

target_link_libraries(trajectory_generator
        ${ICUB_LIBRARIES}
        ${Eigen_LIBRARIES}
        ${YARP_LIBRARIES}
        ${paramHelp_LIBRARIES}
        ${drc_shared_LIBRARIES}
        ${iDynTree_LIBRARIES}
        )

## Specify libraries to link a library or executable target against
# target_link_libraries(yarp_ros_interface ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES} ${catkin_LIBRARIES})
# target_link_libraries(yarp_ros_joint_state_publisher_node yarp_ros_interface ${YARP_LIBRARIES} icubmod                        ${ICUB_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############


## Mark executables and/or libraries for installation
install(TARGETS trajectory_generator RUNTIME DESTINATION bin)