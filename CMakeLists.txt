# Copyright: (C)  Alessandro Settimi, Mirko Ferrati, Arash Ajoudani, Jinoh Lee
# Authors: Alessandro Settimi, Mirko Ferrati
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

#stuff to provide a shared library to the whole superbuild
include(ExternalProject)

set(VARS_PREFIX "TRAJECTORY_GENERATOR")
find_package(YCM REQUIRED)
include(YCMDefaultDirs)
ycm_default_dirs(${VARS_PREFIX})
#end superbuild stuff


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
find_package(roscpp REQUIRED)
find_package(octomap REQUIRED)
find_package(message_generation REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp octomap octomap_ros octomap_msgs)
## System dependencies are found with CMake's conventions
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(drc_shared REQUIRED)
find_package(kdl_codyco REQUIRED)
find_package(TinyXML REQUIRED)
find_package(paramHelp REQUIRED)
find_package(iDynTree REQUIRED)

# set(ROS_LIB_FOLDER $ENV{ROS_LIB_FOLDER} CACHE STRING "ros lib folder")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy roslib 
  DEPENDS system_lib
)


add_subdirectory(app)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${YARP_INCLUDE_DIRS}  ${iDynTree_INCLUDE_DIRS} ${ICUB_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${drc_shared_INCLUDE_DIRS} ${paramHelp_INCLUDE_DIRS})

FILE(GLOB_RECURSE HEADER_FILES include/trajectory_generator/*.h)

add_executable(trajectory_generator
       src/trajectory_generator_thread.cpp
       src/main.cpp
       src/bezier_curve.cpp
       ${HEADER_FILES}
       )

add_library(trajectory_generator_lib SHARED src/trajectory_generator.cpp src/bezier_curve.cpp ${HEADER_FILES})

set_target_properties(trajectory_generator_lib PROPERTIES PUBLIC_HEADER "${HEADER_FILES}")

target_link_libraries(trajectory_generator_lib ${kdl_codyco_LIBRARIES} ${catkin_LIBRARIES})

#install for superbuild
target_include_directories(trajectory_generator_lib PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>"
			  "$<INSTALL_INTERFACE:${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")
			  
install(TARGETS trajectory_generator_lib
    EXPORT trajectory_generator #note that you need to change at least one character
    ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT lib
    RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
    LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib
    PUBLIC_HEADER DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}/trajectory_generator" COMPONENT dev
    )
    
set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS trajectory_generator_lib)

include(InstallBasicPackageFiles)

install_basic_package_files(trajectory_generator VARS_PREFIX ${VARS_PREFIX}
                                       VERSION "1.0.0"
                                       TARGETS_PROPERTY ${VARS_PREFIX}_TARGETS
                                       COMPATIBILITY SameMajorVersion

)

include(AddUninstallTarget)
        
## Specify libraries to link a library or executable target against
# target_link_libraries(yarp_ros_interface ${YARP_LIBRARIES} icubmod ${ICUB_LIBRARIES} ${catkin_LIBRARIES})
# target_link_libraries(yarp_ros_joint_state_publisher_node yarp_ros_interface ${YARP_LIBRARIES} icubmod                        ${ICUB_LIBRARIES} ${catkin_LIBRARIES})

       
target_link_libraries(trajectory_generator
	trajectory_generator_lib
        ${ICUB_LIBRARIES}
        ${Eigen_LIBRARIES}
        ${YARP_LIBRARIES}
        ${paramHelp_LIBRARIES}
        ${drc_shared_LIBRARIES}
        ${iDynTree_LIBRARIES}
        ${Octomap_LIBRARIES}
        ${catkin_LIBRARIES}
        )
   
#############
## Install ##
#############


## Mark executables and/or libraries for installation

install(TARGETS trajectory_generator RUNTIME DESTINATION bin)
