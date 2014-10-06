# Copyright: (C)  Alessandro Settimi, Mirko Ferrati, Arash Ajoudani, Jinoh Lee
# Authors: Alessandro Settimi, Mirko Ferrati
# CopyPolicy: Released under the terms of the GNU GPL v3.0.

# find YARP
find_package(YARP REQUIRED)
# pick up yarp's cmake scripts
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)    
# get all the .ini files in conf
file(GLOB conf conf/*.ini)
# install all the .ini files in conf in the "default context" directory
yarp_install(FILES ${conf} DESTINATION ${YARP_CONTEXTS_INSTALL_DIR}/trajectory_generator)
