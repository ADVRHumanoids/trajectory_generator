
#include <iostream>
#include <yarp/os/all.h>
#include <drc_shared/generic_module.hpp>
#include "trajectory_generator_thread.h"
#include "trajectory_generator_module.hpp"

#define MODULE_PERIOD 1000

int main(int argc, char **argv) 
{
    yarp::os::Network yarp;
    if(!yarp.checkNetwork()){
	std::cout<<"yarpserver not running, pls run yarpserver"<<std::endl;
	return 0;
    }
    yarp.init();

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile( "trajectory_generator.ini" ); 
    rf.setDefaultContext("trajectory_generator");
    rf.configure(argc,argv);
    
    trajectory_generator_module trj = trajectory_generator_module(argc,argv,"trajectory_generator",MODULE_PERIOD,rf);

    yarp.fini();
    
    trj.runModule();
    
    exit(EXIT_SUCCESS);
}