/* Copyright [2014] [Alessandro Settimi, Mirko Ferrati, Arash Ajoudani, Jinoh Lee]
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.*/

#include <yarp/os/all.h>
#include <drc_shared/generic_module.hpp>
#include "trajectory_generator.h"
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
