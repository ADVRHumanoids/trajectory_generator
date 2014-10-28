/* Copyright [2014] [Corrado Pavan, Alessandro Settimi, Mirko Ferrati, Arash Ajoudani, Jinoh Lee]
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

#ifndef TRJ_GEN_MOD_H
#define TRJ_GEN_MOD_H

#include <drc_shared/generic_module.hpp>
#include "trajectory_generator_thread.h"

class trajectory_generator_module: public generic_module<trajectory_generator_thread>
{
public:
    trajectory_generator_module(int argc, char* argv[], std::string module_prefix, int module_period, yarp::os::ResourceFinder rf)
    :generic_module< trajectory_generator_thread >(argc,argv,module_prefix,module_period,rf)
    {}
    
    
  
};

#endif //TRJ_GEN_MOD_H