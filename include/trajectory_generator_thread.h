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

#ifndef TRJ_GEN_TH_H
#define TRJ_GEN_TH_H


#include <yarp/os/all.h>
#include <drc_shared/yarp_status_interface.h>
#include <drc_shared/yarp_rpc_interface.hpp>
#include <drc_shared/yarp_msgs/trj_generator_msg.h>
#include <drc_shared/generic_thread.hpp>
#include <trajectory_generator/trajectory_generator.h>

class trajectory_generator_thread: public generic_thread
{
public:
    trajectory_generator_thread(std::string module_prefix, yarp::os::ResourceFinder rf,std::shared_ptr<paramHelp::ParamHelperServer> ph);
    
private:
    yarp::os::Network* yarp;
    double period;
 
    walkman::drc::yarp_custom_rpc_server_interface<trj_generator_input_msg,trj_generator_output_msg> rpc_server;
    
    trajectory_generator trj_generator;
    trj_generator_input_msg in_msg;
    trj_generator_output_msg out_msg;
    walkman::drc::yarp_status_interface status_interface;
    
    void compute_line_trj(trj_generator_input_msg& in, trj_generator_output_msg& out);
    void compute_circle_trj(trj_generator_input_msg& in, trj_generator_output_msg& out);
    
public:
    
    virtual void run();
    virtual bool custom_init();
    virtual bool custom_pause();
    virtual bool custom_resume();
};

#endif //TRJ_GEN_TH_H