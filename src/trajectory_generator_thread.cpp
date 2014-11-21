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

#include "trajectory_generator_thread.h"

trajectory_generator_thread::trajectory_generator_thread(std::string module_prefix, yarp::os::ResourceFinder rf,std::shared_ptr<paramHelp::ParamHelperServer> ph)
: generic_thread(module_prefix,rf,ph),rpc_server(module_prefix),status_interface(module_prefix)
{

}

bool trajectory_generator_thread::custom_init()
{
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
    
    return true;
}

void trajectory_generator_thread::compute_line_trj(trj_generator_input_msg& in, trj_generator_output_msg& out)
{    
    trj_generator.line_initialize(in.time,in.start,in.displacement);
//     trj_generator.line_trajectory(out.trj); TODO correct this
    
    out.command="result_trj";
}


void trajectory_generator_thread::compute_custom_circle_trj(trj_generator_input_msg& in, trj_generator_output_msg& out)
{    
    trj_generator.custom_circle_initialize(in.time, in.start, in.displacement, in.left, in.hand, in.center_angle, in.radius);
    trj_generator.complete_custom_circle_trajectory(out.trj);
    
    out.command="result_trj";
}


void trajectory_generator_thread::run()
{
    int seq_num;
        
    if(rpc_server.getCommand(in_msg,seq_num))
    {
        std::string command = in_msg.command;

        std::cout<<" - YARP: Command ["<<seq_num<<"] received: "<<command<<std::endl;
	
	if(command=="line")
	{
	    compute_line_trj(in_msg,out_msg);
	    rpc_server.reply(out_msg,seq_num);
	}
	if(command=="circle")
	{
	    compute_custom_circle_trj(in_msg,out_msg);
	    rpc_server.reply(out_msg,seq_num);   
	}
    }
}

bool trajectory_generator_thread::custom_pause()
{

}

bool trajectory_generator_thread::custom_resume()
{

}

