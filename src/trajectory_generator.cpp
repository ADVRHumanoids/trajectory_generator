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

#include "trajectory_generator.h"

trajectory_generator::trajectory_generator(std::string module_prefix, yarp::os::ResourceFinder rf,std::shared_ptr<paramHelp::ParamHelperServer> ph)
: generic_thread(module_prefix,rf,ph),command_input_interface(module_prefix),command_output_interface(module_prefix+"_out"),status_interface(module_prefix)
{

}

bool trajectory_generator::custom_init()
{
    struct sched_param thread_param;
    thread_param.sched_priority = 99;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &thread_param);
    
    return true;
}

void trajectory_generator::compute_line_trj(trj_generator_input_msg& in, trj_generator_output_msg& out)
{
    out.trj.clear();
  
    KDL::Frame temp_frame;
    KDL::Vector temp_vector_p, start_vector_p, dis_vector_p;
    KDL::Vector temp_vector_r, start_vector_r, dis_vector_r;

    start_vector_p.data[0] = in.start.p.x();
    start_vector_p.data[1] = in.start.p.y();
    start_vector_p.data[2] = in.start.p.z();
    double ro,pi,ya;
    in.start.M.GetRPY(ro,pi,ya);
    start_vector_r.data[0] = ro;
    start_vector_r.data[1] = pi;
    start_vector_r.data[2] = ya;

    std::cout<<"TRJ: from ("<<in.start.p.x()<<' '<<in.start.p.y()<<' '<<in.start.p.z()<<' '<<ro<<' '<<pi<<' '<<ya<<") ";
    
    dis_vector_p.data[0] = in.displacement.p.x();
    dis_vector_p.data[1] = in.displacement.p.y();
    dis_vector_p.data[2] = in.displacement.p.z();
    double ro_,pi_,ya_;
    in.displacement.M.GetRPY(ro_,pi_,ya_);
    dis_vector_r.data[0] = ro_;
    dis_vector_r.data[1] = pi_;
    dis_vector_r.data[2] = ya_;

    std::cout<<"to ("<<in.displacement.p.x()<<' '<<in.displacement.p.y()<<' '<<in.displacement.p.z()<<' '<<ro_<<' '<<pi_<<' '<<ya_<<") in "<<in.time<<" [s]"<<std::endl;
    
//     std::cout<<"(line ";
    
    for(double t=0;t<=in.time;t=t+0.01)
    {
          //         Xd = Xinit          + 20.0*Xf          /(2.0*Tf*Tf*Tf)*t*t*t          + (-30.0*Xf)          /(2.0*Tf*Tf*Tf*Tf)*t*t*t*t     + (12.0*Xf)          /(2.0*Tf*Tf*Tf*Tf*Tf)*t*t*t*t*t;
	  temp_vector_p = start_vector_p + 20.0*dis_vector_p/(2.0*pow(in.time,3))*pow(t,3) + (-30.0*dis_vector_p)/(2.0*pow(in.time,4))*pow(t,4) + (12.0*dis_vector_p)/(2.0*pow(in.time,5))*pow(t,5);
	  temp_vector_r = start_vector_r + 20.0*dis_vector_r/(2.0*pow(in.time,3))*pow(t,3) + (-30.0*dis_vector_r)/(2.0*pow(in.time,4))*pow(t,4) + (12.0*dis_vector_r)/(2.0*pow(in.time,5))*pow(t,5);
	  
	  temp_frame.p.x(temp_vector_p.data[0]);
	  temp_frame.p.y(temp_vector_p.data[1]);
	  temp_frame.p.z(temp_vector_p.data[2]);
	  temp_frame.M = KDL::Rotation::RPY(temp_vector_r.data[0],temp_vector_r.data[1],temp_vector_r.data[2]);
	  
	  
	  out.trj[t] = temp_frame;
// 	  std::cout<<t<<' '<<temp_vector_p.data[0]<<' '<<temp_vector_p.data[1]<<' '<<temp_vector_p.data[2]<<' '<<temp_vector_r.data[0]<<' '<<temp_vector_r.data[1]<<' '<<temp_vector_r.data[2]<<' ';
	  
    }
    
    out.command="result_trj";
    
//     std::cout<<')'<<std::endl;
}


void trajectory_generator::run()
{
    int seq_num;
        
    if(command_input_interface.getCommand(in_msg,seq_num))
    {
        std::string command = in_msg.command;

        std::cout<<" - YARP: Command ["<<seq_num<<"] received: "<<command<<std::endl;
	
	if(command=="line")
	{
	    compute_line_trj(in_msg,out_msg);
	    command_output_interface.sendCommand(out_msg,seq_num);
	}
	if(command=="circle")
	{
	    std::cout<<"TODO"<<std::endl;
	}
    }
}

bool trajectory_generator::custom_pause()
{

}

bool trajectory_generator::custom_resume()
{

}

