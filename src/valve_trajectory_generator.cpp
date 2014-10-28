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

#include <trajectory_generator/trajectory_generator.h>

bool trajectory_generator::valve_line_initialize(double time, KDL::Frame& start, KDL::Frame& displacement)
{
    if(time<=0) return false;
    
    valve_line_param.time = time;
    valve_line_param.start = start;
    valve_line_param.displacement = displacement;
    valve_line_param.initialized = true;
    return true;
}

bool trajectory_generator::valve_line_trajectory(double t, KDL::Frame& pos_d, KDL::Frame& vel_d )
{
    if(!valve_line_param.initialized) return false;
  
    polynomial_coefficients poly,vel_poly;
    vel_poly.set_polynomial_coeff(60.0, -120.0, 60);
    
    KDL::Vector temp_vector_p, start_vector_p, dis_vector_p;
    KDL::Vector temp_vector_r, start_vector_r, dis_vector_r;
    
    KDL::Vector temp_vel_vector_p, temp_vel_vector_r;

    start_vector_p.data[0] = valve_line_param.start.p.x();
    start_vector_p.data[1] = valve_line_param.start.p.y();
    start_vector_p.data[2] = valve_line_param.start.p.z();
    double ro,pi,ya;
    valve_line_param.start.M.GetRPY(ro,pi,ya);
    start_vector_r.data[0] = ro;
    start_vector_r.data[1] = pi;
    start_vector_r.data[2] = ya;
    
    dis_vector_p.data[0] = valve_line_param.displacement.p.x();
    dis_vector_p.data[1] = valve_line_param.displacement.p.y();
    dis_vector_p.data[2] = valve_line_param.displacement.p.z();
    double ro_,pi_,ya_;
    valve_line_param.displacement.M.GetRPY(ro_,pi_,ya_);
    dis_vector_r.data[0] = ro_;
    dis_vector_r.data[1] = pi_;
    dis_vector_r.data[2] = ya_;
    
    if(t >= 0.0 && t<=valve_line_param.time)
    {
	  temp_vector_p = start_vector_p + polynomial_interpolation(poly,dis_vector_p,t,valve_line_param.time);
	  temp_vector_r = start_vector_r + polynomial_interpolation(poly,dis_vector_r,t,valve_line_param.time);
	  
	  temp_vel_vector_p = polynomial_interpolation(vel_poly,dis_vector_p,t,valve_line_param.time);
	  temp_vel_vector_r = polynomial_interpolation(vel_poly,dis_vector_r,t,valve_line_param.time);
	  
	  pos_d.p.x(temp_vector_p.data[0]);
	  pos_d.p.y(temp_vector_p.data[1]);
	  pos_d.p.z(temp_vector_p.data[2]);
	  pos_d.M = KDL::Rotation::RPY(temp_vector_r.data[0],temp_vector_r.data[1],temp_vector_r.data[2]);
	  	  
	  vel_d.p.x(temp_vel_vector_p.data[0]);
	  vel_d.p.y(temp_vel_vector_p.data[1]);
	  vel_d.p.z(temp_vel_vector_p.data[2]);
	  vel_d.M = KDL::Rotation::RPY(temp_vel_vector_r.data[0],temp_vel_vector_r.data[1],temp_vel_vector_r.data[2]);
    }
    else if (t > valve_line_param.time)
    {
	  pos_d.p = start_vector_p + dis_vector_p;
	  pos_d.M = KDL::Rotation::RPY(start_vector_r.data[0] + dis_vector_r.data[0],start_vector_r.data[1] + dis_vector_r.data[1],start_vector_r.data[2] + dis_vector_r.data[2]);
	  
	  vel_d.p= KDL::Vector::Zero();
	  vel_d.M = KDL::Rotation::Identity();
    }
    
    return true;
}

bool trajectory_generator::valve_circle_initialize(double time, double radius, double center_angle, KDL::Frame& start)
{
    if(time<=0) return false;
    
    valve_circle_param.time=time;
    valve_circle_param.start = start;
    valve_circle_param.radius = radius;
    valve_circle_param.center_angle = center_angle;
    valve_circle_param.initialized = true;
    
    return true;
}

double trajectory_generator::valve_circle_trajectory(double t, bool left_arm, KDL::Rotation& ROTe, KDL::Frame& pos_d, KDL::Frame& vel_d)
{
    if (!valve_circle_param.initialized) return 0;
    
    polynomial_coefficients poly,vel_poly;
    vel_poly.set_polynomial_coeff(60.0, -120.0, 60);
    
    double Xf1;
    
    KDL::Vector Xd_v_p, dXd_v_p;
    
    Xd_v_p = KDL::Vector::Zero();
    dXd_v_p = KDL::Vector::Zero();
    
    double CircleAngle=0.0, DCircleAngle=0.0;
    Xf1 = valve_circle_param.center_angle;

    if(t>=0.0 && t<=valve_circle_param.time)
    {
        CircleAngle = polynomial_interpolation(poly,Xf1,t,valve_circle_param.time);
        DCircleAngle= polynomial_interpolation(vel_poly,Xf1,t,valve_circle_param.time);

        if(!left_arm)
	{
	    //local X axis (handle)
	    Xd_v_p.data[0] = -valve_circle_param.radius*cos(CircleAngle) ;
	    dXd_v_p.data[0] = valve_circle_param.radius*sin(CircleAngle)*DCircleAngle;
	    
	    //local Y axis (handle)
	    Xd_v_p.data[1]  = valve_circle_param.radius*(sin(CircleAngle));
	    dXd_v_p.data[1]= valve_circle_param.radius*cos(CircleAngle)*DCircleAngle;
        }
	else
	{
	    //local X axis (handle)
	    Xd_v_p.data[0] =  -valve_circle_param.radius*cos(CircleAngle) ;
	    dXd_v_p.data[0] = valve_circle_param.radius*sin(CircleAngle)*DCircleAngle;

	    //local Y axis (handle)
	    Xd_v_p.data[1] = -valve_circle_param.radius*(sin(CircleAngle));
	    dXd_v_p.data[1] = -valve_circle_param.radius*cos(CircleAngle)*DCircleAngle;
        }

        Xd_v_p.data[2] = 0.0; dXd_v_p.data[2] = 0.0;

        //Transformation Xd_v to Xd
        //position
	pos_d.p = valve_circle_param.start.p + ROTe*Xd_v_p;

    }
    else if (t > valve_circle_param.time)
    {

        CircleAngle = Xf1;

        if(!left_arm)
	{
	    //local X axis (handle)
	    Xd_v_p.data[0] =  -valve_circle_param.radius*cos(CircleAngle);

	    //local Y axis (handle)
	    Xd_v_p.data[1] = valve_circle_param.radius*(sin(CircleAngle));
        }
	else
	{
	    //local X axis (handle)
	    Xd_v_p.data[0] =  -valve_circle_param.radius*cos(CircleAngle) ;

	    //local Y axis (handle)
	    Xd_v_p.data[1] = -valve_circle_param.radius*(sin(CircleAngle));
        }

	Xd_v_p.data[2] = 0.0;

	pos_d.p = valve_circle_param.start.p + ROTe*Xd_v_p;
    }

    return CircleAngle;
}

bool trajectory_generator::valve_turn_initialize(double time, double radius, double center_angle, KDL::Frame& start)
{
    if(time<=0) return false;
    
    valve_turn_param.time=time;
    valve_turn_param.start = start;
    valve_turn_param.radius = radius;
    valve_turn_param.center_angle = center_angle;
    valve_turn_param.initialized = true;
    
    return true;
}

double trajectory_generator::valve_turn_trajectory(double t, KDL::Rotation& ROTv, KDL::Frame& pos_d, KDL::Frame& vel_d)
{
    double Xf1;
    
    polynomial_coefficients poly,vel_poly;
    vel_poly.set_polynomial_coeff(60.0, -120.0, 60);
    
    KDL::Vector Xd_v_p, dXd_v_p;
    
    Xd_v_p = KDL::Vector::Zero();
    dXd_v_p = KDL::Vector::Zero();
    
    double CircleAngle=0.0, DCircleAngle=0.0;
    Xf1 = valve_turn_param.center_angle;

    if(t>=0.0 && t<=valve_turn_param.time)
    {
        CircleAngle = -25*M_PI/180 + polynomial_interpolation(poly,Xf1,t,valve_turn_param.time);
        DCircleAngle = polynomial_interpolation(vel_poly,Xf1,t,valve_circle_param.time);

        //Xv axis
        Xd_v_p.data[0] =  valve_turn_param.radius*cos(M_PI/2.0- CircleAngle) ;
        dXd_v_p.data[0] = -valve_turn_param.radius*sin(M_PI/2.0- CircleAngle)*DCircleAngle;

        //Yv axis
        Xd_v_p.data[1] = valve_turn_param.radius*(sin(M_PI/2.0- CircleAngle));
        dXd_v_p.data[1] = valve_turn_param.radius*cos(M_PI/2.0- CircleAngle)*DCircleAngle;

        //Zv axis
        Xd_v_p.data[2] = 0.0; dXd_v_p.data[2] = 0.0;

        //Transformation Xd_v to Xd
        //position
	pos_d.p = valve_circle_param.start.p + ROTv*Xd_v_p;
    }
    else if (t > valve_circle_param.time)
    {
        CircleAngle = -25*M_PI/180 + Xf1;

        //Xv axis
        Xd_v_p.data[0] =  valve_turn_param.radius*cos(M_PI/2.0- CircleAngle) ;
       //Yv axis
        Xd_v_p.data[1] = valve_turn_param.radius*(sin(M_PI/2.0- CircleAngle));

        Xd_v_p.data[2] = 0.0;

	pos_d.p = valve_circle_param.start.p + ROTv*Xd_v_p;

    }

    return CircleAngle;
}