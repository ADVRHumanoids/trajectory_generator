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
#include <tf_conversions/tf_kdl.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Eigen>

bool trajectory_generator::valve_line_initialize(double time, const KDL::Frame& start, const KDL::Frame& final)
{
    if(time<=0) return false;
    
    valve_line_param.time = time;
    valve_line_param.start = start;
    valve_line_param.displacement.p = final.p-start.p;
    double r,p,y,r1,p1,y1;
    start.M.GetRPY(r,p,y);
    final.M.GetRPY(r1,p1,y1);
    valve_line_param.displacement.M=KDL::Rotation::RPY(r1-r,p1-p,y1-y);
    // NOTE using quaternions use the line down here
//     valve_line_param.displacement.M=final.M;
    valve_line_param.initialized = true;
    return true;
}

bool trajectory_generator::valve_line_trajectory(double t, KDL::Frame& pos_d, KDL::Twist& vel_d )
{
    if(!valve_line_param.initialized) return false;
  
    polynomial_coefficients poly,vel_poly;
    vel_poly.set_polynomial_coeff(60.0, -120.0, 60);
    
    KDL::Vector temp_vector_r, start_vector_r, dis_vector_r;
    
    KDL::Vector temp_vel_vector_p, temp_vel_vector_r;

    double ro,pi,ya;
    valve_line_param.start.M.GetRPY(ro,pi,ya);
    start_vector_r.data[0] = ro;
    start_vector_r.data[1] = pi;
    start_vector_r.data[2] = ya;
        
    double ro_,pi_,ya_;
    valve_line_param.displacement.M.GetRPY(ro_,pi_,ya_);
    dis_vector_r.data[0] = ro_;
    dis_vector_r.data[1] = pi_;
    dis_vector_r.data[2] = ya_;
    
    //NOTE for quaternion SLERP
//     Eigen::Quaternion<double> q_start, q_end, q_interp;
//     valve_line_param.start.M.GetQuaternion(q_start.x(),q_start.y(),q_start.z(),q_start.w());
//     valve_line_param.displacement.M.GetQuaternion(q_end.x(),q_end.y(),q_end.z(),q_end.w());
    
    if(t >= 0.0 && t<=valve_line_param.time)
    {
          pos_d.p = valve_line_param.start.p + polynomial_interpolation(poly,valve_line_param.displacement.p,t,valve_line_param.time);
	  temp_vector_r = start_vector_r + polynomial_interpolation(poly,dis_vector_r,t,valve_line_param.time);
	  
          temp_vel_vector_p = polynomial_interpolation(vel_poly,valve_line_param.displacement.p,t,valve_line_param.time);
	  temp_vel_vector_r = polynomial_interpolation(vel_poly,dis_vector_r,t,valve_line_param.time);

	  pos_d.M = KDL::Rotation::RPY(temp_vector_r.data[0],temp_vector_r.data[1],temp_vector_r.data[2]);

// 	  q_interp = q_start.slerp(t,q_end);
// 	  pos_d.M = KDL::Rotation::Quaternion(q_interp.x(),q_interp.y(),q_interp.z(),q_interp.w());
	  
          vel_d.vel=temp_vel_vector_p;
	  vel_d.rot=temp_vel_vector_r;
    }
    else if (t > valve_line_param.time)
    {
          pos_d.p = valve_line_param.start.p + valve_line_param.displacement.p;
	  pos_d.M = KDL::Rotation::RPY(start_vector_r.data[0] + dis_vector_r.data[0],start_vector_r.data[1] + dis_vector_r.data[1],start_vector_r.data[2] + dis_vector_r.data[2]);
// 	  pos_d.M = KDL::Rotation::Quaternion(q_end.x(),q_end.y(),q_end.z(),q_end.w());
	  
	  vel_d.vel= KDL::Vector::Zero();
	  vel_d.rot = KDL::Vector::Zero();

    }
    return true;
}

bool trajectory_generator::valve_turn_initialize(double time, double radius, double center_angle,const KDL::Frame& start, const KDL::Frame& valve)
{
    if(time<=0) return false;
    
    valve_turn_param.time=time;
    valve_turn_param.start = start;
    valve_turn_param.valve = valve;
    valve_turn_param.radius = radius;
    valve_turn_param.center_angle = center_angle;
    valve_turn_param.initialized = true;
    
    return true;
}

bool trajectory_generator::valve_turn_trajectory(double t, KDL::Frame& pos_d, KDL::Twist& vel_d)
{
    polynomial_coefficients poly,vel_poly;
    vel_poly.set_polynomial_coeff(60.0, -120.0, 60);
    
    double CircleAngle=0.0, DCircleAngle=0.0;

    KDL::Frame Valve_Hand, Waist_ValveRotating, Waist_HandRotating, Valve_HandRotating;
    KDL::Frame Waist_Hand(valve_turn_param.start);
    KDL::Frame Waist_Valve(valve_turn_param.valve);
    Valve_Hand = Waist_Valve.Inverse()*Waist_Hand;
    
    if(t>=0.0 && t<=valve_turn_param.time)
    {
        CircleAngle = polynomial_interpolation(poly,valve_turn_param.center_angle,t,valve_turn_param.time);
	DCircleAngle = polynomial_interpolation(vel_poly,valve_turn_param.center_angle,t,valve_turn_param.time); // NOT USED
    }
    else if (t > valve_turn_param.time)
    {
        CircleAngle = valve_turn_param.center_angle;
    }
    
    Valve_HandRotating=KDL::Frame(KDL::Rotation::RotX(CircleAngle))*Valve_Hand;
    Waist_HandRotating=Waist_Valve*Valve_HandRotating;
    pos_d = Waist_HandRotating;
    
    return true;
}

bool trajectory_generator::valve_circle_initialize(double time, double radius, double center_angle, const KDL::Frame& start)
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
