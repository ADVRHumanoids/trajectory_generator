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
#include <../../opt/ros/indigo/include/ros/builtin_message_traits.h>

trajectory_generator::trajectory_generator()
{

}

KDL::Vector trajectory_generator::polynomial_interpolation(const polynomial_coefficients& poly,const KDL::Vector& vec_f, double time, double t_f)
{
    return poly.a()*vec_f/(2.0*pow(t_f,3))*pow(time,3) + (poly.b()*vec_f)/(2.0*pow(t_f,4))*pow(time,4) + (poly.c()*vec_f)/(2.0*pow(t_f,5))*pow(time,5);
}

double trajectory_generator::polynomial_interpolation(const polynomial_coefficients& poly, double value_f, double time, double t_f)
{
    return poly.a()*value_f/(2.0*pow(t_f,3))*pow(time,3) + (poly.b()*value_f)/(2.0*pow(t_f,4))*pow(time,4) + (poly.c()*value_f)/(2.0*pow(t_f,5))*pow(time,5);
}

bool trajectory_generator::line_initialize(double time, const KDL::Frame& start, const KDL::Frame& final)
{
    if(time<=0) return false;
    
    line_param.time = time;
    line_param.start = start;
    line_param.displacement.p = final.p-start.p;
    double r,p,y,r1,p1,y1;
    start.M.GetRPY(r,p,y);
    final.M.GetRPY(r1,p1,y1);
    line_param.displacement.M=KDL::Rotation::RPY(r1-r,p1-p,y1-y);
    // NOTE using quaternions use the line down here
//     line_param.displacement.M=final.M;
    line_param.initialized = true;
    return true;
}

bool trajectory_generator::line_trajectory(double t, KDL::Frame& pos_d, KDL::Twist& vel_d)
{
    if(!line_param.initialized) return false;
  
    polynomial_coefficients poly,vel_poly;
    vel_poly.set_polynomial_coeff(60.0, -120.0, 60);
    
    KDL::Vector temp_vector_r, start_vector_r, dis_vector_r;
    
    KDL::Vector temp_vel_vector_p, temp_vel_vector_r;

    double ro,pi,ya;
    line_param.start.M.GetRPY(ro,pi,ya);
    start_vector_r.data[0] = ro;
    start_vector_r.data[1] = pi;
    start_vector_r.data[2] = ya;
        
    double ro_,pi_,ya_;
    line_param.displacement.M.GetRPY(ro_,pi_,ya_);
    dis_vector_r.data[0] = ro_;
    dis_vector_r.data[1] = pi_;
    dis_vector_r.data[2] = ya_;
    
    //NOTE for quaternion SLERP
//     Eigen::Quaternion<double> q_start, q_end, q_interp;
//     line_param.start.M.GetQuaternion(q_start.x(),q_start.y(),q_start.z(),q_start.w());
//     line_param.displacement.M.GetQuaternion(q_end.x(),q_end.y(),q_end.z(),q_end.w());
    
    if(t >= 0.0 && t<=line_param.time)
    {
          pos_d.p = line_param.start.p + polynomial_interpolation(poly,line_param.displacement.p,t,line_param.time);
	  temp_vector_r = start_vector_r + polynomial_interpolation(poly,dis_vector_r,t,line_param.time);
	  
          temp_vel_vector_p = polynomial_interpolation(vel_poly,line_param.displacement.p,t,line_param.time);
	  temp_vel_vector_r = polynomial_interpolation(vel_poly,dis_vector_r,t,line_param.time);

	  pos_d.M = KDL::Rotation::RPY(temp_vector_r.data[0],temp_vector_r.data[1],temp_vector_r.data[2]);

// 	  q_interp = q_start.slerp(t,q_end);
// 	  pos_d.M = KDL::Rotation::Quaternion(q_interp.x(),q_interp.y(),q_interp.z(),q_interp.w());
	  
          vel_d.vel=temp_vel_vector_p;
	  vel_d.rot=temp_vel_vector_r;
    }
    else if (t > line_param.time)
    {
          pos_d.p = line_param.start.p + line_param.displacement.p;
	  pos_d.M = KDL::Rotation::RPY(start_vector_r.data[0] + dis_vector_r.data[0],start_vector_r.data[1] + dis_vector_r.data[1],start_vector_r.data[2] + dis_vector_r.data[2]);
// 	  pos_d.M = KDL::Rotation::Quaternion(q_end.x(),q_end.y(),q_end.z(),q_end.w());
	  
	  vel_d.vel= KDL::Vector::Zero();
	  vel_d.rot = KDL::Vector::Zero();
    }
    return true;
}

bool trajectory_generator::complete_line_trajectory(std::map< double, KDL::Frame >& pos_trj, std::map< double, KDL::Twist >& vel_trj, double delta_t)
{
	for(double t=0;t<=line_param.time;t=t+delta_t)
	{
	    if(!line_trajectory(t, pos_trj[t] , vel_trj[t])) return false;
	}
	return true;
}

bool trajectory_generator::circle_initialize(double time, double radius, double center_angle, const KDL::Frame& start, const KDL::Frame& object)
{
    if(time<=0) return false;
    
    circle_param.time=time;
    circle_param.start = start;
    circle_param.object = object;
    circle_param.radius = radius;
    circle_param.center_angle = center_angle;
    circle_param.initialized = true;
    
    return true;
}

bool trajectory_generator::circle_trajectory(double t, KDL::Frame& pos_d, KDL::Twist& vel_d)
{
    polynomial_coefficients poly,vel_poly;
    vel_poly.set_polynomial_coeff(60.0, -120.0, 60);
    
    double CircleAngle=0.0, DCircleAngle=0.0;

    KDL::Frame Object_Hand, Waist_ObjectRotating, Waist_HandRotating, Object_HandRotating;
    KDL::Frame Waist_Hand(circle_param.start);
    KDL::Frame Waist_Object(circle_param.object);
    Object_Hand = Waist_Object.Inverse()*Waist_Hand;
    
    if(t>=0.0 && t<=circle_param.time)
    {
        CircleAngle = polynomial_interpolation(poly,circle_param.center_angle,t,circle_param.time);
	DCircleAngle = polynomial_interpolation(vel_poly,circle_param.center_angle,t,circle_param.time); // NOT USED
    }
    else if (t > circle_param.time)
    {
        CircleAngle = circle_param.center_angle;
    }
    
    Object_HandRotating=KDL::Frame(KDL::Rotation::RotX(CircleAngle))*Object_Hand;
    Waist_HandRotating=Waist_Object*Object_HandRotating;
    pos_d = Waist_HandRotating;
    
    return true;
}

bool trajectory_generator::complete_circle_trajectory(std::map< double, KDL::Frame >& pos_trj, std::map< double, KDL::Twist >& vel_trj, double delta_t)
{
	for(double t=0;t<=circle_param.time;t=t+delta_t)
	{
	    if(!circle_trajectory(t, pos_trj[t] , vel_trj[t])) return false;
	}
	return true;
}

bool trajectory_generator::foot_initialize(double time, const KDL::Frame& start, const KDL::Frame& final, double height)
{
    if(time<=0) return false;
    
    foot_param.time=time;
    foot_param.start = start;
    foot_param.displacement.p = final.p - start.p;
    KDL::Vector ctrl1, ctrl2;
    ctrl1.x((start.p.x() + final.p.x())/4);
    ctrl1.y((start.p.y() + final.p.y())/4);
    ctrl1.z((start.p.z() + final.p.z())/4 + height);    
    ctrl2.x(3*(start.p.x() + final.p.x())/4);
    ctrl2.y(3*(start.p.y() + final.p.y())/4);
    ctrl2.z(3*(start.p.z() + final.p.z())/4 + height);
    foot_param.ctrl_points.push_back(start.p);
    foot_param.ctrl_points.push_back(ctrl1);
    foot_param.ctrl_points.push_back(ctrl2);
    foot_param.ctrl_points.push_back(final.p);
    foot_param.initialized = true;
    
    return true;
}

bool trajectory_generator::foot_trajectory(double t, KDL::Frame& pos_d)
{
    if(t >= 0.0 && t<=foot_param.time)
    {		
	pos_d.p.x(0.0);
	pos_d.p.y(0.0);
	pos_d.p.z(0.0);
	for (int j = 0; j < foot_param.ctrl_points.size(); j++)
	{
	    KDL::Vector p = foot_param.ctrl_points.at(j);
	    int n = foot_param.ctrl_points.size();
	    pos_d.p.x(pos_d.p.x()+ p.x() * (fact(n-1)/(fact(j)*fact(n-1 - j))*pow(t/foot_param.time,j)*pow(1-t/foot_param.time,n-1 -j)));
	    pos_d.p.y(pos_d.p.y()+ p.y() * (fact(n-1)/(fact(j)*fact(n-1 - j))*pow(t/foot_param.time,j)*pow(1-t/foot_param.time,n-1 -j)));
	    pos_d.p.z(pos_d.p.z()+ p.z() * (fact(n-1)/(fact(j)*fact(n-1 - j))*pow(t/foot_param.time,j)*pow(1-t/foot_param.time,n-1 -j)));
	}
	
	pos_d.M = KDL::Rotation::Identity();  
    }
    else if (t > foot_param.time)
    {
	pos_d.p = foot_param.start.p + foot_param.displacement.p;
	pos_d.M = KDL::Rotation::Identity();  	  
    }
    return true;
}

bool trajectory_generator::complete_foot_trajectory(std::map< double, KDL::Frame >& pos_trj, std::map< double, KDL::Twist >& vel_trj, double delta_t)
{

}

void trajectory_generator::custom_circle_initialize(double time, KDL::Frame start, const KDL::Frame& displacement, bool left, bool hand, double angle, double radius)
{
    custom_circle_param.center_angle = angle;
    custom_circle_param.displacement = displacement;
    custom_circle_param.start = start;
    custom_circle_param.hand = hand;
    custom_circle_param.left = left;
    custom_circle_param.radius = radius;
    custom_circle_param.time = time;
}

void trajectory_generator::custom_circle_trajectory(double t, KDL::Frame& pos_d)
{
    polynomial_coefficients poly;
    
    KDL::Frame temp_frame;
    KDL::Vector temp_vector_p, start_vector_p, dis_vector_p;
    KDL::Vector temp_vector_r, start_vector_r, dis_vector_r;
    
    double circle_angle;
    double Xf1 = custom_circle_param.center_angle;
    double radius = custom_circle_param.radius;

    start_vector_p.data[0] = custom_circle_param.start.p.x();
    start_vector_p.data[1] = custom_circle_param.start.p.y();
    start_vector_p.data[2] = custom_circle_param.start.p.z();
    double ro,pi,ya;
    custom_circle_param.start.M.GetRPY(ro,pi,ya);
    start_vector_r.data[0] = ro;
    start_vector_r.data[1] = pi;
    start_vector_r.data[2] = ya;
    
//     std::cout<<"TRJ: from ("<<circle_param.start.p.x()<<' '<<circle_param.start.p.y()<<' '<<circle_param.start.p.z()<<' '<<ro<<' '<<pi<<' '<<ya<<") ";
    
    dis_vector_p.data[0] = custom_circle_param.displacement.p.x();
    dis_vector_p.data[1] = custom_circle_param.displacement.p.y();
    dis_vector_p.data[2] = custom_circle_param.displacement.p.z();
    double ro_,pi_,ya_;
    custom_circle_param.displacement.M.GetRPY(ro_,pi_,ya_);
    dis_vector_r.data[0] = ro_;
    dis_vector_r.data[1] = pi_;
    dis_vector_r.data[2] = ya_;

//     std::cout<<"to ("<<circle_param.displacement.p.x()<<' '<<circle_param.displacement.p.y()<<' '<<circle_param.displacement.p.z()<<' '<<ro_<<' '<<pi_<<' '<<ya_<<") in "<<circle_param.time<<" [s], R: "<<circle_param.radius<<
//                " [m], angle: "<<circle_param.center_angle<<" [rad], left: "<<circle_param.left<<std::endl;
    
//     std::cout<<"(circle ";
	       
    double mean_x = (2*start_vector_p.data[0]+dis_vector_p.data[0])/2.0;
    if((!custom_circle_param.hand) && (mean_x < 0)) Xf1=-Xf1;
    double mean_y = (2*start_vector_p.data[1]+dis_vector_p.data[1])/2.0;
    double mean_z = (2*start_vector_p.data[2]+dis_vector_p.data[2])/2.0;

    int index_1 = (2*(custom_circle_param.hand)+(!custom_circle_param.hand));
    
    int index_2 = (2*(!custom_circle_param.hand)+custom_circle_param.hand);
    
    if(t>=0.0 && t<=custom_circle_param.time)
    {
	circle_angle = polynomial_interpolation(poly,Xf1,t,custom_circle_param.time);
	circle_angle = circle_angle - atan2(dis_vector_p.data[index_2],dis_vector_p.data[0]);

	temp_vector_p.data[index_1] = start_vector_p.data[index_1] + polynomial_interpolation(poly,dis_vector_p.data[index_1],t,circle_param.time);
	
	temp_vector_r = start_vector_r + polynomial_interpolation(poly,dis_vector_r,t,custom_circle_param.time);
	
        if(!custom_circle_param.left) //RIGHT
	{
	    temp_vector_p.data[0]=-radius*cos(circle_angle)+mean_x;
	    temp_vector_p.data[index_2]=radius*sin(circle_angle)+custom_circle_param.hand*mean_y+(!custom_circle_param.hand)*mean_z;
        }
        else //LEFT
	{
	    temp_vector_p.data[0]=-radius*cos(circle_angle)+mean_x;
	    temp_vector_p.data[index_2]=radius*sin(circle_angle)+custom_circle_param.hand*mean_y+(!custom_circle_param.hand)*mean_z;
        }
        
	temp_frame.p.x(temp_vector_p.data[0]);
	temp_frame.p.y(temp_vector_p.data[1]);
	temp_frame.p.z(temp_vector_p.data[2]);
	temp_frame.M = KDL::Rotation::RPY(temp_vector_r.data[0],temp_vector_r.data[1],temp_vector_r.data[2]);

	pos_d = temp_frame;
    }
    else if (t > circle_param.time)
    {
        circle_angle = circle_param.center_angle;
	circle_angle = circle_angle - atan2(dis_vector_p.data[index_2],dis_vector_p.data[0]);

	temp_vector_p.data[index_1] = start_vector_p.data[index_1] + polynomial_interpolation(poly,dis_vector_p.data[index_1],t,circle_param.time);

	temp_vector_r = start_vector_r + polynomial_interpolation(poly,dis_vector_r,t,custom_circle_param.time);

        if(!custom_circle_param.left) //RIGHT
	{
	    temp_vector_p.data[0]=-radius*cos(circle_angle)+mean_x;
	    temp_vector_p.data[index_2]=radius*sin(circle_angle)+custom_circle_param.hand*mean_y+(!custom_circle_param.hand)*mean_z;
        }
        else //LEFT
	{
	    temp_vector_p.data[0]=-radius*cos(circle_angle)+mean_x;
	    temp_vector_p.data[index_2]=radius*sin(circle_angle)+custom_circle_param.hand*mean_y+(!custom_circle_param.hand)*mean_z;
        }

	temp_frame.p.x(temp_vector_p.data[0]);
	temp_frame.p.y(temp_vector_p.data[1]);
	temp_frame.p.z(temp_vector_p.data[2]);
	temp_frame.M = KDL::Rotation::RPY(temp_vector_r.data[0],temp_vector_r.data[1],temp_vector_r.data[2]);

	pos_d = temp_frame;
    }
}

void trajectory_generator::complete_custom_circle_trajectory(std::map< double, KDL::Frame >& pos_trj, double delta_t)
{
    for(double t=0;t<=custom_circle_param.time;t=t+0.01)
    {
	custom_circle_trajectory(t,pos_trj[t]);
    }
}

// ************************************************************************************
// 				BEZIER CURVE METHODS
// ************************************************************************************

void trajectory_generator::bezier_initialize(double time, KDL::Frame start, KDL::Frame end)
{
    bezier_param.time = time;
    bezier_param.start = start;
    bezier_param.end = end;
}

void trajectory_generator::bezier_trajectory(std::map<double,KDL::Frame>& trj)
{
    trj.clear();
  
    polynomial_coefficients poly;
    
    KDL::Frame temp_frame;
    KDL::Vector temp_vector_p, start_vector_p, end_vector_p;
    KDL::Vector temp_vector_r, start_vector_r, end_vector_r;

    start_vector_p.data[0] = bezier_param.start.p.x();
    start_vector_p.data[1] = bezier_param.start.p.y();
    start_vector_p.data[2] = bezier_param.start.p.z();
    double ro,pi,ya;
    bezier_param.start.M.GetRPY(ro,pi,ya);
    start_vector_r.data[0] = ro;
    start_vector_r.data[1] = pi;
    start_vector_r.data[2] = ya;
    
    end_vector_p.data[0] = bezier_param.end.p.x();
    end_vector_p.data[1] = bezier_param.end.p.y();
    end_vector_p.data[2] = bezier_param.end.p.z();
    double ro_,pi_,ya_;
    bezier_param.end.M.GetRPY(ro_,pi_,ya_);
    end_vector_r.data[0] = ro_;
    end_vector_r.data[1] = pi_;
    end_vector_r.data[2] = ya_;
    
    bezier_param.bz_fun = new bezier_curve(bezier_param.start.p,bezier_param.end.p);
    // re-implementation of the run method    
    while (1) {
      if (bezier_param.bz_fun->octree != NULL){
	while (trajInCollision())
	{
	    std::cout<<"!!! Bezier curve in collision !!!"<<std::endl; 	  
	    computeNeighborsCOM(); // verify obstacle type in the collision neighborhood and return obstacle type	
	    avoidObstacle();
	}
	std::cout<<"Found Bezier curve not in collision"<<std::endl;
	
	int i=0;
	// Use minimum jerk polynomials for rotation
	for(double t=0;t<=bezier_param.time;t=t+0.01)
	{
	    temp_vector_p = bezier_param.bz_fun->curve->at(i);
	
	temp_vector_r = polynomial_interpolation(poly,end_vector_r,t,bezier_param.time);
	
	    temp_frame.p.x(temp_vector_p.data[0]);
	    temp_frame.p.y(temp_vector_p.data[1]);
	    temp_frame.p.z(temp_vector_p.data[2]);
	    temp_frame.M = KDL::Rotation::RPY(temp_vector_r.data[0],temp_vector_r.data[1],temp_vector_r.data[2]);
	
	    trj[t] = temp_frame;
	    
	    std::cout<<"Computed trajectory: "<<trj[t].p.x()<<" "<<trj[t].p.y()<<" "<<trj[t].p.z()<<std::endl;
	    i++;
	}
	break;
      }
    else	
	 std::cout<<"No Octomap found"<<std::endl;
    
    ros::spinOnce();
    sleep(1);
    }
        
}

bool trajectory_generator::trajInCollision()
{
    computeBezierCurve();
    return bezier_param.bz_fun->inCollisionOctomap();
}

void trajectory_generator::computeNeighborsCOM()
{
    bezier_param.bz_fun->computeNeighborsCOM();
}

void trajectory_generator::avoidObstacle()
{
    bezier_param.bz_fun->avoidObstacle(); // move the control points based on obstacle type
}

void trajectory_generator::computeBezierCurve()
{
    std::cout<<"Computing new Bezier curve..."<<std::endl;
    bezier_param.bz_fun->curve->clear();
    for(double t=0;t<=bezier_param.time;t=t+0.01)
    {
	bezier_param.bz_fun->computeBezier(t);
	bezier_param.bz_fun->curve->push_back(bezier_param.bz_fun->q);
    }
    
}

int trajectory_generator::fact(int x)
{
    if (x <= 1) return 1; else return x*fact(x - 1);
}