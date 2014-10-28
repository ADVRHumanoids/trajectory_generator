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

trajectory_generator::trajectory_generator()
{

}

KDL::Vector trajectory_generator::polynomial_interpolation(polynomial_coefficients& poly, KDL::Vector& vec_f, double time, double t_f)
{
    return poly.a()*vec_f/(2.0*pow(t_f,3))*pow(time,3) + (poly.b()*vec_f)/(2.0*pow(t_f,4))*pow(time,4) + (poly.c()*vec_f)/(2.0*pow(t_f,5))*pow(time,5);
}

double trajectory_generator::polynomial_interpolation(polynomial_coefficients& poly, double value_f, double time, double t_f)
{
    return poly.a()*value_f/(2.0*pow(t_f,3))*pow(time,3) + (poly.b()*value_f)/(2.0*pow(t_f,4))*pow(time,4) + (poly.c()*value_f)/(2.0*pow(t_f,5))*pow(time,5);
}

void trajectory_generator::set_line_time(double time)
{
    line_param.time = time;
}

void trajectory_generator::set_line_start(KDL::Frame start)
{
    line_param.start = start;
}

void trajectory_generator::set_line_displacement(KDL::Frame displacement)
{
    line_param.displacement = displacement;
}

void trajectory_generator::line_trajectory(std::map<double,KDL::Frame>& trj)
{
    trj.clear();
    
    polynomial_coefficients poly;
  
    KDL::Frame temp_frame;
    KDL::Vector temp_vector_p, start_vector_p, dis_vector_p;
    KDL::Vector temp_vector_r, start_vector_r, dis_vector_r;

    start_vector_p.data[0] = line_param.start.p.x();
    start_vector_p.data[1] = line_param.start.p.y();
    start_vector_p.data[2] = line_param.start.p.z();
    double ro,pi,ya;
    line_param.start.M.GetRPY(ro,pi,ya);
    start_vector_r.data[0] = ro;
    start_vector_r.data[1] = pi;
    start_vector_r.data[2] = ya;

//     std::cout<<"TRJ: from ("<<line_param.start.p.x()<<' '<<line_param.start.p.y()<<' '<<line_param.start.p.z()<<' '<<ro<<' '<<pi<<' '<<ya<<") ";
    
    dis_vector_p.data[0] = line_param.displacement.p.x();
    dis_vector_p.data[1] = line_param.displacement.p.y();
    dis_vector_p.data[2] = line_param.displacement.p.z();
    double ro_,pi_,ya_;
    line_param.displacement.M.GetRPY(ro_,pi_,ya_);
    dis_vector_r.data[0] = ro_;
    dis_vector_r.data[1] = pi_;
    dis_vector_r.data[2] = ya_;

//     std::cout<<"to ("<<line_param.displacement.p.x()<<' '<<line_param.displacement.p.y()<<' '<<line_param.displacement.p.z()<<' '<<ro_<<' '<<pi_<<' '<<ya_<<") in "<<line_param.time<<" [s]"<<std::endl;
    
//     std::cout<<"(line ";
    
    for(double t=0;t<=line_param.time;t=t+0.01)
    {
	  temp_vector_p = start_vector_p + polynomial_interpolation(poly,dis_vector_p,t,line_param.time);
	  temp_vector_r = start_vector_r + polynomial_interpolation(poly,dis_vector_r,t,line_param.time);
	  
	  temp_frame.p.x(temp_vector_p.data[0]);
	  temp_frame.p.y(temp_vector_p.data[1]);
	  temp_frame.p.z(temp_vector_p.data[2]);
	  temp_frame.M = KDL::Rotation::RPY(temp_vector_r.data[0],temp_vector_r.data[1],temp_vector_r.data[2]);
	  
	  trj[t] = temp_frame;
// 	  std::cout<<t<<' '<<temp_vector_p.data[0]<<' '<<temp_vector_p.data[1]<<' '<<temp_vector_p.data[2]<<' '<<temp_vector_r.data[0]<<' '<<temp_vector_r.data[1]<<' '<<temp_vector_r.data[2]<<' ';
	  
    }
        
//     std::cout<<')'<<std::endl;
}

void trajectory_generator::set_circle_center_angle(double angle)
{
    circle_param.center_angle = angle;
}

void trajectory_generator::set_circle_displacement(KDL::Frame displacement)
{
    circle_param.displacement = displacement;
}

void trajectory_generator::set_circle_start(KDL::Frame start)
{
    circle_param.start = start;
}

void trajectory_generator::set_circle_hand_ee(bool hand)
{
    circle_param.hand = hand;
}

void trajectory_generator::set_circle_left_ee(bool left)
{
    circle_param.left = left;
}

void trajectory_generator::set_circle_radius(double radius)
{
    circle_param.radius = radius;
}

void trajectory_generator::set_circle_time(double time)
{
    circle_param.time = time;
}

void trajectory_generator::circle_trajectory(std::map<double,KDL::Frame>& trj)
{
    trj.clear();
  
    polynomial_coefficients poly;
    
    KDL::Frame temp_frame;
    KDL::Vector temp_vector_p, start_vector_p, dis_vector_p;
    KDL::Vector temp_vector_r, start_vector_r, dis_vector_r;
    
    double circle_angle;
    double Xf1 = circle_param.center_angle;
    double radius = circle_param.radius;

    start_vector_p.data[0] = circle_param.start.p.x();
    start_vector_p.data[1] = circle_param.start.p.y();
    start_vector_p.data[2] = circle_param.start.p.z();
    double ro,pi,ya;
    circle_param.start.M.GetRPY(ro,pi,ya);
    start_vector_r.data[0] = ro;
    start_vector_r.data[1] = pi;
    start_vector_r.data[2] = ya;
    
//     std::cout<<"TRJ: from ("<<circle_param.start.p.x()<<' '<<circle_param.start.p.y()<<' '<<circle_param.start.p.z()<<' '<<ro<<' '<<pi<<' '<<ya<<") ";
    
    dis_vector_p.data[0] = circle_param.displacement.p.x();
    dis_vector_p.data[1] = circle_param.displacement.p.y();
    dis_vector_p.data[2] = circle_param.displacement.p.z();
    double ro_,pi_,ya_;
    circle_param.displacement.M.GetRPY(ro_,pi_,ya_);
    dis_vector_r.data[0] = ro_;
    dis_vector_r.data[1] = pi_;
    dis_vector_r.data[2] = ya_;

//     std::cout<<"to ("<<circle_param.displacement.p.x()<<' '<<circle_param.displacement.p.y()<<' '<<circle_param.displacement.p.z()<<' '<<ro_<<' '<<pi_<<' '<<ya_<<") in "<<circle_param.time<<" [s], R: "<<circle_param.radius<<
//                " [m], angle: "<<circle_param.center_angle<<" [rad], left: "<<circle_param.left<<std::endl;
    
//     std::cout<<"(circle ";
	       
    double mean_x = (2*start_vector_p.data[0]+dis_vector_p.data[0])/2.0;
    if((!circle_param.hand) && (mean_x < 0)) Xf1=-Xf1;
    double mean_y = (2*start_vector_p.data[1]+dis_vector_p.data[1])/2.0;
    double mean_z = (2*start_vector_p.data[2]+dis_vector_p.data[2])/2.0;

    int index_1 = (2*(circle_param.hand)+(!circle_param.hand));
    
    int index_2 = (2*(!circle_param.hand)+circle_param.hand);
    
    for(double t=0;t<=circle_param.time;t=t+0.01)
    {
	circle_angle = polynomial_interpolation(poly,Xf1,t,circle_param.time);
	circle_angle = circle_angle - atan2(dis_vector_p.data[index_2],dis_vector_p.data[0]);

	temp_vector_p.data[index_1] = start_vector_p.data[index_1] + polynomial_interpolation(poly,dis_vector_p.data[index_1],t,circle_param.time);
	
	temp_vector_r = start_vector_r + polynomial_interpolation(poly,dis_vector_r,t,circle_param.time);
	
        if(!circle_param.left) //RIGHT
	{
	    //Xd_v(0) = -Radius*cos(CircleAngle) ;
	    //Xd_v(1) = Radius*(sin(CircleAngle));
	    
	    temp_vector_p.data[0]=-radius*cos(circle_angle)+mean_x;
	    temp_vector_p.data[index_2]=radius*sin(circle_angle)+circle_param.hand*mean_y+(!circle_param.hand)*mean_z;
        }
        else //LEFT
	{
	    //Xd_v(0) = -Radius*cos(CircleAngle) ;
	    //Xd_v(1) = -Radius*(sin(CircleAngle));

	    temp_vector_p.data[0]=-radius*cos(circle_angle)+mean_x;
	    temp_vector_p.data[index_2]=radius*sin(circle_angle)+circle_param.hand*mean_y+(!circle_param.hand)*mean_z;
        }
        
        //Xd_v(2) = 0.0;
//         temp_vector_p.data[2] = 0.0 + mean_z;

        //Xd.rows(0,2) = Xinit.rows(0,2) + ROTe*Xd_v.rows(0,2);	
	
	temp_frame.p.x(temp_vector_p.data[0]);
	temp_frame.p.y(temp_vector_p.data[1]);
	temp_frame.p.z(temp_vector_p.data[2]);
	temp_frame.M = KDL::Rotation::RPY(temp_vector_r.data[0],temp_vector_r.data[1],temp_vector_r.data[2]);

	trj[t] = temp_frame;
	//std::cout<<t<<' '<<temp_vector_p.data[0]<<' '<<temp_vector_p.data[1]<<' '<<temp_vector_p.data[2]<<' '<<temp_vector_r.data[0]<<' '<<temp_vector_r.data[1]<<' '<<temp_vector_r.data[2]<<' ';


    }    
    //std::cout<<')'<<std::endl;
}



// ************************************************************************************
// 				BEZIER CURVE METHODS
// ************************************************************************************

void trajectory_generator::set_bezier_time(double time)
{
    bezier_param.time = time;
}

void trajectory_generator::set_bezier_start(KDL::Frame start)
{
    bezier_param.start = start;
}

void trajectory_generator::set_bezier_end(KDL::Frame end)
{
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
    
    bezier_param.bz_fun = new bezier_curve(&n,bezier_param.start.p,bezier_param.end.p);
    // re-implementation of the run method    
    while (trajInCollision())
    {
	std::cout<<"!!! Bezier curve in collision !!!"<<std::endl; 	  
	computeNeighborsCOM(); // verify obstacle type in the collision neighborhood and return obstacle type	
	avoidObstacle();
    }
    std::cout<<"Found Bezier curve not in collision"<<std::endl;
    
    // Use minimum jerk polynomials for rotation
    for(double t=0;t<=bezier_param.time;t=t+0.01)
    {
	temp_vector_p = bezier_param.bz_fun->curve->at(t);
    
	temp_vector_r = polynomial_interpolation(poly,end_vector_r,t,bezier_param.time);
    
	temp_frame.p.x(temp_vector_p.data[0]);
	temp_frame.p.y(temp_vector_p.data[1]);
	temp_frame.p.z(temp_vector_p.data[2]);
	temp_frame.M = KDL::Rotation::RPY(temp_vector_r.data[0],temp_vector_r.data[1],temp_vector_r.data[2]);
    
	trj[t] = temp_frame;
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
	bezier_param.bz_fun->computeBezier(bezier_param.bz_fun->ctrl_points,bezier_param.bz_fun->curve,t);
	bezier_param.bz_fun->curve->push_back(bezier_param.bz_fun->q);
    }
    
}
