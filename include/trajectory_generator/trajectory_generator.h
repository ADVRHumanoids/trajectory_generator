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

#ifndef TRJ_GEN_H
#define TRJ_GEN_H

#include <string>
#include <map>
#include <kdl/frames.hpp>

#include <geometry_msgs/Point.h>
#include "bezier_curve.h"
    
class line_parameters
{
public:
    line_parameters()
    {
	time=1;
	start.p = KDL::Vector::Zero();
	start.M = KDL::Rotation::Identity();
	displacement.p = KDL::Vector::Zero();
	displacement.M = KDL::Rotation::Identity();
    }
    
    double time;
    KDL::Frame start;
    KDL::Frame displacement;  
};

class circle_parameters
{
public:
    circle_parameters()
    {
	time=1;
	start.p = KDL::Vector::Zero();
	start.M = KDL::Rotation::Identity();
	displacement.p = KDL::Vector::Zero();
	displacement.M = KDL::Rotation::Identity();
	left=1;
	hand=1;
	center_angle = 0;
	radius=0;
    }
  
    double time;
    KDL::Frame start;
    KDL::Frame displacement;
    
    bool left;
    bool hand;
    double center_angle;
    double radius;  
};

class bezier_parameters
{
public:
    
    bezier_parameters()
    {
	time=1;
	start.p = KDL::Vector::Zero();
	start.M = KDL::Rotation::Identity();
	end.p = KDL::Vector::Zero();
	end.M = KDL::Rotation::Identity();
    }
    
    double time;
    KDL::Frame start;
    KDL::Frame end; 
    bezier_curve* bz_fun;

};
    
class trajectory_generator
{
public:
  
    ros::NodeHandle n;
    
    trajectory_generator();
    
    void line_trajectory(std::map<double,KDL::Frame>& trj);
    void circle_trajectory(std::map<double,KDL::Frame>& trj);
    void bezier_trajectory(std::map<double,KDL::Frame>& trj);
    
    void set_line_time(double t);
    void set_line_start(KDL::Frame start);
    void set_line_displacement(KDL::Frame displacement);
    
    void set_circle_time(double t);
    void set_circle_start(KDL::Frame start);
    void set_circle_displacement(KDL::Frame displacement);
    void set_circle_left_ee(bool left);
    void set_circle_hand_ee(bool hand);
    void set_circle_center_angle(double angle);
    void set_circle_radius(double radius);
    
    void set_bezier_time(double t);
    void set_bezier_start(KDL::Frame start);
    void set_bezier_end(KDL::Frame end);
    bool trajInCollision();
    void computeNeighborsCOM();
    void computeBezierCurve();
    void avoidObstacle();

  
private:
    line_parameters line_param;
    circle_parameters circle_param;    
    bezier_parameters bezier_param;
};    
    
#endif //TRJ_GEN_H