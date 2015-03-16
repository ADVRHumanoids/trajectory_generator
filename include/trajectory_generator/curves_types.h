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

#ifndef CURVE_TYPES_H
#define CURVE_TYPES_H

#include <string>
#include <map>
#include <netinet/sctp.h>
#include <kdl/frames.hpp>
#include "bezier_curve.h"

class polynomial_coefficients
{
public:
    polynomial_coefficients()
    {
	a_=20.0;
	b_=-30.0;
	c_=12.0;
    }

    void set_polynomial_coeff(double a, double b, double c)
    {
	a_=a;
	b_=b;
	c_=c;
    }
    
    double a() const{ return a_;}
    double b() const{ return b_;}
    double c() const{ return c_;}
    
private:
    double a_,b_,c_;
};

class line_parameters
{
public:
    line_parameters()
    {
    initialized = false;
	time=1;
	start.p = KDL::Vector::Zero();
	start.M = KDL::Rotation::Identity();
	displacement.p = KDL::Vector::Zero();
	displacement.M = KDL::Rotation::Identity();
    }
    
    bool initialized;
    double time;
    KDL::Frame start;
    KDL::Frame displacement;  
};

class circle_parameters
{
public:
    circle_parameters()
    {
	initialized=false;
	time=1;
	start.p = KDL::Vector::Zero();
	start.M = KDL::Rotation::Identity();
	center_angle=0;
	radius=1;
    }
  
    bool initialized;
    double time;
    KDL::Frame start, object;
    double center_angle;
    double radius;
};

class foot_traj_parameters
{
public:
    foot_traj_parameters()
    {
	initialized=false;
	time=1;
	start.p = KDL::Vector::Zero();
	start.M = KDL::Rotation::Identity();
	displacement.p = KDL::Vector::Zero();
	displacement.M = KDL::Rotation::Identity();
	ctrl_points.clear();
    }
  
    bool initialized;
    double time;
    KDL::Frame start;
    KDL::Frame displacement;
    std::vector<KDL::Vector> ctrl_points;
};

class custom_circle_parameters
{
public:
    custom_circle_parameters()
    {
    initialized=false;
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
  
    bool initialized;
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
    initialized=false;
	time=1;
	start.p = KDL::Vector::Zero();
	start.M = KDL::Rotation::Identity();
	end.p = KDL::Vector::Zero();
	end.M = KDL::Rotation::Identity();
    }
    
    bool initialized;
    double time;
    KDL::Frame start;
    KDL::Frame end; 
    bezier_curve* bz_fun;

};

#endif //CURVE_TYPES_H
