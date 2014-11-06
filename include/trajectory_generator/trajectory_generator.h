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
#include "curves_types.h"
#include <ros/ros.h>
    
class trajectory_generator
{
public:
    
    trajectory_generator();
    
    KDL::Vector polynomial_interpolation(const polynomial_coefficients& poly, const KDL::Vector& vec_f, double time, double t_f);
    double polynomial_interpolation(const polynomial_coefficients& poly, double value_f, double time, double t_f);
    // GENERIC TRAJECTORIES
    
    void line_trajectory(std::map<double,KDL::Frame>& trj);
    void set_line_time(double time);
    void set_line_start(KDL::Frame start);
    void set_line_displacement(const KDL::Frame& displacement);
    
    void circle_trajectory(std::map<double,KDL::Frame>& trj);
    void set_circle_time(double time);
    void set_circle_start(KDL::Frame start);
    void set_circle_displacement(const KDL::Frame& displacement);
    void set_circle_left_ee(bool left);
    void set_circle_hand_ee(bool hand);
    void set_circle_center_angle(double angle);
    void set_circle_radius(double radius);
    
    void bezier_trajectory(std::map<double,KDL::Frame>& trj);
    void set_bezier_time(double time);
    void set_bezier_start(KDL::Frame start);
    void set_bezier_end(KDL::Frame end);
    bool trajInCollision();
    void computeNeighborsCOM();
    void computeBezierCurve();
    void avoidObstacle();
    
    // VALVE TRAJECTORIES
    
    bool valve_line_trajectory(double t, KDL::Frame& pos_d, KDL::Twist& vel_d );
    bool valve_line_initialize(double time, const KDL::Frame& start, const KDL::Frame& final);
    
    double valve_circle_trajectory(double t, bool left_arm, KDL::Rotation& ROTe, KDL::Frame& pos_d, KDL::Frame& vel_d);
    bool valve_circle_initialize(double time, double radius, double center_angle, const KDL::Frame& start);
    
    double valve_turn_trajectory_old(double t, KDL::Rotation& ROTv, KDL::Frame& pos_d, KDL::Twist& vel_d);
    bool valve_turn_trajectory(double t, KDL::Frame& pos_d, KDL::Twist& vel_d);
    bool valve_turn_initialize(double time, double radius, double center_angle, const KDL::Frame& start, const KDL::Frame& valve);
    
private:
    line_parameters line_param;
    circle_parameters circle_param;    
    bezier_parameters bezier_param;
    valve_line_parameters valve_line_param;
    valve_circle_parameters valve_circle_param;
    valve_turn_parameters valve_turn_param;
};    
    
#endif //TRJ_GEN_H