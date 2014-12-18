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
#include <tf_conversions/tf_kdl.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Eigen>

class trajectory_generator
{
public:
    
    trajectory_generator();
    
    KDL::Vector polynomial_interpolation(const polynomial_coefficients& poly, const KDL::Vector& vec_f, double time, double t_f);
    double polynomial_interpolation(const polynomial_coefficients& poly, double value_f, double time, double t_f);
    
    // GENERIC TRAJECTORIES
    bool line_trajectory(double t, KDL::Frame& pos_d, KDL::Twist& vel_d );
    bool line_initialize(double time, const KDL::Frame& start, const KDL::Frame& final);
    bool complete_line_trajectory(std::map<double, KDL::Frame>& pos_trj, std::map<double, KDL::Twist>& vel_trj, double delta_t=0.01);

    bool circle_trajectory(double t, KDL::Frame& pos_d, KDL::Twist& vel_d);
    bool circle_initialize(double time, double radius, double center_angle, const KDL::Frame& start, const KDL::Frame& object);
    bool complete_circle_trajectory(std::map<double, KDL::Frame>& pos_trj, std::map<double, KDL::Twist>& vel_trj, double delta_t=0.01);
    
    bool foot_trajectory(double t, KDL::Frame& pos_d);
    bool foot_initialize(double time, const KDL::Frame& start, const KDL::Frame& final, double height, double cp1 = 0.25, double cp2 = 0.75);
    bool complete_foot_trajectory(std::map<double, KDL::Frame>& pos_trj, std::map<double, KDL::Twist>& vel_trj, double delta_t=0.01);
    
    // CUSTOM CIRCLE
    void custom_circle_trajectory(double t, KDL::Frame& pos_d);
    void custom_circle_initialize(double time, KDL::Frame start, const KDL::Frame& displacement, 
				  bool left, bool hand, double angle, double radius);
    void complete_custom_circle_trajectory(std::map<double, KDL::Frame>& pos_trj, double delta_t=0.01);
    
    // BEZIER TRAJECTORIES
    void bezier_initialize(double time, KDL::Frame start, KDL::Frame end);
    void bezier_trajectory(std::map<double,KDL::Frame>& trj);
    bool trajInCollision();
    void computeNeighborsCOM();
    void computeBezierCurve();
    void avoidObstacle();
    
    int fact(int x);
    
private:
    line_parameters line_param;
    circle_parameters circle_param;    
    foot_traj_parameters foot_param;
    custom_circle_parameters custom_circle_param;
    bezier_parameters bezier_param;
};    
    
#endif //TRJ_GEN_H