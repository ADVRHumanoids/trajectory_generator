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

#ifndef BEZIER_CURVE_H
#define BEZIER_CURVE_H

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <kdl/frames.hpp>

#include <ros/ros.h>
#include "std_msgs/String.h"

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>

#include "geometry_msgs/PointStamped.h"
#include "tf/transform_listener.h"

class bezier_curve
{
public:
  bezier_curve(ros::NodeHandle* n, KDL::Vector p_start, KDL::Vector p_end);

  void computeBezier(double t);
  void octomapCallback(const octomap_msgs::Octomap& octomap_msg);
  bool inCollisionOctomap();
  void avoidObstacle();
  void computeNeighborsCOM();//KDL::Vector neighbor, KDL::Vector neighborCOM);
  KDL::Vector q; 
  KDL::Vector pm, ctrl_p1, ctrl_p2;
  KDL::Vector start, end;
  KDL::Vector curve_collision;		// collision point in the curve
  octomap::point3d obstacle_collision;	// collision point in the octomap
  octomap::point3d neighborsCOM; 		// mean value of neighborhood
  std::vector<KDL::Vector>* curve;	
  std::vector<KDL::Vector>* ctrl_points;		
  bool obst_down;
  bool obst_right;
  octomap::OcTree* octree;
  
private:
  float b;
  float w;
  float Bernstein(int n, int j, float t);
  int factorial (int x);
  int binomialCoeff (int a, int b);
  
  tf::TransformListener listener;
  ros::Subscriber octomap_sub;
  octomap::AbstractOcTree* tree;
  
  
};

#endif // BEZIER_CURVE_H
