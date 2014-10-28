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

#include "../include/trajectory_generator/bezier_curve.h"
#include <boost/iterator/iterator_concepts.hpp>

//TODO 	Make the obstacle avoidance algorithm generic: 
//TODO	given a goal vector P=(p_x,p_y,p_z), the invariant dimension will be
//TODO 	the one with maximum magnitude, i.e. if P=(3,2,1), the invariant dimension
//TODO 	will be x and the control points act on y and z (as it is now)

bezier_curve::bezier_curve(ros::NodeHandle* n, KDL::Vector p_start, KDL::Vector p_end)
{ 
    octomap_sub = n->subscribe("octomap_binary", 1000, &bezier_curve::octomapCallback, this);
    
    curve = new std::vector<KDL::Vector>;
    curve->clear();
    ctrl_points = new std::vector<KDL::Vector>;
    ctrl_points->clear();
    
    start = p_start;
    end = p_end;
    
    pm.x((start.x() + end.x())/2);
    pm.y((start.y() + end.y())/2);
    pm.z((start.z() + end.z())/2);
    ctrl_p1 = ctrl_p2 = pm;
  
    ctrl_points->push_back(start);
    ctrl_points->push_back(ctrl_p1);
    ctrl_points->push_back(ctrl_p2);
    ctrl_points->push_back(end);
}

void bezier_curve::computeBezier( std::vector<KDL::Vector>* cp, std::vector<KDL::Vector>* curve, double t)
{
    q.x(0.0);
    q.y(0.0);
    q.z(0.0);
    for (int j = 0; j < cp->size(); j++)
    {
	KDL::Vector p = cp->at(j);
	q.x(q.x()+ p.x() * Bernstein(cp->size()-1,j,t));
	q.y(q.y()+ p.y() * Bernstein(cp->size()-1,j,t));
	q.z(q.z()+ p.z() * Bernstein(cp->size()-1,j,t));
    }	
}

int bezier_curve::factorial(int x) {
    if (x <= 1) return 1; else return x*factorial(x - 1);
}

int bezier_curve::binomialCoeff(int a, int b)
{
    return factorial(a) / (factorial(b) * factorial (a-b));
}

float bezier_curve::Bernstein(int n, int j, float t)
{   
    return binomialCoeff(n,j)*pow(t,j)*pow(1-t,n-j);
}

bool bezier_curve::inCollisionOctomap()	
{
//     ComputeBezier(cp,n_points,curve);
    bool result = false;
    
    // NO PROBLEM WITH TF: BOTH CURVE AND OCTOMAP ARE IN WORLD FRAME
    for (int i=0;i<curve->size();i++)
    { 
	KDL::Vector pc = curve->at(i);	// get a point of the curve  
	
	// Transform from world to camera_link
	std::string target_frame="camera_link";    
	geometry_msgs::PoseStamped world_point;
	world_point.pose.orientation.w=1;world_point.pose.orientation.x=0;world_point.pose.orientation.y=0;world_point.pose.orientation.z=0;
	world_point.pose.position.x=pc.x();
	world_point.pose.position.y=pc.y();
	world_point.pose.position.z=pc.z(),
	world_point.header.frame_id="base_link";
	geometry_msgs::PoseStamped camera_point;
	
	std::string err_msg;
	if(!(tf.waitForTransform(target_frame,world_point.header.frame_id,ros::Time::now(), ros::Duration(1.0), ros::Duration(0.01), &err_msg)))
	{
	    ROS_ERROR("Error in tf: %s",err_msg.c_str());
	    return true;
	}
	
	tf.transformPose(target_frame,world_point,camera_point);
	pc.x(camera_point.pose.position.x);
	pc.y(camera_point.pose.position.y);
	pc.z(camera_point.pose.position.z);
	
	if(octree != NULL)
	{
	    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(); it!= octree->end_leafs(); ++it)
	    {     	    
		// Collision CONDITION
		octomap::point3d po = it.getCoordinate();
		if ((pc.x() >= po.x()-0.025) && (pc.x() <= po.x()+0.025) &&	// curva interna ad un cubetto 5x5x5
		    (pc.y() >= po.y()-0.025) && (pc.y() <= po.y()+0.025) && 
		    (pc.z() >= po.z()-0.025) && (pc.z() <= po.z()+0.025)) 
		{ 
		    std::cout<<"Found collision in: ("<<pc.x()<<", "<<pc.y()<<", "<<pc.z()<<")"<<std::endl; 
		    std::cout<<"Associated voxel is: ("<<po.x()<<", "<<po.y()<<", "<<po.z()<<")"<<std::endl; 
		    curve_collision = pc;
		    obstacle_collision = po;
		    result=true;
		    break;  
		}
	    } 
	}
	if (result == true) break;
    }	
    return result;
}

void bezier_curve::computeNeighborsCOM()
{
    octomap::OcTreeKey key = octree->coordToKey(obstacle_collision); // get key
    std::cout<<"Key of the voxel: "<<key[0]<<" "<<key[1]<<" "<<key[2]<<std::endl;		
    octomap::OcTreeKey neighborkey = key;
    std::vector<octomap::point3d> neighbor_vector_pos;
    neighbor_vector_pos.clear();
    
    octomap::OcTreeNode* neighbor_node;
    octomap::point3d pn;
    
    // Compute the 6 neighbors of the selected voxel
//     for (int i=0;i < 3;i++)
//     {
// 	neighborkey[i] +=1;
// 	std::cout<<"Key of the voxel neighbor: "<<neighborkey[i]<<std::endl;
// 	pn = octree->keyToCoord(neighborkey);
// 	neighbor_node = octree->search(pn);
// 			
// 	if (neighbor_node != NULL)
// 	{
// 	    if(octree->isNodeOccupied(neighbor_node))
// 	    {
// 		neighbor_vector_pos.push_back(pn);
// 		std::cout<<"Neighbor point: "<<pn.x()<<" "<<pn.y()<<" "<<pn.z()<<" ["<<neighbor_node->getOccupancy()<<"]"<<std::endl;
// 	    }
// 	    else
// 		std::cout<<"Neighbor not occupied ["<<neighbor_node->getOccupancy()<<"]"<<std::endl;
// 	}
// 	else
// 	    std::cout<<"Neighbor does not exists"<<std::endl;
// 	
// 	neighborkey[i] -=2;
// 	std::cout<<"Key of the voxel neighbor: "<<neighborkey[i]<<std::endl;
// 	pn = octree->keyToCoord(neighborkey);
// 	neighbor_node = octree->search(pn);
// 	
// 	if (neighbor_node != NULL)
// 	{
// 	    if(octree->isNodeOccupied(neighbor_node))
// 	    {
// 		neighbor_vector_pos.push_back(pn);
// 		std::cout<<"Neighbor point: "<<pn.x()<<" "<<pn.y()<<" "<<pn.z()<<" ["<<neighbor_node->getOccupancy()<<"]"<<std::endl;
// 	    }
// 	    else
// 		std::cout<<"Neighbor not occupied ["<<neighbor_node->getOccupancy()<<"]"<<std::endl;
// 	}
// 	else
// 	    std::cout<<"Neighbor does not exists"<<std::endl; 
//     }
    
    // Compute 26 neighbors
    for (int n = 1; n < 4 ; n++)
    {
      std::cout<<"Expanding to the level:"<<n<<std::endl;
      for (neighborkey[2] = key[2] - n; neighborkey[2] <= key[2] + n; ++neighborkey[2])
      {
	for (neighborkey[1] = key[1] - n; neighborkey[1] <= key[1] + n; ++neighborkey[1])
	{
	  for (neighborkey[0] = key[0] - n; neighborkey[0] <= key[0] + n; ++neighborkey[0])
	  {
// 		  std::cout<<"Key of the voxel neighbor: "<<neighborkey[0]<<" "<<neighborkey[1]<<" "<<neighborkey[2]<<std::endl;
	    if (neighborkey != key)	
	    {
	      pn = octree->keyToCoord(neighborkey);
	      neighbor_node = octree->search(pn);
	      if (neighbor_node != NULL)
	      {
		if(octree->isNodeOccupied(neighbor_node))
		{
		  neighbor_vector_pos.push_back(pn);
// 			std::cout<<"Neighbor point: "<<pn.x()<<" "<<pn.y()<<" "<<pn.z()<<" ["<<neighbor_node->getOccupancy()<<"]"<<std::endl;
// 		  count++;
		}
// 		      else
// 			std::cout<<"Neighbor not occupied ["<<neighbor_node->getOccupancy()<<"]"<<std::endl;
	      }
	      else
		std::cout<<"Neighbor does not exists"<<std::endl;
	    }
	  }
	}
      }
    }
    
    for (int i=0;i<neighbor_vector_pos.size();i++)	
    {
	octomap::point3d pnx = neighbor_vector_pos.at(i);
	neighborsCOM += pnx;
    }
    
    neighborsCOM /= neighbor_vector_pos.size();
    
}

void bezier_curve::avoidObstacle() //TODO Now works with x invariant, TO BE GENERALIZED
{
    // Transform curve collision and neighborsCOM from camera_link to world
    std::string target_frame="base_link";    
    geometry_msgs::PoseStamped collision_camera, neighborsCOM_camera;
    
    collision_camera.pose.orientation.w=1;collision_camera.pose.orientation.x=0;collision_camera.pose.orientation.y=0;collision_camera.pose.orientation.z=0;
    collision_camera.pose.position.x=curve_collision.x();
    collision_camera.pose.position.y=curve_collision.y();
    collision_camera.pose.position.z=curve_collision.z();
    collision_camera.header.frame_id="camera_link";
    geometry_msgs::PoseStamped collision_world;
    
    std::string err_msg;
    if(!(tf.waitForTransform(target_frame,collision_camera.header.frame_id,ros::Time::now(), ros::Duration(1.0), ros::Duration(0.01), &err_msg)))
    {
	ROS_ERROR("Error in tf: %s",err_msg.c_str());
	return;
    }
  
    tf.transformPose(target_frame,collision_camera,collision_world);
    curve_collision.x(collision_world.pose.position.x);
    curve_collision.y(collision_world.pose.position.y);
    curve_collision.z(collision_world.pose.position.z);
  
    neighborsCOM_camera.pose.orientation.w=1;neighborsCOM_camera.pose.orientation.x=0;neighborsCOM_camera.pose.orientation.y=0;neighborsCOM_camera.pose.orientation.z=0;
    neighborsCOM_camera.pose.position.x=neighborsCOM.x();
    neighborsCOM_camera.pose.position.y=neighborsCOM.y();
    neighborsCOM_camera.pose.position.z=neighborsCOM.z();
    neighborsCOM_camera.header.frame_id="camera_link";
    geometry_msgs::PoseStamped neighborsCOM_world;
    
    if(!(tf.waitForTransform(target_frame,neighborsCOM_camera.header.frame_id,ros::Time::now(), ros::Duration(1.0), ros::Duration(0.01), &err_msg)))
    {
	ROS_ERROR("Error in tf: %s",err_msg.c_str());
	return;
    }
    
    tf.transformPose(target_frame,neighborsCOM_camera,neighborsCOM_world);
    neighborsCOM.x()=neighborsCOM_world.pose.position.x;
    neighborsCOM.y()=neighborsCOM_world.pose.position.y;
    neighborsCOM.z()=neighborsCOM_world.pose.position.z;
    
    if (neighborsCOM.y() < curve_collision.y())	
	obst_right = true;
    else
	obst_right = false;
    if (neighborsCOM.z() < curve_collision.z())
	obst_down = true;
    else
	obst_down = false;
	    
  // TODO modify the control points depending on the computed com 
  if (curve_collision.x() <= pm.x())
      ctrl_p1.x(ctrl_p1.x()+ w*(curve_collision.x() - pm.x()));
  if (curve_collision.x() > pm.x())
      ctrl_p2.x(ctrl_p2.x() + w*(curve_collision.x() - pm.x()));
  // Check z-axis
  if (curve_collision.z() > pm.z())
  {
      if (obst_down)
      {
	  ctrl_p1.z(ctrl_p1.z()+ w*(curve_collision.z() - pm.z()));
	  ctrl_p2.z(ctrl_p2.z()+ w*(curve_collision.z() - pm.z()));
      }
      else
      {
	  ctrl_p1.z(ctrl_p1.z()+ w*(pm.z() -curve_collision.z())); 
	  ctrl_p2.z(ctrl_p2.z()+ w*(pm.z() -curve_collision.z())); 
      } 
  }
  else
  {
      if (obst_down)
      {
	  ctrl_p1.z(ctrl_p1.z()+ w*(pm.z() -curve_collision.z())); 
	  ctrl_p2.z(ctrl_p2.z()+ w*(pm.z() -curve_collision.z()));
      }
      else
      {
	  ctrl_p1.z(ctrl_p1.z()+ w*(curve_collision.z() - pm.z()));
	  ctrl_p2.z(ctrl_p2.z()+ w*(curve_collision.z() - pm.z()));
      }
  }
  // Check y-axis //TODO Verify that obst_down corresponds to obst_right in the y-axis test
  if (curve_collision.y() > pm.y())
  {
      if (obst_right)
      {
	  ctrl_p1.y(ctrl_p1.y()+ w*(curve_collision.y() - pm.y()));
	  ctrl_p2.y(ctrl_p2.y()+ w*(curve_collision.y() - pm.y()));
      }
      else
      {
	  ctrl_p1.y(ctrl_p1.y()+ w*(pm.y() -curve_collision.y())); 
	  ctrl_p2.y(ctrl_p2.y()+ w*(pm.y() -curve_collision.y())); 
      } 
  }
  else
  {
      if (obst_right)
      {
	  ctrl_p1.y(ctrl_p1.y()+ w*(pm.y() -curve_collision.y())); 
	  ctrl_p2.y(ctrl_p2.y()+ w*(pm.y() -curve_collision.y()));
      }
      else
      {
	  ctrl_p1.y(ctrl_p1.y()+ w*(curve_collision.y() - pm.y()));
	  ctrl_p2.y(ctrl_p2.y()+ w*(curve_collision.y() - pm.y()));
      }
  }
    
  std::cout<<"Control point 1: "<<ctrl_p1.x()<<" "<<ctrl_p1.y()<<" "<<ctrl_p1.z()<<std::endl; 
  std::cout<<"Control point 2: "<<ctrl_p2.x()<<" "<<ctrl_p2.y()<<" "<<ctrl_p2.z()<<std::endl; 
  ctrl_points->clear();
  ctrl_points->push_back(start);
  ctrl_points->push_back(ctrl_p1);
  ctrl_points->push_back(ctrl_p2);
  ctrl_points->push_back(end);
}

void bezier_curve::octomapCallback(const octomap_msgs::Octomap& octomap_msg)
{	
     tree = octomap_msgs::binaryMsgToMap(octomap_msg);
     octree = dynamic_cast<octomap::OcTree*>(tree);
     if (octree==NULL)
	  std::cout<<"!! OcTree is NULL !!"<<std::endl;
      else
      {
	    ROS_INFO("Received Octomap");
      }
}
