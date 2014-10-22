/* Copyright [2014] [Ciccio Baciccio]
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "../include/trajectory_generator/bezier_curve.h"
#include <boost/iterator/iterator_concepts.hpp>

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
	KDL::Vector pc = curve->at(i);  
	
// 	geometry_msgs::PoseStamped world_point;
// 	world_point.pose.orientation.w=1;world_point.pose.orientation.x=0;world_point.pose.orientation.y=0;world_point.pose.orientation.z=0;
// 	world_point.pose.position.x=pc.x();
// 	world_point.pose.position.y=pc.y();
// 	world_point.pose.position.z=pc.z();
// 	world_point.header.frame_id="world";
// 	
// 	std::string frame="camera_link"; 	
// 	geometry_msgs::PoseStamped camera_point;
// 	
// 	tf.transformPose(frame,world_point,camera_point);
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

bool bezier_curve::computeNeighborsCOM()
{
    octomap::OcTreeKey key = octree->coordToKey(obstacle_collision); // get key
    std::cout<<"Key of the voxel: "<<key[0]<<" "<<key[1]<<" "<<key[2]<<std::endl;		
    octomap::OcTreeKey neighborkey = key;
    std::vector<octomap::point3d> neighbor_vector_pos;
    neighbor_vector_pos.clear();
    
    octomap::OcTreeNode* neighbor_node;
    octomap::point3d pn;
    
    // Compute the 6 neighbors of the selected voxel
    for (int i=0;i < 3;i++)
    {
	neighborkey[i] +=1;
	std::cout<<"Key of the voxel neighbor: "<<neighborkey[i]<<std::endl;
	pn = octree->keyToCoord(neighborkey);
	neighbor_node = octree->search(pn);
			
	if (neighbor_node != NULL)
	{
	    if(octree->isNodeOccupied(neighbor_node))
	    {
		neighbor_vector_pos.push_back(pn);
		std::cout<<"Neighbor point: "<<pn.x()<<" "<<pn.y()<<" "<<pn.z()<<" ["<<neighbor_node->getOccupancy()<<"]"<<std::endl;
	    }
	    else
		std::cout<<"Neighbor not occupied ["<<neighbor_node->getOccupancy()<<"]"<<std::endl;
	}
	else
	    std::cout<<"Neighbor does not exists"<<std::endl;
	
	neighborkey[i] -=2;
	std::cout<<"Key of the voxel neighbor: "<<neighborkey[i]<<std::endl;
	pn = octree->keyToCoord(neighborkey);
	neighbor_node = octree->search(pn);
	
	if (neighbor_node != NULL)
	{
	    if(octree->isNodeOccupied(neighbor_node))
	    {
		neighbor_vector_pos.push_back(pn);
		std::cout<<"Neighbor point: "<<pn.x()<<" "<<pn.y()<<" "<<pn.z()<<" ["<<neighbor_node->getOccupancy()<<"]"<<std::endl;
	    }
	    else
		std::cout<<"Neighbor not occupied ["<<neighbor_node->getOccupancy()<<"]"<<std::endl;
	}
	else
	    std::cout<<"Neighbor does not exists"<<std::endl; 
    }
    
    for (int i=0;i<neighbor_vector_pos.size();i++)	
    {
	octomap::point3d pnx = neighbor_vector_pos.at(i);
	neighborsCOM += pnx;
    }
    
    neighborsCOM /= neighbor_vector_pos.size();
    
    return true;
    
}

bool bezier_curve::CheckNeighbors(KDL::Vector curvePoint) // TODO from Ale, compute the com of the neighborhood of the node in collision
{
    return true;
  
}

void bezier_curve::avoidObstacle()
{
    if (neighborsCOM.y() < curve_collision.y())	
	obst_left = true;
    else
	obst_left = false;
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
  // Check y-axis //TODO Verify that obst_down corresponds to obst_left in the y-axis test
  if (curve_collision.y() > pm.y())
  {
      if (obst_left)
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
      if (obst_left)
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
