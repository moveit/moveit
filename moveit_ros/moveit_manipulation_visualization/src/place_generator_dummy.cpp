/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: E. Gil Jones

#include <moveit_manipulation_visualization/place_generator_dummy.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_tools/shape_extents.h>
#include <ros/console.h>

namespace moveit_manipulation_visualization {

class ShapeVisitorComputeExtents : public boost::static_visitor<Eigen::Vector3d>
{
public:
    
  Eigen::Vector3d operator()(const shape_msgs::Plane &shape_msg) const
  {
    Eigen::Vector3d e(0.0, 0.0, 0.0);
    return e;
  }
  
  Eigen::Vector3d operator()(const shape_msgs::Mesh &shape_msg) const
  {   
    double x_extent, y_extent, z_extent;
    shape_tools::getShapeExtents(shape_msg, x_extent, y_extent, z_extent);
    Eigen::Vector3d e(x_extent, y_extent, z_extent);
    return e;
  }
  
  Eigen::Vector3d operator()(const shape_msgs::SolidPrimitive &shape_msg) const
  {
    double x_extent, y_extent, z_extent;
    shape_tools::getShapeExtents(shape_msg, x_extent, y_extent, z_extent);
    Eigen::Vector3d e(x_extent, y_extent, z_extent);
    return e;
  }
};

bool PlaceGeneratorDummy::generatePlaceLocations(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                 const std::string& object,
                                                 const std::string& support,
                                                 std::vector<geometry_msgs::PoseStamped>& place_locations)
{
  place_locations.clear();

  moveit_msgs::CollisionObject sup;
  if(!planning_scene->getCollisionObjectMsg(support, 
                                            sup)) {
    ROS_WARN_STREAM("Don't appear to have object " << support << " in planning scene for place support");
    return false;
  }

  std::vector<const planning_models::KinematicState::AttachedBody*> ab;
  planning_scene->getCurrentState().getAttachedBodies(ab);

  if(ab.size() == 0) {
    ROS_WARN_STREAM("No attached bodies associated with current state of planning scene.  Can't place");
    return false;
  }
  shapes::ShapeMsg attached_shape;
  bool found = false;
  for(unsigned int i = 0; i < ab.size(); i++) {
    if(ab[i]->getName() == object) {
      shapes::constructMsgFromShape(ab[i]->getShapes()[0].get(), attached_shape);
      found = true;
      break;
    }
  }
  if(!found) {
    ROS_WARN_STREAM("Don't appear to have object " << object << " in planning scene for place");
  }

  if(sup.primitives.empty() || sup.primitives[0].type != shape_msgs::SolidPrimitive::BOX) {
    ROS_WARN_STREAM("Dummy generator can only deal with support surface boxes");
    return false;
  }

  Eigen::Affine3d sup_pose;
  planning_models::poseFromMsg(sup.primitive_poses[0], sup_pose);


  Eigen::Vector3d ex = boost::apply_visitor(ShapeVisitorComputeExtents(), attached_shape);
  double xex = ex.x();
  double yex = ex.y();
  double zex = ex.z();
  
  if(object.find("drive") != std::string::npos) {
    
  } 

  double l = sup.primitives[0].dimensions[0]-xex;
  double w = sup.primitives[0].dimensions[1]-yex;
  double d = sup.primitives[0].dimensions[2]/2.0;//+zex/2.0;

  double spacing = .1;

  unsigned int lnum = floor(l/spacing);
  unsigned int wnum = floor(w/spacing);

  l = ((lnum*1.0)*spacing);
  w = ((wnum*1.0)*spacing);

  std::vector<double> angles;
  angles.push_back(0);
  angles.push_back(M_PI/4.0);
  angles.push_back(-M_PI/4.0);
  angles.push_back(M_PI/2.0);
  angles.push_back(-M_PI/2.0);
  angles.push_back(-M_PI);
  angles.push_back(M_PI);

  unsigned int total_place_locations = lnum*wnum*angles.size();

  if(total_place_locations == 0) {
    ROS_WARN_STREAM("Surface too small for placing");
    return false;
  }

  std::vector<unsigned int> random_numbers(total_place_locations);
  for(unsigned int i = 0; i < total_place_locations; i++) {
    random_numbers[i] = i;
  }
  //random_shuffle(random_numbers.begin(), random_numbers.end());
  
  place_locations.resize(total_place_locations);
  unsigned int cur_ind = 0;
  for(unsigned int i = 0; i < lnum; i++) {
    for(unsigned int j = 0; j < wnum; j++) { 
      for(unsigned int k = 0; k < angles.size(); k++, cur_ind++) {
        geometry_msgs::PoseStamped place_pose;
        place_pose.pose = sup.primitive_poses[0];
        place_pose.header.frame_id = planning_scene->getPlanningFrame();
        place_pose.pose.position.x += -(l/2.0)+((i*1.0)*spacing);
        place_pose.pose.position.y += -(w/2.0)+((j*1.0)*spacing);
        place_pose.pose.position.z += d;
        Eigen::Affine3d cur;
        planning_models::poseFromMsg(place_pose.pose, cur);
        Eigen::Affine3d trans(Eigen::AngleAxisd(angles[k], Eigen::Vector3d::UnitZ()));
        Eigen::Affine3d conv = cur*trans;
        planning_models::msgFromPose(conv, place_pose.pose);
        ROS_DEBUG_STREAM("Place location " << i << " " << j << " " 
                         << place_pose.pose.position.x << " " 
                         << place_pose.pose.position.y << " " 
                         << place_pose.pose.position.z);
        place_locations[random_numbers[cur_ind]] = place_pose;
      }
    }
  }
  return true;
}

}
