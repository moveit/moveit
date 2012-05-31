/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \author E. Gil Jones */

#include <collision_distance_field/collision_robot_distance_field.h>
#include <collision_distance_field/collision_common_distance_field.h>

namespace collision_distance_field
{

CollisionRobotDistanceField::CollisionRobotDistanceField(const planning_models::KinematicModelConstPtr& kmodel, 
                                                         double size_x, 
                                                         double size_y,
                                                         double size_z,
                                                         bool use_signed_distance_field,
                                                         double resolution,
                                                         double collision_tolerance,
                                                         double max_propogation_distance,
                                                         double padding, 
                                                         double scale)
  : collision_detection::CollisionRobot(kmodel, padding, scale),
    size_x_(size_x),
    size_y_(size_y),
    size_z_(size_z),
    use_signed_distance_field_(true),
    resolution_(resolution),
    collision_tolerance_(collision_tolerance),
    max_propogation_distance_(max_propogation_distance)
{  
}

void CollisionRobotDistanceField::checkSelfCollisionHelper(const collision_detection::CollisionRequest& req,
                                                           collision_detection::CollisionResult& res,
                                                           const planning_models::KinematicState& state,
                                                           const collision_detection::AllowedCollisionMatrix *acm) const
{
  boost::shared_ptr<const DistanceFieldCacheEntry> dfce = getDistanceFieldCacheEntry(req.group_name,
                                                                                     state,
                                                                                     acm);
  if(!dfce) {
    boost::shared_ptr<DistanceFieldCacheEntry> new_dfce = generateDistanceFieldCacheEntry(req.group_name,
                                                                                          state,
                                                                                          acm);
    boost::mutex::scoped_lock slock(update_cache_lock_);
    (const_cast<CollisionRobotDistanceField*>(this))->distance_field_cache_entry_ = new_dfce;
  }
}

boost::shared_ptr<const CollisionRobotDistanceField::DistanceFieldCacheEntry> 
CollisionRobotDistanceField::getDistanceFieldCacheEntry(const std::string& group_name,
                                                        const planning_models::KinematicState& state,
                                                        const collision_detection::AllowedCollisionMatrix *acm) const
{ 
  boost::shared_ptr<const DistanceFieldCacheEntry> ret;
  boost::shared_ptr<const DistanceFieldCacheEntry> cur = distance_field_cache_entry_;
  if(group_name != cur->group_name_) {
    ROS_INFO_STREAM("No cache entry as group name changed from " << cur->group_name_ << " to " << group_name);
    return ret;
  } // else if(!stateIsTheSame(state, cur->state_)) {
  //   ROS_INFO_STREAM("Regenerating distance field as state has changed from last time");
  //   return ret;
  // } else if(!acmIsTheSame(acm, cur)) {
  //   ROS_INFO_STREAM("Regenerating distance field as some relevant part of the acm changed");    
  //   return ret;
  // }
  return cur;
}

void CollisionRobotDistanceField::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                                     collision_detection::CollisionResult& res,
                                                     const planning_models::KinematicState& state) const
{
  return checkSelfCollisionHelper(req, res, state, NULL);
}

boost::shared_ptr<CollisionRobotDistanceField::DistanceFieldCacheEntry> 
CollisionRobotDistanceField::generateDistanceFieldCacheEntry(const std::string& group_name,
                                                             const planning_models::KinematicState& state,
                                                             const collision_detection::AllowedCollisionMatrix *acm) const
{ 
  boost::shared_ptr<DistanceFieldCacheEntry> dfce(new DistanceFieldCacheEntry());
  if(kmodel_->getJointModelGroup(group_name) == NULL) {
    ROS_WARN_STREAM("No group " << group_name);
    return dfce;
  }
  dfce->group_name_ = group_name;
  dfce->state_.reset(new planning_models::KinematicState(state));
  if(acm) {
    dfce->acm_ = *acm;
  } 
  dfce->link_names_ = kmodel_->getJointModelGroup(group_name)->getUpdatedLinkModelsWithGeometryNames();
  const std::vector<planning_models::KinematicState::LinkState*>& lsv = state.getLinkStateVector();
  //need to do with with all link models for attached objects, not just those with geometry
  for(unsigned int i = 0; i < kmodel_->getJointModelGroup(group_name)->getUpdatedLinkModelNames().size(); i++) {
    bool found = false;
    for(unsigned int j = 0; j < lsv.size(); j++) {
      if(lsv[j]->getName() == kmodel_->getJointModelGroup(group_name)->getUpdatedLinkModelNames()[i]) {
        dfce->link_state_indices_.push_back(j);
        found = true;
        break;
      }
    }
    if(!found) {
      ROS_INFO_STREAM("No link state found for link " << dfce->link_names_[i]);
      return dfce;
    }
    dfce->link_body_decompositions_.push_back(getLinkBodyDecomposition(dfce->state_->getLinkState(dfce->link_names_[i]), resolution_));
    std::vector<const planning_models::KinematicState::AttachedBody*> attached_bodies;
    dfce->state_->getLinkState(dfce->link_names_[i])->getAttachedBodies(attached_bodies);
    for(unsigned int i = 0; i < attached_bodies.size(); i++) {
      dfce->attached_body_decompositions_.push_back(getAttachedBodyDecomposition(attached_bodies[i], resolution_));
      dfce->attached_body_names_.push_back(attached_bodies[i]->getName());
      dfce->attached_body_link_state_indices_.push_back(dfce->link_state_indices_[i]);
    }
  }
  //TODO - deal with AllowedCollisionMatrix
  //now we need to actually set the points
  //TODO - deal with shifted robot
  std::vector<Eigen::Vector3d> all_points;
  for(unsigned int i = 0; i < dfce->link_body_decompositions_.size(); i++) {
    all_points.insert(all_points.end(), 
                      dfce->link_body_decompositions_[i]->getCollisionPoints().begin(), 
                      dfce->link_body_decompositions_[i]->getCollisionPoints().end());
  }
  for(unsigned int i = 0; i < dfce->attached_body_decompositions_.size(); i++) {
    all_points.insert(all_points.end(),
                      dfce->attached_body_decompositions_[i]->getCollisionPoints().begin(),
                      dfce->attached_body_decompositions_[i]->getCollisionPoints().end());
  }
  dfce->distance_field_->addPointsToField(all_points);
  return dfce;
}

}  
