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
#include <distance_field/propagation_distance_field.h>

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
    use_signed_distance_field_(use_signed_distance_field),
    resolution_(resolution),
    collision_tolerance_(collision_tolerance),
    max_propogation_distance_(max_propogation_distance)
{  
  const std::map<std::string, planning_models::KinematicModel::JointModelGroup*>& jmgm = kmodel_->getJointModelGroupMap();
  for(std::map<std::string, planning_models::KinematicModel::JointModelGroup*>::const_iterator it = jmgm.begin();
      it != jmgm.end();
      it++) {
    std::map<std::string, bool> updated_group_entry;
    for(unsigned int i = 0; i < it->second->getUpdatedLinkModelsWithGeometryNames().size(); i++) {
      updated_group_entry[it->second->getUpdatedLinkModelsWithGeometryNames()[i]] = true;
    }
    in_group_update_map_[it->first] = updated_group_entry;
  }
  addLinkBodyDecompositions(resolution_);
}

boost::shared_ptr<CollisionRobotDistanceField::GroupStateRepresentation> 
CollisionRobotDistanceField::generateCollisionCheckingStructures(const std::string& group_name,
                                                                 const planning_models::KinematicState& state,
                                                                 const collision_detection::AllowedCollisionMatrix *acm,
                                                                 boost::shared_ptr<const DistanceFieldCacheEntry>& dfce,
                                                                 bool generate_distance_field) const
{
  dfce = getDistanceFieldCacheEntry(group_name,
                                    state,
                                    acm);
  if(!dfce || (generate_distance_field && !dfce->distance_field_)) {
    boost::shared_ptr<DistanceFieldCacheEntry> new_dfce = generateDistanceFieldCacheEntry(group_name,
                                                                                          state,
                                                                                          acm,
                                                                                          generate_distance_field);
    boost::mutex::scoped_lock slock(update_cache_lock_);
    (const_cast<CollisionRobotDistanceField*>(this))->distance_field_cache_entry_ = new_dfce;
    dfce = new_dfce;
  } 
  ros::WallTime n = ros::WallTime::now();
  boost::shared_ptr<GroupStateRepresentation> gsr = getGroupStateRepresentation(dfce,state);
  ROS_INFO_STREAM("Gsr creation " << (ros::WallTime::now()-n).toSec());
  return gsr;
}

void CollisionRobotDistanceField::checkSelfCollisionHelper(const collision_detection::CollisionRequest& req,
                                                           collision_detection::CollisionResult& res,
                                                           const planning_models::KinematicState& state,
                                                           const collision_detection::AllowedCollisionMatrix *acm) const
{
  boost::shared_ptr<const DistanceFieldCacheEntry> dfce;
  boost::shared_ptr<GroupStateRepresentation> gsr = generateCollisionCheckingStructures(req.group_name,
                                                                                        state,
                                                                                        acm,
                                                                                        dfce,
                                                                                        true);
  ros::WallTime n = ros::WallTime::now();
  if(req.contacts) {
    bool in_self = getSelfProximityGradients(dfce, gsr);
    bool in_intra = getIntraGroupProximityGradients(dfce, gsr);
    res.collision = in_self || in_intra;
  } else {
    res.collision = getSelfCollisions(dfce, gsr);
    if(!res.collision) {
      res.collision = getIntraGroupCollisions(dfce, gsr);
    }
  }
  ROS_INFO_STREAM("Getting self proximity took " << (ros::WallTime::now()-n).toSec());
  (const_cast<CollisionRobotDistanceField*>(this))->last_gsr_ = gsr;
}

boost::shared_ptr<const CollisionRobotDistanceField::DistanceFieldCacheEntry> 
CollisionRobotDistanceField::getDistanceFieldCacheEntry(const std::string& group_name,
                                                        const planning_models::KinematicState& state,
                                                        const collision_detection::AllowedCollisionMatrix *acm) const
{ 
  boost::shared_ptr<const DistanceFieldCacheEntry> ret;
  if(!distance_field_cache_entry_) {
    ROS_WARN_STREAM("No current dfce");
    return ret;
  }
  boost::shared_ptr<const DistanceFieldCacheEntry> cur = distance_field_cache_entry_;
  if(group_name != cur->group_name_) {
    ROS_INFO_STREAM("No cache entry as group name changed from " << cur->group_name_ << " to " << group_name);
    return ret;
  } 
  // else if(!stateIsTheSame(state, cur->state_)) {
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

void CollisionRobotDistanceField::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                                     collision_detection::CollisionResult& res,
                                                     const planning_models::KinematicState& state,
                                                     const collision_detection::AllowedCollisionMatrix &acm) const
{
  return checkSelfCollisionHelper(req, res, state, &acm);
}

bool CollisionRobotDistanceField::getSelfCollisions(//const collision_detection::CollisionRequest& req,
                                                    //collision_detection::CollisionResult& res,
                                                    const boost::shared_ptr<const CollisionRobotDistanceField::DistanceFieldCacheEntry>& dfce,
                                                    boost::shared_ptr<GroupStateRepresentation>& gsr) const {
  bool in_collision = false;
  for(unsigned int i = 0; i < dfce->link_names_.size(); i++) {
    if(!dfce->link_has_geometry_[i] || !dfce->self_collision_enabled_[i]) continue;
    bool coll = getCollisionSphereCollision(dfce->distance_field_.get(),
                                            gsr->link_body_decompositions_[i]->getCollisionSpheres(),
                                            gsr->link_body_decompositions_[i]->getSphereCenters(),
                                            max_propogation_distance_,
                                            0.0);
    if(coll) {
      ROS_INFO_STREAM("Link " << dfce->link_names_[i] << " in self collision");
      in_collision = true;
      //return true;
    }
  }
  return in_collision;
  //return false;
}  

bool CollisionRobotDistanceField::getSelfProximityGradients(const boost::shared_ptr<const CollisionRobotDistanceField::DistanceFieldCacheEntry>& dfce,
                                                            boost::shared_ptr<GroupStateRepresentation>& gsr) const {
  bool in_collision = false;
  for(unsigned int i = 0; i < dfce->link_names_.size(); i++) {
    if(!dfce->link_has_geometry_[i] || !dfce->self_collision_enabled_[i]) continue;
    bool coll = getCollisionSphereGradients(dfce->distance_field_.get(),
                                            gsr->link_body_decompositions_[i]->getCollisionSpheres(),
                                            gsr->link_body_decompositions_[i]->getSphereCenters(),
                                            gsr->gradients_[i],
                                            collision_distance_field::SELF,
                                            0.0,
                                            false,
                                            max_propogation_distance_,
                                            false);
    if(coll) {
      in_collision = true;
    }
  }
  return in_collision;
} 

bool CollisionRobotDistanceField::getIntraGroupCollisions(const boost::shared_ptr<const CollisionRobotDistanceField::DistanceFieldCacheEntry>& dfce,
                                                          boost::shared_ptr<GroupStateRepresentation>& gsr) const {

  unsigned int num_links = dfce->link_names_.size();
  unsigned int num_attached_bodies = dfce->attached_body_names_.size();
  //TODO - deal with attached bodies
  for(unsigned int i = 0; i < num_links; i++) {
    for(unsigned int j = i+1; j < num_links; j++) {    
      if(i == j) continue;
      if(!dfce->link_has_geometry_[i] || !dfce->link_has_geometry_[j]) continue;
      if(!dfce->intra_group_collision_enabled_[i][j]) continue;
      if(!doBoundingSpheresIntersect(gsr->link_body_decompositions_[i],
                                     gsr->link_body_decompositions_[j])) {
        ROS_INFO_STREAM("Bounding spheres for " << dfce->link_names_[i] << " and " << dfce->link_names_[j]
                        << " don't intersect");
        continue;
      } else {
        ROS_INFO_STREAM("Bounding spheres for " << dfce->link_names_[i] << " and " << dfce->link_names_[j]
                        << " intersect");
      }
      const std::vector<CollisionSphere>* collision_spheres_1;
      const std::vector<CollisionSphere>* collision_spheres_2;
      const std::vector<Eigen::Vector3d>* sphere_centers_1;
      const std::vector<Eigen::Vector3d>* sphere_centers_2;
      collision_spheres_1 = &(gsr->link_body_decompositions_[i]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->link_body_decompositions_[i]->getSphereCenters());
      collision_spheres_2 = &(gsr->link_body_decompositions_[j]->getCollisionSpheres());
      sphere_centers_2 = &(gsr->link_body_decompositions_[j]->getSphereCenters());
      for(unsigned int k = 0; k < collision_spheres_1->size(); k++) {
        for(unsigned int l = 0; l < collision_spheres_2->size(); l++) {
          Eigen::Vector3d gradient = (*sphere_centers_1)[k] - (*sphere_centers_2)[l];
          double dist = gradient.norm();
          if(dist < (*collision_spheres_1)[k].radius_+(*collision_spheres_2)[l].radius_) {
            ROS_INFO_STREAM("Sphere 1 " << (*sphere_centers_1)[k]);
            ROS_INFO_STREAM("Sphere 2 " << (*sphere_centers_2)[l]);
            ROS_INFO_STREAM("Norm " << gradient.norm());
            ROS_INFO_STREAM("Dist is " << dist 
                            << " radius 1 " << (*collision_spheres_1)[k].radius_ 
                            << " radius 2 " << (*collision_spheres_2)[l].radius_);
            ROS_INFO_STREAM("Gradient " << gradient);
            ROS_INFO_STREAM("Spheres intersect for " << dfce->link_names_[i] << " and " << dfce->link_names_[j]);
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool CollisionRobotDistanceField::getIntraGroupProximityGradients(const boost::shared_ptr<const CollisionRobotDistanceField::DistanceFieldCacheEntry>& dfce,
                                                                  boost::shared_ptr<GroupStateRepresentation>& gsr) const {
  bool in_collision = false;
  unsigned int num_links = dfce->link_names_.size();
  unsigned int num_attached_bodies = dfce->attached_body_names_.size();
  //TODO - deal with attached bodies
  for(unsigned int i = 0; i < num_links; i++) {
    for(unsigned int j = i+1; j < num_links; j++) {    
      if(i == j) continue;
      if(!dfce->link_has_geometry_[i] || !dfce->link_has_geometry_[j]) continue;
      if(!dfce->intra_group_collision_enabled_[i][j]) continue;
      const std::vector<CollisionSphere>* collision_spheres_1;
      const std::vector<CollisionSphere>* collision_spheres_2;
      const std::vector<Eigen::Vector3d>* sphere_centers_1;
      const std::vector<Eigen::Vector3d>* sphere_centers_2;
      collision_spheres_1 = &(gsr->link_body_decompositions_[i]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->link_body_decompositions_[i]->getSphereCenters());
      collision_spheres_2 = &(gsr->link_body_decompositions_[j]->getCollisionSpheres());
      sphere_centers_2 = &(gsr->link_body_decompositions_[j]->getSphereCenters());
      for(unsigned int k = 0; k < collision_spheres_1->size(); k++) {
        for(unsigned int l = 0; l < collision_spheres_2->size(); l++) {
          Eigen::Vector3d gradient = (*sphere_centers_1)[k] - (*sphere_centers_2)[l];
          double dist = gradient.norm();
          if(dist < gsr->gradients_[i].distances[k]) {
            //ROS_INFO_STREAM("Gradient " << gradient);
            //ROS_INFO_STREAM("Dist " << dist);
            gsr->gradients_[i].distances[k] = dist;
            gsr->gradients_[i].gradients[k] = gradient;
            gsr->gradients_[i].types[k] = INTRA;
          }
          if(dist < gsr->gradients_[j].distances[l]) {
            //ROS_INFO_STREAM("Gradient sec " << gradient);
            //ROS_INFO_STREAM("Dist sec " << dist);
            gsr->gradients_[j].distances[l] = dist;
            gsr->gradients_[j].gradients[l] = -gradient;
            gsr->gradients_[j].types[l] = INTRA;
          }
        }
      }
    }
  }
  return in_collision;
}
boost::shared_ptr<CollisionRobotDistanceField::DistanceFieldCacheEntry> 
CollisionRobotDistanceField::generateDistanceFieldCacheEntry(const std::string& group_name,
                                                             const planning_models::KinematicState& state,
                                                             const collision_detection::AllowedCollisionMatrix *acm,
                                                             bool generate_distance_field) const
{ 
  ros::WallTime n = ros::WallTime::now();
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
  //generateAllowedCollisionInformation(dfce);
  dfce->link_names_ = kmodel_->getJointModelGroup(group_name)->getUpdatedLinkModelNames();
  std::vector<bool> all_true(dfce->link_names_.size(), true);
  std::vector<bool> all_false(dfce->link_names_.size(), false);
  const std::vector<planning_models::KinematicState::LinkState*>& lsv = state.getLinkStateVector();
  for(unsigned int i = 0; i < dfce->link_names_.size(); i++) {
    std::string link_name = dfce->link_names_[i];
    const planning_models::KinematicState::LinkState* link_state = dfce->state_->getLinkState(link_name);
    if(link_state->getLinkModel()->getShape()) {
      dfce->link_has_geometry_.push_back(true);
      dfce->link_body_indices_.push_back(link_body_decomposition_index_map_.find(link_name)->second);
      if(acm) {
        collision_detection::AllowedCollision::Type t;
        if(!acm->getEntry(link_name, link_name, t)) {
          dfce->self_collision_enabled_.push_back(true);
        }
        if(t == collision_detection::AllowedCollision::ALWAYS) {
          dfce->self_collision_enabled_.push_back(false);
        }
        std::vector<bool> intra_entries = all_true;
        for(unsigned int j = 0; j < dfce->link_names_.size(); j++) {
          if(link_name == dfce->link_names_[j]) {
            intra_entries[j] = false;
            continue;
          } 
          if(acm->getEntry(link_name, dfce->link_names_[j], t)) {
            if(t == collision_detection::AllowedCollision::ALWAYS) {
              //ROS_INFO_STREAM("Disabling collisions between " << link_name << " and " << dfce->link_names_[j]);
              intra_entries[j] = false;
            }
          }
        }
        dfce->intra_group_collision_enabled_.push_back(intra_entries);
      } else {
        dfce->self_collision_enabled_.push_back(true);
        dfce->intra_group_collision_enabled_.push_back(all_true);
      }
    } else {
      dfce->link_has_geometry_.push_back(false);
      dfce->link_body_indices_.push_back(0);
      dfce->self_collision_enabled_.push_back(false);
      dfce->intra_group_collision_enabled_.push_back(all_false);
    }
    bool found = false;
    for(unsigned int j = 0; j < lsv.size(); j++) {
      if(lsv[j]->getName() == link_name) {
        dfce->link_state_indices_.push_back(j);
        found = true;
        break;
      }
    }
    if(!found) {
      ROS_INFO_STREAM("No link state found for link " << dfce->link_names_[i]);
      return dfce;
    }
    std::vector<const planning_models::KinematicState::AttachedBody*> attached_bodies;
    link_state->getAttachedBodies(attached_bodies);
    for(unsigned int i = 0; i < attached_bodies.size(); i++) {
      dfce->attached_body_names_.push_back(attached_bodies[i]->getName());
      dfce->attached_body_link_state_indices_.push_back(dfce->link_state_indices_[i]);
    }
  }
  if(generate_distance_field) {
    std::vector<PosedBodyPointDecompositionPtr> non_group_link_decompositions;
    std::vector<PosedBodyPointDecompositionVectorPtr> non_group_attached_body_decompositions;
    const std::map<std::string, bool>& updated_group_map = in_group_update_map_.find(group_name)->second;
    for(unsigned int i = 0; i < kmodel_->getLinkModelsWithCollisionGeometry().size(); i++) {
      std::string link_name = kmodel_->getLinkModelsWithCollisionGeometry()[i]->getName();
      const planning_models::KinematicState::LinkState* link_state = dfce->state_->getLinkState(link_name);
      if(updated_group_map.find(link_name) != updated_group_map.end()) continue;
      non_group_link_decompositions.push_back(getPosedLinkBodyPointDecomposition(link_state));
      //ROS_INFO_STREAM("Adding link " << link_name << " with " << non_group_link_decompositions.back()->getCollisionPoints().size() << " points");
      std::vector<const planning_models::KinematicState::AttachedBody*> attached_bodies;
      link_state->getAttachedBodies(attached_bodies);
      for(unsigned int i = 0; i < attached_bodies.size(); i++) {
        non_group_attached_body_decompositions.push_back(getAttachedBodyPointDecomposition(attached_bodies[i], resolution_));
      }
    }
    ros::WallTime before_create = ros::WallTime::now();
    if(use_signed_distance_field_)
    {
      dfce->distance_field_.reset(new distance_field::SignedPropagationDistanceField(size_x_,
                                                                                     size_y_, 
                                                                                     size_z_, 
                                                                                     resolution_, 
                                                                                     -(size_x_/2.0), 
                                                                                     -(size_y_/2.0), 
                                                                                     -(size_z_/2.0), 
                                                                                     max_propogation_distance_));
    }
    else
    {
      ROS_INFO_STREAM("This one");
      dfce->distance_field_.reset(new distance_field::PropagationDistanceField(size_x_,
                                                                               size_y_, 
                                                                               size_z_, 
                                                                               resolution_, 
                                                                               -(size_x_/2.0), 
                                                                               -(size_y_/2.0), 
                                                                               -(size_z_/2.0), 
                                                                               max_propogation_distance_));
    }
    ROS_INFO_STREAM("Creation took " << (ros::WallTime::now()-before_create).toSec());
    //TODO - deal with AllowedCollisionMatrix
    //now we need to actually set the points
    //TODO - deal with shifted robot
    std::vector<Eigen::Vector3d> all_points;
    for(unsigned int i = 0; i < non_group_link_decompositions.size(); i++) {
      all_points.insert(all_points.end(),
                        non_group_link_decompositions[i]->getCollisionPoints().begin(),
                        non_group_link_decompositions[i]->getCollisionPoints().end());
    }
    for(unsigned int i = 0; i < non_group_attached_body_decompositions.size(); i++) {
      all_points.insert(all_points.end(),
                        non_group_attached_body_decompositions[i]->getCollisionPoints().begin(),
                        non_group_attached_body_decompositions[i]->getCollisionPoints().end());
    }
    //ROS_INFO_STREAM("Pre-dim " << dfce->distance_field_->getNumCells(distance_field::VoxelGrid<distance_field::PropDistanceFieldVoxel>::DIM_X));
    ros::WallTime before_add = ros::WallTime::now();  
    dfce->distance_field_->addPointsToField(all_points);
    ROS_INFO_STREAM("Adding points took " << (ros::WallTime::now()-before_add).toSec());
    ROS_INFO_STREAM("Total is " << (ros::WallTime::now()-n).toSec());
    //ROS_INFO_STREAM("Pre-dim " << dfce->distance_field_->getNumCells(distance_field::VoxelGrid<distance_field::PropDistanceFieldVoxel>::DIM_X));
    //ROS_INFO_STREAM("Creation took " << (ros::WallTime::now()-n));
  }
  return dfce;
}

void CollisionRobotDistanceField::addLinkBodyDecompositions(double resolution) 
{
  const std::vector<planning_models::KinematicModel::LinkModel*>& link_models = kmodel_->getLinkModelsWithCollisionGeometry();
  for(unsigned int i = 0; i < link_models.size(); i++) {
    if(!link_models[i]->getShape()) {
      ROS_WARN_STREAM("No collision geometry for link model " << link_models[i]->getName() << " though there should be");
      continue;
    }
    ROS_DEBUG_STREAM("Generating model for " << link_models[i]->getName());
    BodyDecompositionConstPtr bd(new BodyDecomposition(link_models[i]->getShape(), resolution, resolution));
    link_body_decomposition_vector_.push_back(bd);
    link_body_decomposition_index_map_[link_models[i]->getName()] = link_body_decomposition_vector_.size()-1;
  }
}

PosedBodySphereDecompositionPtr 
CollisionRobotDistanceField::getPosedLinkBodySphereDecomposition(const planning_models::KinematicState::LinkState* ls,
                                                                 unsigned int ind) const {
  PosedBodySphereDecompositionPtr ret;
  ret.reset(new PosedBodySphereDecomposition(link_body_decomposition_vector_[ind]));
  ret->updatePose(ls->getGlobalCollisionBodyTransform());
  return ret;
}

PosedBodyPointDecompositionPtr 
CollisionRobotDistanceField::getPosedLinkBodyPointDecomposition(const planning_models::KinematicState::LinkState* ls) const
{
  PosedBodyPointDecompositionPtr ret;
  std::map<std::string, unsigned int>::const_iterator it = link_body_decomposition_index_map_.find(ls->getName());
  if(it == link_body_decomposition_index_map_.end()) {
    ROS_ERROR_STREAM("No link body decomposition for link " << ls->getName());
    return ret;
  }
  ret.reset(new PosedBodyPointDecomposition(link_body_decomposition_vector_[it->second]));
  ret->updatePose(ls->getGlobalCollisionBodyTransform());
  return ret;
}

boost::shared_ptr<CollisionRobotDistanceField::GroupStateRepresentation> 
CollisionRobotDistanceField::getGroupStateRepresentation(const boost::shared_ptr<const DistanceFieldCacheEntry>& dfce, 
                                                         const planning_models::KinematicState& state) const
{
  boost::shared_ptr<GroupStateRepresentation> gsr(new GroupStateRepresentation());
  gsr->gradients_.resize(dfce->link_names_.size()+dfce->attached_body_names_.size());
  for(unsigned int i = 0; i < dfce->link_names_.size(); i++) {
    if(dfce->link_has_geometry_[i]) {
      const planning_models::KinematicState::LinkState* ls = state.getLinkStateVector()[dfce->link_state_indices_[i]];
      gsr->link_body_decompositions_.push_back(getPosedLinkBodySphereDecomposition(ls, dfce->link_body_indices_[i]));
      gsr->gradients_[i].types.resize(gsr->link_body_decompositions_.back()->getCollisionSpheres().size());
      gsr->gradients_[i].distances.resize(gsr->link_body_decompositions_.back()->getCollisionSpheres().size(), DBL_MAX);
      gsr->gradients_[i].gradients.resize(gsr->link_body_decompositions_.back()->getCollisionSpheres().size());
    } else {
      PosedBodySphereDecompositionPtr emp;
      gsr->link_body_decompositions_.push_back(emp);
    }
  }
  for(unsigned int i = 0; i < dfce->attached_body_names_.size(); i++) {
    const planning_models::KinematicState::LinkState* ls = state.getLinkStateVector()[dfce->attached_body_link_state_indices_[i]];
    gsr->attached_body_decompositions_.push_back(getAttachedBodySphereDecomposition(ls->getAttachedBody(dfce->attached_body_names_[i]), resolution_));
  }
  return gsr;
}

// void CollisionRobotDistanceField::generateAllowedCollisionInformation(boost::shared_ptr<CollisionRobotDistanceField::DistanceFieldCacheEntry>& dfce)
// {
//   for(unsigned int i = 0; i < dfce.link_names_.size(); i++) {
//     for(unsigned int j = 0; j < 
//     if(dfce->acm.find
//   }
// }

}  
