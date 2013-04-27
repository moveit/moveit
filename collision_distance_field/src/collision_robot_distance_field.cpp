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

/* Author: E. Gil Jones */

#include <moveit/collision_distance_field/collision_robot_distance_field.h>
#include <moveit/collision_distance_field/collision_common_distance_field.h>
#include <moveit/distance_field/propagation_distance_field.h>

namespace collision_detection
{

CollisionRobotDistanceField::CollisionRobotDistanceField(const robot_model::RobotModelConstPtr& kmodel)
  : CollisionRobot(kmodel)
{  
}

CollisionRobotDistanceField::CollisionRobotDistanceField(const robot_model::RobotModelConstPtr& kmodel, 
                                                         const std::map<std::string, std::vector<CollisionSphere> >& link_body_decompositions,
                                                         double size_x, 
                                                         double size_y,
                                                         double size_z,
                                                         bool use_signed_distance_field,
                                                         double resolution,
                                                         double collision_tolerance,
                                                         double max_propogation_distance,
                                                         double padding, 
                                                         double scale)
  : CollisionRobot(kmodel, padding, scale)
{  
  initialize(link_body_decompositions, size_x, size_y, size_z, use_signed_distance_field, resolution, collision_tolerance, max_propogation_distance);
}

CollisionRobotDistanceField::CollisionRobotDistanceField(const CollisionRobotDistanceField& other) :
  CollisionRobot(other)
{
  size_x_ = other.size_x_;
  size_y_ = other.size_y_;
  size_z_ = other.size_z_;
  use_signed_distance_field_ = other.use_signed_distance_field_;
  resolution_ = other.resolution_;
  collision_tolerance_ = other.collision_tolerance_;
  max_propogation_distance_ = other.max_propogation_distance_;
  link_body_decomposition_vector_ = other.link_body_decomposition_vector_;
  link_body_decomposition_index_map_ = other.link_body_decomposition_index_map_;
  in_group_update_map_ = other.in_group_update_map_;
  pregenerated_group_state_representation_map_ = other.pregenerated_group_state_representation_map_;
}

void CollisionRobotDistanceField::initialize(const std::map<std::string, std::vector<CollisionSphere> >& link_body_decompositions,
                                             double size_x, 
                                             double size_y,
                                             double size_z,
                                             bool use_signed_distance_field,
                                             double resolution,
                                             double collision_tolerance,
                                             double max_propogation_distance)
{
  size_x_ = size_x;
  size_y_ = size_y;
  size_z_ = size_z;
  use_signed_distance_field_ = use_signed_distance_field;
  resolution_ = resolution;
  collision_tolerance_ = collision_tolerance;
  max_propogation_distance_ = max_propogation_distance;
  addLinkBodyDecompositions(resolution_, link_body_decompositions);  
  robot_state::RobotState state(kmodel_);
  const std::map<std::string, robot_model::JointModelGroup*>& jmgm = kmodel_->getJointModelGroupMap();
  for(std::map<std::string, robot_model::JointModelGroup*>::const_iterator it = jmgm.begin();
      it != jmgm.end();
      it++) {
    std::map<std::string, bool> updated_group_entry;
    for(unsigned int i = 0; i < it->second->getUpdatedLinkModelsWithGeometryNames().size(); i++) {
      updated_group_entry[it->second->getUpdatedLinkModelsWithGeometryNames()[i]] = true;
    }
    in_group_update_map_[it->first] = updated_group_entry;
    boost::shared_ptr<DistanceFieldCacheEntry> dfce = generateDistanceFieldCacheEntry(it->first,
                                                                                      state,
                                                                                      NULL,
                                                                                      false);
    getGroupStateRepresentation(dfce, state, pregenerated_group_state_representation_map_[it->first]);
  }
}

void CollisionRobotDistanceField::generateCollisionCheckingStructures(const std::string& group_name,
                                                                      const robot_state::RobotState& state,
                                                                      const collision_detection::AllowedCollisionMatrix *acm,
                                                                      boost::shared_ptr<GroupStateRepresentation>& gsr, 
                                                                      bool generate_distance_field) const
{
  boost::shared_ptr<const DistanceFieldCacheEntry> dfce = getDistanceFieldCacheEntry(group_name,
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
  //ros::WallTime n = ros::WallTime::now();
  getGroupStateRepresentation(dfce,state, gsr);
  //ROS_INFO_STREAM("Gsr creation " << (ros::WallTime::now()-n).toSec());
  logDebug("Generated group state representation");
}

void CollisionRobotDistanceField::checkSelfCollisionHelper(const collision_detection::CollisionRequest& req,
                                                           collision_detection::CollisionResult& res,
                                                           const robot_state::RobotState& state,
                                                           const collision_detection::AllowedCollisionMatrix *acm,
                                                           boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  if(!gsr) {
    generateCollisionCheckingStructures(req.group_name,
                                        state,
                                        acm,
                                        gsr,
                                        true);
  } else {
    updateGroupStateRepresentationState(state, gsr);
  }
  //ros::WallTime n = ros::WallTime::now();
  bool done = getSelfCollisions(req, res, gsr);
  //std::cerr << "Self collision " << res.collision << std::endl;
  if(!done) {
    getIntraGroupCollisions(req, res, gsr);
  }
  //ROS_INFO_STREAM("Getting self proximity took " << (ros::WallTime::now()-n).toSec());
  //(const_cast<CollisionRobotDistanceField*>(this))->last_gsr_ = gsr;
}

boost::shared_ptr<const DistanceFieldCacheEntry> 
CollisionRobotDistanceField::getDistanceFieldCacheEntry(const std::string& group_name,
                                                        const robot_state::RobotState& state,
                                                        const collision_detection::AllowedCollisionMatrix *acm) const
{ 
  boost::shared_ptr<const DistanceFieldCacheEntry> ret;
  if(!distance_field_cache_entry_) {
    //ROS_WARN_STREAM("No current dfce");
    return ret;
  }
  boost::shared_ptr<const DistanceFieldCacheEntry> cur = distance_field_cache_entry_;
  if(group_name != cur->group_name_) {
    logDebug("No cache entry as group name changed from %s to %s", cur->group_name_.c_str(), group_name.c_str());
    return ret;
  } else if(!compareCacheEntryToState(cur, state)) {
    logDebug("Regenerating distance field as state has changed from last time");
    return ret;
  } else if(acm && !compareCacheEntryToAllowedCollisionMatrix(cur, *acm)) {
    logDebug("Regenerating distance field as some relevant part of the acm changed");    
    return ret;
  }
  return cur;
}

void CollisionRobotDistanceField::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                                     collision_detection::CollisionResult& res,
                                                     const robot_state::RobotState& state) const
{
  boost::shared_ptr<GroupStateRepresentation> gsr;
  checkSelfCollisionHelper(req, res, state, NULL, gsr);
}

void CollisionRobotDistanceField::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                                     collision_detection::CollisionResult& res,
                                                     const robot_state::RobotState& state,
                                                     boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  checkSelfCollisionHelper(req, res, state, NULL, gsr);
}

void CollisionRobotDistanceField::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                                     collision_detection::CollisionResult& res,
                                                     const robot_state::RobotState& state,
                                                     const collision_detection::AllowedCollisionMatrix &acm) const
{
  boost::shared_ptr<GroupStateRepresentation> gsr;
  checkSelfCollisionHelper(req, res, state, &acm, gsr);
}

void CollisionRobotDistanceField::checkSelfCollision(const collision_detection::CollisionRequest& req,
                                                     collision_detection::CollisionResult& res,
                                                     const robot_state::RobotState& state,
                                                     const collision_detection::AllowedCollisionMatrix &acm,
                                                     boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  if(gsr) {
    logWarn("Shouldn't be calling this function with initialized gsr - ACM will be ignored");
  } 
  checkSelfCollisionHelper(req, res, state, &acm, gsr);
}

bool CollisionRobotDistanceField::getSelfCollisions(const collision_detection::CollisionRequest& req,
                                                    collision_detection::CollisionResult& res,
                                                    boost::shared_ptr<GroupStateRepresentation>& gsr) const {
  for(unsigned int i = 0; i < gsr->dfce_->link_names_.size()+gsr->dfce_->attached_body_names_.size(); i++) {
    bool is_link = i < gsr->dfce_->link_names_.size();
    if((is_link && !gsr->dfce_->link_has_geometry_[i]) || !gsr->dfce_->self_collision_enabled_[i]) continue;
    const std::vector<CollisionSphere>* collision_spheres_1;
    const EigenSTL::vector_Vector3d* sphere_centers_1;
    if(is_link) {
      collision_spheres_1 = &(gsr->link_body_decompositions_[i]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->link_body_decompositions_[i]->getSphereCenters());
    } else {
      collision_spheres_1 = &(gsr->attached_body_decompositions_[i-gsr->dfce_->link_names_.size()]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->attached_body_decompositions_[i-gsr->dfce_->link_names_.size()]->getSphereCenters());
    }
    if(req.contacts) {
      std::vector<unsigned int> colls;
      bool coll = getCollisionSphereCollision(gsr->dfce_->distance_field_.get(),
                                              *collision_spheres_1,
                                              *sphere_centers_1,
                                              max_propogation_distance_,
                                              0.0,
                                              std::min(req.max_contacts_per_pair, req.max_contacts-res.contact_count),
                                              colls);
      if(coll) {
        res.collision = true;
        for(unsigned int j = 0; j < colls.size(); j++) {
          collision_detection::Contact con;
          if(is_link) {
            con.pos = gsr->link_body_decompositions_[i]->getSphereCenters()[colls[j]];
            con.body_type_1 = collision_detection::BodyTypes::ROBOT_LINK;
            con.body_name_1 = gsr->dfce_->link_names_[i];
          } else {
            con.pos = gsr->attached_body_decompositions_[i-gsr->dfce_->link_names_.size()]->getSphereCenters()[colls[j]];
            con.body_type_1 = collision_detection::BodyTypes::ROBOT_ATTACHED;
            con.body_name_1 = gsr->dfce_->attached_body_names_[i-gsr->dfce_->link_names_.size()];
          }
          //std::cerr << "Self collision with " << con.body_name_1 << std::endl;
          con.body_type_2 = collision_detection::BodyTypes::ROBOT_LINK;
          con.body_name_2 = "self";
          res.contact_count++;
          res.contacts[std::pair<std::string,std::string>(con.body_name_1, con.body_name_2)].push_back(con);
          gsr->gradients_[i].types[colls[j]] = SELF;
        }
        gsr->gradients_[i].collision = true;
        if(res.contact_count >= req.max_contacts) {
          return true;
        }
      }
    } else {
      bool coll = getCollisionSphereCollision(gsr->dfce_->distance_field_.get(),
                                              *collision_spheres_1,
                                              *sphere_centers_1,
                                              max_propogation_distance_,
                                              0.0);
      if(coll) {
        logDebug("Link %s in self collision", gsr->dfce_->link_names_[i].c_str());
        //if(is_link) {
        //   std::cerr << "Link " << gsr->dfce_->link_names_[i] << " in self collision" << std::endl;
        // } else {
        //   std::cerr << "Attached body in self collision" << std::endl;
        // }
        res.collision = true;
        return true;
      }
    }
  }
  return (res.contact_count >= req.max_contacts);
}  

bool CollisionRobotDistanceField::getSelfProximityGradients(boost::shared_ptr<GroupStateRepresentation>& gsr) const {
  bool in_collision = false;
  for(unsigned int i = 0; i < gsr->dfce_->link_names_.size(); i++) {
    bool is_link = i < gsr->dfce_->link_names_.size();
    if((is_link && !gsr->dfce_->link_has_geometry_[i]) || !gsr->dfce_->self_collision_enabled_[i]) continue;
    const std::vector<CollisionSphere>* collision_spheres_1;
    const EigenSTL::vector_Vector3d* sphere_centers_1;
    if(is_link) {
      collision_spheres_1 = &(gsr->link_body_decompositions_[i]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->link_body_decompositions_[i]->getSphereCenters());
    } else {
      collision_spheres_1 = &(gsr->attached_body_decompositions_[i-gsr->dfce_->link_names_.size()]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->attached_body_decompositions_[i-gsr->dfce_->link_names_.size()]->getSphereCenters());
    }
    bool coll = getCollisionSphereGradients(gsr->dfce_->distance_field_.get(),
                                            *collision_spheres_1,
                                            *sphere_centers_1,
                                            gsr->gradients_[i],
                                            collision_detection::SELF,
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

bool CollisionRobotDistanceField::getIntraGroupCollisions(const collision_detection::CollisionRequest& req,
                                                          collision_detection::CollisionResult& res,
                                                          boost::shared_ptr<GroupStateRepresentation>& gsr) const 
{
  unsigned int num_links = gsr->dfce_->link_names_.size();
  unsigned int num_attached_bodies = gsr->dfce_->attached_body_names_.size();
  for(unsigned int i = 0; i < num_links+num_attached_bodies; i++) {
    for(unsigned int j = i+1; j < num_links+num_attached_bodies; j++) {    
      if(i == j) continue;
      bool i_is_link = i < num_links;
      bool j_is_link = j < num_links;
      if((i_is_link && !gsr->dfce_->link_has_geometry_[i]) || (j_is_link && !gsr->dfce_->link_has_geometry_[j])) continue;
      if(!gsr->dfce_->intra_group_collision_enabled_[i][j]) continue;
      if(i_is_link && j_is_link && !doBoundingSpheresIntersect(gsr->link_body_decompositions_[i],
                                                               gsr->link_body_decompositions_[j])) {
        //ROS_DEBUG_STREAM("Bounding spheres for " << gsr->dfce_->link_names_[i] << " and " << gsr->dfce_->link_names_[j]
        //<< " don't intersect");
        continue;
      } else if(!i_is_link || !j_is_link) {
        bool all_ok = true;
        if(!i_is_link && j_is_link) {
          for(unsigned int k = 0; k < gsr->attached_body_decompositions_[i-num_links]->getSize(); k++) {
            if(doBoundingSpheresIntersect(gsr->link_body_decompositions_[j], 
                                          gsr->attached_body_decompositions_[i-num_links]->getPosedBodySphereDecomposition(k))) {
              all_ok = false;
              break;
            }
          } 
        } else if(i_is_link && !j_is_link) {
          for(unsigned int k = 0; k < gsr->attached_body_decompositions_[j-num_links]->getSize(); k++) {
            if(doBoundingSpheresIntersect(gsr->link_body_decompositions_[i], 
                                          gsr->attached_body_decompositions_[j-num_links]->getPosedBodySphereDecomposition(k))) {
              all_ok = false;
              break;
            }
          } 
        } else {
          for(unsigned int k = 0; k < gsr->attached_body_decompositions_[i-num_links]->getSize() && all_ok; k++) {
            for(unsigned int l = 0; l < gsr->attached_body_decompositions_[j-num_links]->getSize(); l++) {
              if(doBoundingSpheresIntersect(gsr->attached_body_decompositions_[i-num_links]->getPosedBodySphereDecomposition(k),
                                            gsr->attached_body_decompositions_[j-num_links]->getPosedBodySphereDecomposition(l))) {
                
                all_ok = false;
                break;
              }
            }
          }
        }
        if(all_ok) {
          continue;
        }
        // std::cerr << "Bounding spheres for " << gsr->dfce_->link_names_[i] << " and " << gsr->dfce_->link_names_[j]
        //           << " intersect" << std::endl;
      }
      int num_pair = -1;
      std::string name_1;
      std::string name_2;
      if(i_is_link) {
        name_1 = gsr->dfce_->link_names_[i];
      } else {
        name_1 = gsr->dfce_->attached_body_names_[i-num_links];        
      }
      if(j_is_link) {
        name_2 = gsr->dfce_->link_names_[j];
      } else {
        name_2 = gsr->dfce_->attached_body_names_[j-num_links];        
      }
      if(req.contacts) {
        collision_detection::CollisionResult::ContactMap::iterator it = res.contacts.find(std::pair<std::string,std::string>(name_1, name_2));
        if(it == res.contacts.end()) {
          num_pair = 0;
        } else {
          num_pair = it->second.size();
        }
      }
      const std::vector<CollisionSphere>* collision_spheres_1;
      const std::vector<CollisionSphere>* collision_spheres_2;
      const EigenSTL::vector_Vector3d* sphere_centers_1;
      const EigenSTL::vector_Vector3d* sphere_centers_2;
      if(i_is_link) {
        collision_spheres_1 = &(gsr->link_body_decompositions_[i]->getCollisionSpheres());
        sphere_centers_1 = &(gsr->link_body_decompositions_[i]->getSphereCenters());
      } else {
        collision_spheres_1 = &(gsr->attached_body_decompositions_[i-num_links]->getCollisionSpheres());
        sphere_centers_1 = &(gsr->attached_body_decompositions_[i-num_links]->getSphereCenters());
      }
      if(j_is_link) {
        collision_spheres_2 = &(gsr->link_body_decompositions_[j]->getCollisionSpheres());
        sphere_centers_2 = &(gsr->link_body_decompositions_[j]->getSphereCenters());
      } else {
        collision_spheres_2 = &(gsr->attached_body_decompositions_[j-num_links]->getCollisionSpheres());
        sphere_centers_2 = &(gsr->attached_body_decompositions_[j-num_links]->getSphereCenters());
      }
      for(unsigned int k = 0; k < collision_spheres_1->size() && num_pair < (int)req.max_contacts_per_pair; k++) {
        for(unsigned int l = 0; l < collision_spheres_2->size() && num_pair < (int)req.max_contacts_per_pair; l++) {
          Eigen::Vector3d gradient = (*sphere_centers_1)[k] - (*sphere_centers_2)[l];
          double dist = gradient.norm();
          //std::cerr << "Dist is " << dist << " rad " << (*collision_spheres_1)[k].radius_+(*collision_spheres_2)[l].radius_ << std::endl;
          if(dist < (*collision_spheres_1)[k].radius_+(*collision_spheres_2)[l].radius_) {
            logDebug("Intra-group contact between %s and %s", name_1.c_str(), name_2.c_str());
            res.collision = true;
            if(req.contacts) {
              collision_detection::Contact con;
              con.pos = gsr->link_body_decompositions_[i]->getSphereCenters()[k];
              con.body_name_1 = name_1;
              con.body_name_2 = name_2;
              if(i_is_link) {
                con.body_type_1 = collision_detection::BodyTypes::ROBOT_LINK;
              } else {
                con.body_type_1 = collision_detection::BodyTypes::ROBOT_ATTACHED;
              }
              if(j_is_link) {
                con.body_type_2 = collision_detection::BodyTypes::ROBOT_LINK;
              } else {
                con.body_type_2 = collision_detection::BodyTypes::ROBOT_ATTACHED;
              }
              res.contact_count++;
              res.contacts[std::pair<std::string,std::string>(con.body_name_1, con.body_name_2)].push_back(con);
              num_pair++;
              //std::cerr << "Pushing back intra " << con.body_name_1 << " and " << con.body_name_2 << std::endl;
              gsr->gradients_[i].types[k] = INTRA;
              gsr->gradients_[i].collision = true;
              gsr->gradients_[j].types[l] = INTRA;
              gsr->gradients_[j].collision = true;
              // ROS_INFO_STREAM("Sphere 1 " << (*sphere_centers_1)[k]);
              // ROS_INFO_STREAM("Sphere 2 " << (*sphere_centers_2)[l]);
              // ROS_INFO_STREAM("Norm " << gradient.norm());
              // ROS_INFO_STREAM("Dist is " << dist 
              //                 << " radius 1 " << (*collision_spheres_1)[k].radius_ 
              //                 << " radius 2 " << (*collision_spheres_2)[l].radius_);
              // ROS_INFO_STREAM("Gradient " << gradient);
              // ROS_INFO_STREAM("Spheres intersect for " << gsr->dfce_->link_names_[i] << " and " << gsr->dfce_->link_names_[j]);
              //std::cerr << "Spheres intersect for " << gsr->dfce_->link_names_[i] << " and " << gsr->dfce_->link_names_[j] << std::cerr;
              if(res.contact_count >= req.max_contacts) {
                return true;
              }
            } else {
              return true;
            }
          }
        }
      }
    }
  }
  return false;
}

bool CollisionRobotDistanceField::getIntraGroupProximityGradients(boost::shared_ptr<GroupStateRepresentation>& gsr) const {
  bool in_collision = false;
  unsigned int num_links = gsr->dfce_->link_names_.size();
  unsigned int num_attached_bodies = gsr->dfce_->attached_body_names_.size();
  //TODO - deal with attached bodies
  for(unsigned int i = 0; i < num_links+num_attached_bodies; i++) {
    for(unsigned int j = i+1; j < num_links+num_attached_bodies; j++) {    
      if(i == j) continue;
      bool i_is_link = i < num_links;
      bool j_is_link = j < num_links;
      if((i_is_link && !gsr->dfce_->link_has_geometry_[i]) || (j_is_link && !gsr->dfce_->link_has_geometry_[j])) continue;
      if(!gsr->dfce_->intra_group_collision_enabled_[i][j]) continue;
      const std::vector<CollisionSphere>* collision_spheres_1;
      const std::vector<CollisionSphere>* collision_spheres_2;
      const EigenSTL::vector_Vector3d* sphere_centers_1;
      const EigenSTL::vector_Vector3d* sphere_centers_2;
      if(i_is_link) {
        collision_spheres_1 = &(gsr->link_body_decompositions_[i]->getCollisionSpheres());
        sphere_centers_1 = &(gsr->link_body_decompositions_[i]->getSphereCenters());
      } else {
        collision_spheres_1 = &(gsr->attached_body_decompositions_[i-num_links]->getCollisionSpheres());
        sphere_centers_1 = &(gsr->attached_body_decompositions_[i-num_links]->getSphereCenters());
      }
      if(j_is_link) {
        collision_spheres_2 = &(gsr->link_body_decompositions_[j]->getCollisionSpheres());
        sphere_centers_2 = &(gsr->link_body_decompositions_[j]->getSphereCenters());
      } else {
        collision_spheres_2 = &(gsr->attached_body_decompositions_[j-num_links]->getCollisionSpheres());
        sphere_centers_2 = &(gsr->attached_body_decompositions_[j-num_links]->getSphereCenters());
      }
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
boost::shared_ptr<DistanceFieldCacheEntry> 
CollisionRobotDistanceField::generateDistanceFieldCacheEntry(const std::string& group_name,
                                                             const robot_state::RobotState& state,
                                                             const collision_detection::AllowedCollisionMatrix *acm,
                                                             bool generate_distance_field) const
{ 
  ros::WallTime n = ros::WallTime::now();
  boost::shared_ptr<DistanceFieldCacheEntry> dfce(new DistanceFieldCacheEntry());
  if(kmodel_->getJointModelGroup(group_name) == NULL) {
    logWarn("No group %s", group_name.c_str());
    return dfce;
  }
  dfce->group_name_ = group_name;
  dfce->state_.reset(new robot_state::RobotState(state));
  if(acm) {
    dfce->acm_ = *acm;
  }
  //generateAllowedCollisionInformation(dfce);
  dfce->link_names_ = kmodel_->getJointModelGroup(group_name)->getUpdatedLinkModelNames();
  std::vector<const robot_state::AttachedBody*> all_attached_bodies;
  dfce->state_->getAttachedBodies(all_attached_bodies);
  unsigned int att_count = 0;
  //may be bigger than necessary
  std::vector<bool> all_true(dfce->link_names_.size()+all_attached_bodies.size(), true);
  std::vector<bool> all_false(dfce->link_names_.size()+all_attached_bodies.size(), false);
  const std::vector<robot_state::LinkState*>& lsv = state.getLinkStateVector();
  dfce->self_collision_enabled_.resize(dfce->link_names_.size()+all_attached_bodies.size(), true);
  dfce->intra_group_collision_enabled_.resize(dfce->link_names_.size()+all_attached_bodies.size());
  for(unsigned int i = 0; i < dfce->link_names_.size(); i++) {
    std::string link_name = dfce->link_names_[i];
    const robot_state::LinkState* link_state = dfce->state_->getLinkState(link_name);
    bool found = false;
    for(unsigned int j = 0; j < lsv.size(); j++) {
      if(lsv[j]->getName() == link_name) {
        dfce->link_state_indices_.push_back(j);
        found = true;
        break;
      }
    }
    if(!found) {
      logInform("No link state found for link %s", dfce->link_names_[i].c_str());
      return dfce;
    }
    if(link_state->getLinkModel()->getShape()) {
      dfce->link_has_geometry_.push_back(true);
      dfce->link_body_indices_.push_back(link_body_decomposition_index_map_.find(link_name)->second);
      if(acm) {
        collision_detection::AllowedCollision::Type t;
        if(acm->getEntry(link_name, link_name, t) && 
           t == collision_detection::AllowedCollision::ALWAYS) {
          dfce->self_collision_enabled_[i] = false;
        }
        dfce->intra_group_collision_enabled_[i] = all_true;
        for(unsigned int j = i+1; j < dfce->link_names_.size(); j++) {
          if(link_name == dfce->link_names_[j]) {
            dfce->intra_group_collision_enabled_[i][j] = false;
            continue;
          } 
          if(acm->getEntry(link_name, dfce->link_names_[j], t) &&
             t == collision_detection::AllowedCollision::ALWAYS) 
          {
            dfce->intra_group_collision_enabled_[i][j] = false;
          } 
          //else {
          //std::cerr << "Setting not allowed for " << link_name << " and " << dfce->link_names_[j] << std::endl;
          //}
        }
        std::vector<const robot_state::AttachedBody*> link_attached_bodies;
        link_state->getAttachedBodies(link_attached_bodies);
        for(unsigned int j = 0; j < link_attached_bodies.size(); j++, att_count++) {
          dfce->attached_body_names_.push_back(link_attached_bodies[j]->getName());
          dfce->attached_body_link_state_indices_.push_back(dfce->link_state_indices_[i]);
          if(acm->getEntry(link_name, link_attached_bodies[j]->getName(), t)) {
            if(t == collision_detection::AllowedCollision::ALWAYS) {
              dfce->intra_group_collision_enabled_[i][att_count+dfce->link_names_.size()] = false;
            }
          } 
          // std::cerr << "Checking touch links for " << link_name << " and " << attached_bodies[j]->getName() 
          //           << " num " << attached_bodies[j]->getTouchLinks().size() << std::endl;
          //touch links take priority
          if(link_attached_bodies[j]->getTouchLinks().find(link_name) != link_attached_bodies[j]->getTouchLinks().end()) {
            dfce->intra_group_collision_enabled_[i][att_count+dfce->link_names_.size()] = false;
            //std::cerr << "Setting intra group for " << link_name << " and attached body " << link_attached_bodies[j]->getName() << " to false" << std::endl;
          }
        }
      } else {
        dfce->self_collision_enabled_[i] = true;
        dfce->intra_group_collision_enabled_[i] = all_true;
      }
    } else {
      dfce->link_has_geometry_.push_back(false);
      dfce->link_body_indices_.push_back(0);
      dfce->self_collision_enabled_[i] = false;
      dfce->intra_group_collision_enabled_[i] = all_false;
    }
  }
  for(unsigned int i = 0; i < dfce->attached_body_names_.size(); i++) {  
    dfce->intra_group_collision_enabled_[i+dfce->link_names_.size()] = all_true;
    if(acm) {
      collision_detection::AllowedCollision::Type t;
      if(acm->getEntry(dfce->attached_body_names_[i], dfce->attached_body_names_[i], t) &&
         t == collision_detection::AllowedCollision::ALWAYS) {
        dfce->self_collision_enabled_[i+dfce->link_names_.size()] = false;
      }
      for(unsigned int j = i+1; j < dfce->attached_body_names_.size(); j++) {
        if(acm->getEntry(dfce->attached_body_names_[i], dfce->attached_body_names_[j], t) &&
           t == collision_detection::AllowedCollision::ALWAYS) {
          dfce->intra_group_collision_enabled_[i+dfce->link_names_.size()][j+dfce->link_names_.size()] = false;
        } 
        //TODO - allow for touch links to be attached bodies?
        //else {
        //std::cerr << "Setting not allowed for " << link_name << " and " << dfce->link_names_[j] << std::endl;
        //}
      }
    } 
  }
  std::map<std::string, boost::shared_ptr<GroupStateRepresentation> >::const_iterator it = pregenerated_group_state_representation_map_.find(dfce->group_name_);
  if(it != pregenerated_group_state_representation_map_.end()) {
    dfce->pregenerated_group_state_representation_ = it->second;
  }
  std::map<std::string, bool> updated_map;
  const robot_model::JointModel* joint_model = dfce->state_->getLinkState(dfce->link_names_[0])->getLinkModel()->getParentJointModel();  
  std::vector<const robot_model::JointModel*> child_joint_models;
  kmodel_->getChildJointModels(joint_model, child_joint_models);
  for(unsigned int i = 0; i < child_joint_models.size(); i++) {
    updated_map[child_joint_models[i]->getName()] = true;
  }
  for(unsigned int i = 0; i < state.getJointStateVector().size(); i++) {
    if(state.getJointStateVector()[i]->getJointModel()->getMimic()) continue;
    if(updated_map.find(state.getJointStateVector()[i]->getName()) == updated_map.end()) {
      for(unsigned int j = 0; j < state.getJointStateVector()[i]->getVariableCount(); j++) {
        dfce->state_values_.push_back(state.getJointStateVector()[i]->getVariableValues()[j]);
        dfce->state_check_indices_.push_back(dfce->state_values_.size()-1);
        //std::cerr << "Pushing back " << state.getJointStateVector()[i]->getName() << " index " << dfce->state_values_.size()-1 << std::endl;
      }
    } else {
      for(unsigned int j = 0; j < state.getJointStateVector()[i]->getVariableCount(); j++) {
        dfce->state_values_.push_back(state.getJointStateVector()[i]->getVariableValues()[j]);
      }
    }
  }
  if(generate_distance_field) {
    std::vector<PosedBodyPointDecompositionPtr> non_group_link_decompositions;
    std::vector<PosedBodyPointDecompositionVectorPtr> non_group_attached_body_decompositions;
    const std::map<std::string, bool>& updated_group_map = in_group_update_map_.find(group_name)->second;
    for(unsigned int i = 0; i < kmodel_->getLinkModelsWithCollisionGeometry().size(); i++) {
      std::string link_name = kmodel_->getLinkModelsWithCollisionGeometry()[i]->getName();
      const robot_state::LinkState* link_state = dfce->state_->getLinkState(link_name);
      if(updated_group_map.find(link_name) != updated_group_map.end()) continue;
      non_group_link_decompositions.push_back(getPosedLinkBodyPointDecomposition(link_state));
      //ROS_INFO_STREAM("Adding link " << link_name << " with " << non_group_link_decompositions.back()->getCollisionPoints().size() << " points");
      std::vector<const robot_state::AttachedBody*> attached_bodies;
      link_state->getAttachedBodies(attached_bodies);
      for(unsigned int j = 0; j < attached_bodies.size(); j++) {
        //ROS_INFO_STREAM("Adding attached body " << attached_bodies[j]->getName());
        non_group_attached_body_decompositions.push_back(getAttachedBodyPointDecomposition(attached_bodies[j], resolution_));
      }
    }
    //ros::WallTime before_create = ros::WallTime::now();
    if(use_signed_distance_field_)
    {
      dfce->distance_field_.reset(new distance_field::PropagationDistanceField(size_x_,
                                                                               size_y_, 
                                                                               size_z_, 
                                                                               resolution_, 
                                                                               -(size_x_/2.0), 
                                                                               -(size_y_/2.0), 
                                                                               -(size_z_/2.0), 
                                                                               max_propogation_distance_, true));
    }
    else
    {

      dfce->distance_field_.reset(new distance_field::PropagationDistanceField(size_x_,
                                                                               size_y_, 
                                                                               size_z_, 
                                                                               resolution_, 
                                                                               -(size_x_/2.0), 
                                                                               -(size_y_/2.0), 
                                                                               -(size_z_/2.0), 
                                                                               max_propogation_distance_, false));
    }
    //ROS_INFO_STREAM("Creation took " << (ros::WallTime::now()-before_create).toSec());
    //TODO - deal with AllowedCollisionMatrix
    //now we need to actually set the points
    //TODO - deal with shifted robot
    EigenSTL::vector_Vector3d all_points;
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
    //ros::WallTime before_add = ros::WallTime::now();  
    dfce->distance_field_->addPointsToField(all_points);
    
    //ROS_INFO_STREAM("Adding points took " << (ros::WallTime::now()-before_add).toSec());
    //ROS_INFO_STREAM("Total is " << (ros::WallTime::now()-n).toSec());
    //ROS_INFO_STREAM("Pre-dim " << dfce->distance_field_->getNumCells(distance_field::VoxelGrid<distance_field::PropDistanceFieldVoxel>::DIM_X));
    //ROS_INFO_STREAM("Creation took " << (ros::WallTime::now()-n));
  }
  return dfce;
}

void CollisionRobotDistanceField::addLinkBodyDecompositions(double resolution) 
{
  const std::vector<robot_model::LinkModel*>& link_models = kmodel_->getLinkModelsWithCollisionGeometry();
  for(unsigned int i = 0; i < link_models.size(); i++) {
    if(!link_models[i]->getShape()) {
      logWarn("No collision geometry for link model %s though there should be", link_models[i]->getName().c_str());
      continue;
    }
    logDebug("Generating model for %s", link_models[i]->getName().c_str());
    BodyDecompositionConstPtr bd(new BodyDecomposition(link_models[i]->getShape(), resolution, resolution));
    link_body_decomposition_vector_.push_back(bd);
    link_body_decomposition_index_map_[link_models[i]->getName()] = link_body_decomposition_vector_.size()-1;
  }
}

void CollisionRobotDistanceField::addLinkBodyDecompositions(double resolution,
                                                            const std::map<std::string, std::vector<CollisionSphere> >& link_spheres) 
{
  const std::vector<robot_model::LinkModel*>& link_models = kmodel_->getLinkModelsWithCollisionGeometry();
  for(unsigned int i = 0; i < link_models.size(); i++) {
    if(!link_models[i]->getShape()) {
      logWarn("No collision geometry for link model %s though there should be", link_models[i]->getName().c_str());
      continue;
    }
    logDebug("Generating model for %s", link_models[i]->getName().c_str());
    BodyDecompositionPtr bd(new BodyDecomposition(link_models[i]->getShape(), resolution, resolution));
    if(link_spheres.find(link_models[i]->getName()) != link_spheres.end()) {
      bd->replaceCollisionSpheres(link_spheres.find(link_models[i]->getName())->second, Eigen::Affine3d::Identity());
    }
    link_body_decomposition_vector_.push_back(bd);
    link_body_decomposition_index_map_[link_models[i]->getName()] = link_body_decomposition_vector_.size()-1;
  }
}

PosedBodySphereDecompositionPtr 
CollisionRobotDistanceField::getPosedLinkBodySphereDecomposition(const robot_state::LinkState* ls,
                                                                 unsigned int ind) const {
  PosedBodySphereDecompositionPtr ret;
  ret.reset(new PosedBodySphereDecomposition(link_body_decomposition_vector_[ind]));
  ret->updatePose(ls->getGlobalCollisionBodyTransform());
  return ret;
}

PosedBodyPointDecompositionPtr 
CollisionRobotDistanceField::getPosedLinkBodyPointDecomposition(const robot_state::LinkState* ls) const
{
  PosedBodyPointDecompositionPtr ret;
  std::map<std::string, unsigned int>::const_iterator it = link_body_decomposition_index_map_.find(ls->getName());
  if(it == link_body_decomposition_index_map_.end()) {
    logError("No link body decomposition for link %s", ls->getName().c_str());
    return ret;
  }
  ret.reset(new PosedBodyPointDecomposition(link_body_decomposition_vector_[it->second]));
  ret->updatePose(ls->getGlobalCollisionBodyTransform());
  return ret;
}

void CollisionRobotDistanceField::updateGroupStateRepresentationState(const robot_state::RobotState& state,
                                                                      boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  for(unsigned int i = 0; i < gsr->dfce_->link_names_.size(); i++) {
    const robot_state::LinkState* ls = state.getLinkStateVector()[gsr->dfce_->link_state_indices_[i]];
    if(gsr->dfce_->link_has_geometry_[i]) {
      gsr->link_body_decompositions_[i]->updatePose(ls->getGlobalCollisionBodyTransform());
      gsr->gradients_[i].closest_distance = DBL_MAX;
      gsr->gradients_[i].collision = false;
      gsr->gradients_[i].types.assign(gsr->link_body_decompositions_[i]->getCollisionSpheres().size(), NONE);
      gsr->gradients_[i].distances.assign(gsr->link_body_decompositions_[i]->getCollisionSpheres().size(), DBL_MAX);
      gsr->gradients_[i].gradients.assign(gsr->link_body_decompositions_[i]->getCollisionSpheres().size(), Eigen::Vector3d(0.0,0.0,0.0));
      gsr->gradients_[i].sphere_locations = gsr->link_body_decompositions_[i]->getSphereCenters();
    }
  }
  for(unsigned int i = 0; i < gsr->dfce_->attached_body_names_.size(); i++) {
    const robot_state::LinkState* ls = state.getLinkStateVector()[gsr->dfce_->attached_body_link_state_indices_[i]];
    ///std::cerr << "Attached " << dfce->attached_body_names_[i] << " index " << dfce->attached_body_link_state_indices_[i] << std::endl;
    const robot_state::AttachedBody* att = ls->getAttachedBody(gsr->dfce_->attached_body_names_[i]);
    if(!att) {
      logWarn("Attached body discrepancy");
      continue;
    }
    if(gsr->attached_body_decompositions_.size() != att->getShapes().size()) {
      logWarn("Attached body size discrepancy");
      continue;
    }
    for(unsigned int j = 0; j < att->getShapes().size(); j++) {
      gsr->attached_body_decompositions_[i]->updatePose(j, att->getGlobalCollisionBodyTransforms()[j]);
    }
    gsr->gradients_[i+gsr->dfce_->link_names_.size()].closest_distance = DBL_MAX;
    gsr->gradients_[i+gsr->dfce_->link_names_.size()].collision = false;
    gsr->gradients_[i+gsr->dfce_->link_names_.size()].types.assign(gsr->attached_body_decompositions_[i]->getCollisionSpheres().size(), NONE);
    gsr->gradients_[i+gsr->dfce_->link_names_.size()].distances.assign(gsr->attached_body_decompositions_[i]->getCollisionSpheres().size(), DBL_MAX);
    gsr->gradients_[i+gsr->dfce_->link_names_.size()].gradients.assign(gsr->attached_body_decompositions_[i]->getCollisionSpheres().size(), Eigen::Vector3d(0.0,0.0,0.0));
    gsr->gradients_[i+gsr->dfce_->link_names_.size()].sphere_locations = gsr->attached_body_decompositions_[i]->getSphereCenters();
  }
}


void CollisionRobotDistanceField::getGroupStateRepresentation(const boost::shared_ptr<const DistanceFieldCacheEntry>& dfce, 
                                                              const robot_state::RobotState& state,
                                                              boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  if(!dfce->pregenerated_group_state_representation_) {
    //unsigned int count = 0;
    ros::WallTime b = ros::WallTime::now();
    gsr.reset(new GroupStateRepresentation());
    gsr->dfce_ = dfce;
    gsr->gradients_.resize(dfce->link_names_.size()+dfce->attached_body_names_.size());
    for(unsigned int i = 0; i < dfce->link_names_.size(); i++) {
      const robot_state::LinkState* ls = state.getLinkStateVector()[dfce->link_state_indices_[i]];
      if(dfce->link_has_geometry_[i]) {
        gsr->link_body_decompositions_.push_back(getPosedLinkBodySphereDecomposition(ls, dfce->link_body_indices_[i]));
        //std::cerr << dfce->link_names_[i] << " num " << gsr->link_body_decompositions_.back()->getCollisionSpheres().size() << std::endl;
        //count += gsr->link_body_decompositions_.back()->getCollisionSpheres().size();
        gsr->gradients_[i].types.resize(gsr->link_body_decompositions_.back()->getCollisionSpheres().size(), NONE);
        gsr->gradients_[i].distances.resize(gsr->link_body_decompositions_.back()->getCollisionSpheres().size(), DBL_MAX);
        gsr->gradients_[i].gradients.resize(gsr->link_body_decompositions_.back()->getCollisionSpheres().size());
        gsr->gradients_[i].sphere_radii = gsr->link_body_decompositions_.back()->getSphereRadii();
        gsr->gradients_[i].joint_name = ls->getLinkModel()->getParentJointModel()->getName();
      } else {
        PosedBodySphereDecompositionPtr emp;
        gsr->link_body_decompositions_.push_back(emp);
      }
    }
    //std::cerr << "Total count for group " << dfce->group_name_ << " " << count << std::endl;
    //std::cerr << "Initial creation for " << dfce->group_name_ << " took " << (b-ros::WallTime::now()).toSec() << std::endl;
  } else {
    //ros::WallTime b = ros::WallTime::now();
    gsr.reset(new GroupStateRepresentation(*(dfce->pregenerated_group_state_representation_)));
    gsr->dfce_ = dfce;
    //std::cerr << "Copy no update took " << (ros::WallTime::now()-b).toSec() << std::endl;
    gsr->gradients_.resize(dfce->link_names_.size()+dfce->attached_body_names_.size());
    for(unsigned int i = 0; i < dfce->link_names_.size(); i++) {
      const robot_state::LinkState* ls = state.getLinkStateVector()[dfce->link_state_indices_[i]];
      if(dfce->link_has_geometry_[i]) {
        gsr->link_body_decompositions_[i]->updatePose(ls->getGlobalCollisionBodyTransform());
        gsr->gradients_[i].sphere_locations = gsr->link_body_decompositions_[i]->getSphereCenters();
      }
    }
    //std::cerr << "Copy took " << (ros::WallTime::now()-b).toSec() << std::endl;
  }
  for(unsigned int i = 0; i < dfce->attached_body_names_.size(); i++) {
    const robot_state::LinkState* ls = state.getLinkStateVector()[dfce->attached_body_link_state_indices_[i]];
    ///std::cerr << "Attached " << dfce->attached_body_names_[i] << " index " << dfce->attached_body_link_state_indices_[i] << std::endl;
    gsr->attached_body_decompositions_.push_back(getAttachedBodySphereDecomposition(ls->getAttachedBody(dfce->attached_body_names_[i]), resolution_));
    gsr->gradients_[i+dfce->link_names_.size()].types.resize(gsr->attached_body_decompositions_.back()->getCollisionSpheres().size(), NONE);
    gsr->gradients_[i+dfce->link_names_.size()].distances.resize(gsr->attached_body_decompositions_.back()->getCollisionSpheres().size(), DBL_MAX);
    gsr->gradients_[i+dfce->link_names_.size()].gradients.resize(gsr->attached_body_decompositions_.back()->getCollisionSpheres().size());
    gsr->gradients_[i+dfce->link_names_.size()].sphere_locations = gsr->attached_body_decompositions_.back()->getSphereCenters();
    gsr->gradients_[i+dfce->link_names_.size()].sphere_radii = gsr->attached_body_decompositions_.back()->getSphereRadii();
    gsr->gradients_[i+dfce->link_names_.size()].joint_name = ls->getLinkModel()->getParentJointModel()->getName();
  }
}

bool CollisionRobotDistanceField::compareCacheEntryToState(const boost::shared_ptr<const DistanceFieldCacheEntry>& dfce, 
                                                           const robot_state::RobotState& state) const
{
  std::vector<double> new_state_values;
  state.getStateValues(new_state_values);
  if(dfce->state_values_.size() != new_state_values.size()) {
    logError("State value size mismatch");
    return false;
  }
  for(unsigned int i = 0; i < dfce->state_check_indices_.size(); i++) {
    if(fabs(dfce->state_values_[dfce->state_check_indices_[i]]-new_state_values[dfce->state_check_indices_[i]]) > .0001) {
      // std::cerr << "Relevant state value changed from " << dfce->state_values_[dfce->state_check_indices_[i]] << " to " 
      //           << new_state_values[dfce->state_check_indices_[i]] << std::endl;
      return false;
    }
  }
  std::vector<const robot_state::AttachedBody*> attached_bodies_dfce;
  std::vector<const robot_state::AttachedBody*> attached_bodies_state;
  dfce->state_->getAttachedBodies(attached_bodies_dfce);
  state.getAttachedBodies(attached_bodies_state);
  if(attached_bodies_dfce.size() != attached_bodies_state.size()) {
    return false;
  }
  //TODO - figure all the things that can change
  for(unsigned int i = 0; i < attached_bodies_dfce.size(); i++) {
    if(attached_bodies_dfce[i]->getName() != attached_bodies_state[i]->getName()) {
      return false;
    }
    if(attached_bodies_dfce[i]->getTouchLinks() != attached_bodies_state[i]->getTouchLinks()) {
      return false;
    }
    if(attached_bodies_dfce[i]->getShapes().size() != attached_bodies_state[i]->getShapes().size()) {
      return false;
    }
    for(unsigned int j = 0; j < attached_bodies_dfce[i]->getShapes().size(); j++) {
      if(attached_bodies_dfce[i]->getShapes()[j] != attached_bodies_state[i]->getShapes()[j]) {
        return false;
      }
    }
  }
  return true;
}             

bool CollisionRobotDistanceField::compareCacheEntryToAllowedCollisionMatrix(const boost::shared_ptr<const DistanceFieldCacheEntry>& dfce, 
                                                                            const collision_detection::AllowedCollisionMatrix& acm) const
{
  if(dfce->acm_.getSize() != acm.getSize()) {
    logDebug("Allowed collision matrix size mismatch");
    return false;
  }
  std::vector<const robot_state::AttachedBody*> attached_bodies;
  dfce->state_->getAttachedBodies(attached_bodies);
  for(unsigned int i = 0; i < dfce->link_names_.size(); i++) {
    std::string link_name = dfce->link_names_[i];
    if(dfce->link_has_geometry_[i]) {
      bool self_collision_enabled = true;
      collision_detection::AllowedCollision::Type t;
      if(acm.getEntry(link_name, link_name, t)) {
        if(t == collision_detection::AllowedCollision::ALWAYS) {
          self_collision_enabled = false;
        }
      }
      if(self_collision_enabled != dfce->self_collision_enabled_[i]) {
        //ROS_INFO_STREAM("Self collision for " << link_name << " went from " << dfce->self_collision_enabled_[i] << " to " << self_collision_enabled);
        return false;
      }
      for(unsigned int j = i; j < dfce->link_names_.size(); j++) {
        if(i == j) continue;
        if(dfce->link_has_geometry_[j]) {
          bool intra_collision_enabled = true;
          if(acm.getEntry(link_name, dfce->link_names_[j], t)) {
            if(t == collision_detection::AllowedCollision::ALWAYS) {
              intra_collision_enabled = false;
            }
          }
          if(dfce->intra_group_collision_enabled_[i][j] != intra_collision_enabled) {
            // std::cerr << "Intra collision for " << dfce->link_names_[i] << " " << dfce->link_names_[j]
            //           << " went from " << dfce->intra_group_collision_enabled_[i][j] << " to " << intra_collision_enabled << std::endl;
            return false;
          }
        }
      }
    }
  }
  return true;
}

// void CollisionRobotDistanceField::generateAllowedCollisionInformation(boost::shared_ptr<CollisionRobotDistanceField::DistanceFieldCacheEntry>& dfce)
// {
//   for(unsigned int i = 0; i < dfce.link_names_.size(); i++) {
//     for(unsigned int j = 0; j < 
//     if(dfce->acm.find
//   }
// }

}  
