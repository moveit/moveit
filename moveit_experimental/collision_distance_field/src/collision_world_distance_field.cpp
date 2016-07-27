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

#include <moveit/collision_distance_field/collision_world_distance_field.h>
#include <moveit/collision_distance_field/collision_common_distance_field.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <boost/make_shared.hpp>
#include <boost/bind.hpp>

namespace collision_detection
{

CollisionWorldDistanceField::~CollisionWorldDistanceField()
{
  getWorld()->removeObserver(observer_handle_);
}

CollisionWorldDistanceField::CollisionWorldDistanceField(double size_x, 
                                                         double size_y,
                                                         double size_z,
                                                         bool use_signed_distance_field,
                                                         double resolution,
                                                         double collision_tolerance,
                                                         double max_propogation_distance)
  : CollisionWorld(),
    size_x_(size_x),
    size_y_(size_y),
    size_z_(size_z),
    use_signed_distance_field_(use_signed_distance_field),
    resolution_(resolution),
    collision_tolerance_(collision_tolerance),
    max_propogation_distance_(max_propogation_distance)
{  
  distance_field_cache_entry_ = generateDistanceFieldCacheEntry();

  // request notifications about changes to world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldDistanceField::notifyObjectChange, this, _1, _2));
}

CollisionWorldDistanceField::CollisionWorldDistanceField(const WorldPtr& world,
                                                         double size_x, 
                                                         double size_y,
                                                         double size_z,
                                                         bool use_signed_distance_field,
                                                         double resolution,
                                                         double collision_tolerance,
                                                         double max_propogation_distance) :
    CollisionWorld(world),
    size_x_(size_x),
    size_y_(size_y),
    size_z_(size_z),
    use_signed_distance_field_(use_signed_distance_field),
    resolution_(resolution),
    collision_tolerance_(collision_tolerance),
    max_propogation_distance_(max_propogation_distance)
{  
  distance_field_cache_entry_ = generateDistanceFieldCacheEntry();

  // request notifications about changes to world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldDistanceField::notifyObjectChange, this, _1, _2));
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionWorldDistanceField::CollisionWorldDistanceField(const CollisionWorldDistanceField& other,
                                                         const WorldPtr& world) :
  CollisionWorld(other, world)
{
  size_x_ = other.size_x_;
  size_y_ = other.size_y_;
  size_z_ = other.size_z_;
  use_signed_distance_field_ = other.use_signed_distance_field_;
  resolution_ = other.resolution_;
  collision_tolerance_ = other.collision_tolerance_;
  max_propogation_distance_ = other.max_propogation_distance_;
  distance_field_cache_entry_ = generateDistanceFieldCacheEntry();

  // request notifications about changes to world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldDistanceField::notifyObjectChange, this, _1, _2));
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void CollisionWorldDistanceField::checkCollision(const CollisionRequest &req, 
                                                 CollisionResult &res, 
                                                 const CollisionRobot &robot, 
                                                 const robot_state::RobotState &state) const
{
  boost::shared_ptr<GroupStateRepresentation> gsr; 
  checkCollision(req, 
                 res,
                 robot,
                 state,
                 gsr);
}

void CollisionWorldDistanceField::checkCollision(const CollisionRequest &req,
                                                 CollisionResult &res,
                                                 const CollisionRobot &robot,
                                                 const robot_state::RobotState &state,
                                                 boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  try {
    const CollisionRobotDistanceField& cdr = dynamic_cast<const CollisionRobotDistanceField&>(robot);
    if(!gsr) {
      cdr.generateCollisionCheckingStructures(req.group_name,
                                              state,
                                              NULL,
                                              gsr,
                                              true);
    } else {
      cdr.updateGroupStateRepresentationState(state, gsr);
    }
    bool done = cdr.getSelfCollisions(req, res, gsr);
    if(!done) {
      done = cdr.getIntraGroupCollisions(req, res, gsr);
    }
    if(!done) {
      getEnvironmentCollisions(req, res, distance_field_cache_entry_->distance_field_, gsr);
    }
  } catch(...) {
    logError("Could not cast CollisionRobot to CollisionRobotDistanceField");
    return;
  }

  //(const_cast<CollisionWorldDistanceField*>(this))->last_gsr_ = gsr;
}

void CollisionWorldDistanceField::checkCollision(const CollisionRequest &req, 
                                                 CollisionResult &res, 
                                                 const CollisionRobot &robot, 
                                                 const robot_state::RobotState &state, 
                                                 const AllowedCollisionMatrix &acm) const
{
  boost::shared_ptr<GroupStateRepresentation> gsr; 
  checkCollision(req, 
                 res,
                 robot,
                 state,
                 acm,
                 gsr);
}

void CollisionWorldDistanceField::checkCollision(const CollisionRequest &req,
                                                 CollisionResult &res,
                                                 const CollisionRobot &robot,
                                                 const robot_state::RobotState &state,
                                                 const AllowedCollisionMatrix &acm,
                                                 boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  try {
    const CollisionRobotDistanceField& cdr = dynamic_cast<const CollisionRobotDistanceField&>(robot);
    if(!gsr) {
      cdr.generateCollisionCheckingStructures(req.group_name,
                                              state,
                                              &acm,
                                              gsr,
                                              true);
    } else {
      cdr.updateGroupStateRepresentationState(state, gsr);
    }
    bool done = cdr.getSelfCollisions(req, res, gsr);
    if(!done) {
      done = cdr.getIntraGroupCollisions(req, res, gsr);
    }
    if(!done) {
      getEnvironmentCollisions(req, res, distance_field_cache_entry_->distance_field_, gsr);
    }
  } catch(...) {
    logError("Could not cast CollisionRobot to CollisionRobotDistanceField");
    return;
  }

  //(const_cast<CollisionWorldDistanceField*>(this))->last_gsr_ = gsr;
}

void CollisionWorldDistanceField::checkRobotCollision(const CollisionRequest &req, 
                                                      CollisionResult &res, 
                                                      const CollisionRobot &robot, 
                                                      const robot_state::RobotState &state) const
{
  boost::shared_ptr<GroupStateRepresentation> gsr;
  checkRobotCollision(req, res, robot, state, gsr);
}

void CollisionWorldDistanceField::checkRobotCollision(const CollisionRequest &req, 
                                                      CollisionResult &res, 
                                                      const CollisionRobot &robot, 
                                                      const robot_state::RobotState &state,
                                                      boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  boost::shared_ptr<const distance_field::DistanceField> env_distance_field = distance_field_cache_entry_->distance_field_;
  try {
    const CollisionRobotDistanceField& cdr = dynamic_cast<const CollisionRobotDistanceField&>(robot);
    boost::shared_ptr<const DistanceFieldCacheEntry> dfce;
    if(!gsr) {
      cdr.generateCollisionCheckingStructures(req.group_name,
                                              state,
                                              NULL,
                                              gsr,
                                              false);
    } else {
      cdr.updateGroupStateRepresentationState(state, gsr);
    }
    getEnvironmentCollisions(req, res, env_distance_field, gsr);
    //(const_cast<CollisionWorldDistanceField*>(this))->last_gsr_ = gsr;
    //checkRobotCollisionHelper(req, res, robot, state, &acm);
  } catch(...) {
    logError("Could not cast CollisionRobot to CollisionRobotDistanceField");
    return;
  }
}


void CollisionWorldDistanceField::checkRobotCollision(const CollisionRequest &req, 
                                                      CollisionResult &res, 
                                                      const CollisionRobot &robot, 
                                                      const robot_state::RobotState &state, 
                                                      const AllowedCollisionMatrix &acm) const
{
  boost::shared_ptr<GroupStateRepresentation> gsr;
  checkRobotCollision(req, res, robot, state, acm, gsr);
}

void CollisionWorldDistanceField::checkRobotCollision(const CollisionRequest &req, 
                                                      CollisionResult &res, 
                                                      const CollisionRobot &robot, 
                                                      const robot_state::RobotState &state, 
                                                      const AllowedCollisionMatrix &acm,
                                                      boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  boost::shared_ptr<const distance_field::DistanceField> env_distance_field = distance_field_cache_entry_->distance_field_;
  try {
    const CollisionRobotDistanceField& cdr = dynamic_cast<const CollisionRobotDistanceField&>(robot);
    boost::shared_ptr<const DistanceFieldCacheEntry> dfce;
    if(!gsr) {
      cdr.generateCollisionCheckingStructures(req.group_name,
                                              state,
                                              &acm,
                                              gsr,
                                              false);
    } else {
      cdr.updateGroupStateRepresentationState(state, gsr);
    }
    getEnvironmentCollisions(req, res, env_distance_field, gsr);
    //(const_cast<CollisionWorldDistanceField*>(this))->last_gsr_ = gsr;
    //checkRobotCollisionHelper(req, res, robot, state, &acm);
  } catch(...) {
    logError("Could not cast CollisionRobot to CollisionRobotDistanceField");
    return;
  }
}

void CollisionWorldDistanceField::getCollisionGradients(const CollisionRequest &req, 
                                                        CollisionResult &res, 
                                                        const CollisionRobot &robot, 
                                                        const robot_state::RobotState &state, 
                                                        const AllowedCollisionMatrix* acm,
                                                        boost::shared_ptr<GroupStateRepresentation>& gsr) const {
  boost::shared_ptr<const distance_field::DistanceField> env_distance_field = distance_field_cache_entry_->distance_field_;
  try {
    const CollisionRobotDistanceField& cdr = dynamic_cast<const CollisionRobotDistanceField&>(robot);
    if(!gsr) {
      cdr.generateCollisionCheckingStructures(req.group_name,
                                              state,
                                              acm,
                                              gsr,
                                              true);
    } else {
      cdr.updateGroupStateRepresentationState(state, gsr);
    }
    cdr.getSelfProximityGradients(gsr);
    cdr.getIntraGroupProximityGradients(gsr);
    getEnvironmentProximityGradients(env_distance_field, gsr);
  } catch(...) {
    logError("Could not cast CollisionRobot to CollisionRobotDistanceField");
    return;
  }
}

void CollisionWorldDistanceField::getAllCollisions(const CollisionRequest &req, 
                                                   CollisionResult &res, 
                                                   const CollisionRobot &robot, 
                                                   const robot_state::RobotState &state, 
                                                   const AllowedCollisionMatrix* acm,
                                                   boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  try {
    const CollisionRobotDistanceField& cdr = dynamic_cast<const CollisionRobotDistanceField&>(robot);
    if(!gsr) {
      cdr.generateCollisionCheckingStructures(req.group_name,
                                              state,
                                              acm,
                                              gsr,
                                              true);
    } else {
      cdr.updateGroupStateRepresentationState(state, gsr);
    }
    cdr.getSelfCollisions(req, res, gsr);
    cdr.getIntraGroupCollisions(req, res, gsr);
    boost::shared_ptr<const distance_field::DistanceField> env_distance_field = distance_field_cache_entry_->distance_field_;
    getEnvironmentCollisions(req, res, env_distance_field, gsr);
  } catch(...) {
    logError("Could not cast CollisionRobot to CollisionRobotDistanceField");    
    return;
  }  
}

bool CollisionWorldDistanceField::getEnvironmentCollisions(const CollisionRequest& req,
                                                           CollisionResult& res,
                                                           const boost::shared_ptr<const distance_field::DistanceField>& env_distance_field,
                                                           boost::shared_ptr<GroupStateRepresentation>& gsr) const
{
  for(unsigned int i = 0; i < gsr->dfce_->link_names_.size()+gsr->dfce_->attached_body_names_.size(); i++) {
    bool is_link = i < gsr->dfce_->link_names_.size();
    if(is_link && !gsr->dfce_->link_has_geometry_[i]) continue;
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
      bool coll = getCollisionSphereCollision(env_distance_field.get(),
                                              *collision_spheres_1,
                                              *sphere_centers_1,
                                              max_propogation_distance_,
                                              0.0,
                                              std::min(req.max_contacts_per_pair, req.max_contacts-res.contact_count),
                                              colls);
      if(coll) {
        res.collision = true;
        for(unsigned int j = 0; j < colls.size(); j++) {
          Contact con;
          if(is_link) {
            con.pos = gsr->link_body_decompositions_[i]->getSphereCenters()[colls[j]];
            con.body_type_1 = BodyTypes::ROBOT_LINK;
            con.body_name_1 = gsr->dfce_->link_names_[i];
          } else {
            con.pos = gsr->attached_body_decompositions_[i-gsr->dfce_->link_names_.size()]->getSphereCenters()[colls[j]];
            con.body_type_1 = BodyTypes::ROBOT_ATTACHED;
            con.body_name_1 = gsr->dfce_->attached_body_names_[i-gsr->dfce_->link_names_.size()];
          }
          con.body_type_2 = BodyTypes::WORLD_OBJECT;
          con.body_name_2 = "environment";
          res.contact_count++;
          res.contacts[std::pair<std::string,std::string>(con.body_name_1, con.body_name_2)].push_back(con);
          gsr->gradients_[i].types[colls[j]] = ENVIRONMENT;
          //ROS_DEBUG_STREAM("Link " << dfce->link_names_[i] << " sphere " << colls[j] << " in env collision");
        }
        gsr->gradients_[i].collision = true;
        if(res.contact_count >= req.max_contacts) {
          return true;
        }
      } 
    } else {
      bool coll = getCollisionSphereCollision(env_distance_field.get(),
                                              *collision_spheres_1,
                                              *sphere_centers_1,
                                              max_propogation_distance_,
                                              false);
      if(coll) {
        res.collision = true;
        return true;
      } 
    }
  }
  return (res.contact_count >= req.max_contacts);
} 

bool CollisionWorldDistanceField::getEnvironmentProximityGradients(const boost::shared_ptr<const distance_field::DistanceField>& env_distance_field,
                                                                   boost::shared_ptr<GroupStateRepresentation>& gsr) const 
{
  bool in_collision = false;
  for(unsigned int i = 0; i < gsr->dfce_->link_names_.size(); i++) {
    bool is_link = i < gsr->dfce_->link_names_.size();
    if(is_link && !gsr->dfce_->link_has_geometry_[i]) continue;
    const std::vector<CollisionSphere>* collision_spheres_1;
    const EigenSTL::vector_Vector3d* sphere_centers_1;
    if(is_link) {
      collision_spheres_1 = &(gsr->link_body_decompositions_[i]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->link_body_decompositions_[i]->getSphereCenters());
    } else {
      collision_spheres_1 = &(gsr->attached_body_decompositions_[i-gsr->dfce_->link_names_.size()]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->attached_body_decompositions_[i-gsr->dfce_->link_names_.size()]->getSphereCenters());
    }
    bool coll = getCollisionSphereGradients(env_distance_field.get(),
                                            *collision_spheres_1,
                                            *sphere_centers_1,
                                            gsr->gradients_[i],
                                            ENVIRONMENT,
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

void CollisionWorldDistanceField::setWorld(const WorldPtr& world)
{
  if (world == getWorld())
    return;

  // turn off notifications about old world
  getWorld()->removeObserver(observer_handle_);

  // clear out objects from old world
  distance_field_cache_entry_->distance_field_->reset();

  CollisionWorld::setWorld(world);

  // request notifications about changes to new world
  observer_handle_ = getWorld()->addObserver(boost::bind(&CollisionWorldDistanceField::notifyObjectChange, this, _1, _2));

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void CollisionWorldDistanceField::notifyObjectChange(CollisionWorldDistanceField *self, const ObjectConstPtr& obj, World::Action action)
{
  ros::WallTime n = ros::WallTime::now();

  EigenSTL::vector_Vector3d add_points;
  EigenSTL::vector_Vector3d subtract_points;
  self->updateDistanceObject(obj->id_, self->distance_field_cache_entry_, add_points, subtract_points);

  if (action == World::DESTROY)
  {
    self->distance_field_cache_entry_->distance_field_->removePointsFromField(subtract_points);
  }
  else if (action & (World::MOVE_SHAPE | World::REMOVE_SHAPE))
  {
    self->distance_field_cache_entry_->distance_field_->removePointsFromField(subtract_points);
    self->distance_field_cache_entry_->distance_field_->addPointsToField(add_points);
  }
  else
  {
    self->distance_field_cache_entry_->distance_field_->addPointsToField(add_points);
  }

  logDebug("Modifying object %s took %lf s",
    obj->id_.c_str(),
    (ros::WallTime::now()-n).toSec());
}

void CollisionWorldDistanceField::updateDistanceObject(const std::string& id,
                                                       boost::shared_ptr<CollisionWorldDistanceField::DistanceFieldCacheEntry>& dfce,
                                                       EigenSTL::vector_Vector3d& add_points,
                                                       EigenSTL::vector_Vector3d& subtract_points)
{
  std::map<std::string, std::vector<PosedBodyPointDecompositionPtr> >::iterator cur_it = dfce->posed_body_point_decompositions_.find(id);
  if(cur_it != dfce->posed_body_point_decompositions_.end()) {
    for(unsigned int i = 0; i < cur_it->second.size(); i++) {
      subtract_points.insert(subtract_points.end(),
                             cur_it->second[i]->getCollisionPoints().begin(),
                             cur_it->second[i]->getCollisionPoints().end());
    }
  }
  World::ObjectConstPtr object = getWorld()->getObject(id);
  if(object) {
    std::vector<PosedBodyPointDecompositionPtr> shape_points;
    for(unsigned int i = 0; i < object->shapes_.size(); i++) {
      
      BodyDecompositionConstPtr bd = getBodyDecompositionCacheEntry(object->shapes_[i],
                                                                    resolution_);
      
      shape_points.push_back(boost::make_shared<PosedBodyPointDecomposition>(bd, object->shape_poses_[i]));
      add_points.insert(add_points.end(),
                        shape_points.back()->getCollisionPoints().begin(),
                        shape_points.back()->getCollisionPoints().end());
    }
    dfce->posed_body_point_decompositions_[id] = shape_points;
  } else {
    dfce->posed_body_point_decompositions_.erase(id);
  }
}

boost::shared_ptr<CollisionWorldDistanceField::DistanceFieldCacheEntry>
CollisionWorldDistanceField::generateDistanceFieldCacheEntry()
{
  boost::shared_ptr<DistanceFieldCacheEntry> dfce(new DistanceFieldCacheEntry());
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
  EigenSTL::vector_Vector3d add_points;
  EigenSTL::vector_Vector3d subtract_points;
  for(World::const_iterator it=getWorld()->begin(); it!=getWorld()->end(); ++it)
  {
    updateDistanceObject(it->first, dfce, add_points, subtract_points);
  }
  dfce->distance_field_->addPointsToField(add_points);
  return dfce;
}

}

#include <moveit/collision_distance_field/collision_detector_allocator_distance_field.h>
const std::string collision_detection::CollisionDetectorAllocatorDistanceField::NAME_("DISTANCE_FIELD");
