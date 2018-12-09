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

#include <ros/console.h>
#include <moveit/collision_distance_field/collision_world_distance_field.h>
#include <moveit/collision_distance_field/collision_common_distance_field.h>
#include <moveit/distance_field/propagation_distance_field.h>
#include <boost/bind.hpp>
#include <memory>

namespace collision_detection
{
CollisionWorldDistanceField::~CollisionWorldDistanceField()
{
  getWorld()->removeObserver(observer_handle_);
}

CollisionWorldDistanceField::CollisionWorldDistanceField(Eigen::Vector3d size, Eigen::Vector3d origin,
                                                         bool use_signed_distance_field, double resolution,
                                                         double collision_tolerance, double max_propogation_distance)
  : CollisionWorld()
  , size_(size)
  , origin_(origin)
  , use_signed_distance_field_(use_signed_distance_field)
  , resolution_(resolution)
  , collision_tolerance_(collision_tolerance)
  , max_propogation_distance_(max_propogation_distance)
{
  distance_field_cache_entry_ = generateDistanceFieldCacheEntry();

  // request notifications about changes to world
  observer_handle_ =
      getWorld()->addObserver(boost::bind(&CollisionWorldDistanceField::notifyObjectChange, this, _1, _2));
}

CollisionWorldDistanceField::CollisionWorldDistanceField(const WorldPtr& world, Eigen::Vector3d size,
                                                         Eigen::Vector3d origin, bool use_signed_distance_field,
                                                         double resolution, double collision_tolerance,
                                                         double max_propogation_distance)
  : CollisionWorld(world)
  , size_(size)
  , origin_(origin)
  , use_signed_distance_field_(use_signed_distance_field)
  , resolution_(resolution)
  , collision_tolerance_(collision_tolerance)
  , max_propogation_distance_(max_propogation_distance)
{
  distance_field_cache_entry_ = generateDistanceFieldCacheEntry();

  // request notifications about changes to world
  observer_handle_ =
      getWorld()->addObserver(boost::bind(&CollisionWorldDistanceField::notifyObjectChange, this, _1, _2));
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

CollisionWorldDistanceField::CollisionWorldDistanceField(const CollisionWorldDistanceField& other,
                                                         const WorldPtr& world)
  : CollisionWorld(other, world)
{
  size_ = other.size_;
  origin_ = other.origin_;
  use_signed_distance_field_ = other.use_signed_distance_field_;
  resolution_ = other.resolution_;
  collision_tolerance_ = other.collision_tolerance_;
  max_propogation_distance_ = other.max_propogation_distance_;
  distance_field_cache_entry_ = generateDistanceFieldCacheEntry();

  // request notifications about changes to world
  observer_handle_ =
      getWorld()->addObserver(boost::bind(&CollisionWorldDistanceField::notifyObjectChange, this, _1, _2));
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void CollisionWorldDistanceField::checkCollision(const CollisionRequest& req, CollisionResult& res,
                                                 const CollisionRobot& robot,
                                                 const robot_state::RobotState& state) const
{
  GroupStateRepresentationPtr gsr;
  checkCollision(req, res, robot, state, gsr);
}

void CollisionWorldDistanceField::checkCollision(const CollisionRequest& req, CollisionResult& res,
                                                 const CollisionRobot& robot, const robot_state::RobotState& state,
                                                 GroupStateRepresentationPtr& gsr) const
{
  try
  {
    const CollisionRobotDistanceField& cdr = dynamic_cast<const CollisionRobotDistanceField&>(robot);
    if (!gsr)
    {
      cdr.generateCollisionCheckingStructures(req.group_name, state, NULL, gsr, true);
    }
    else
    {
      cdr.updateGroupStateRepresentationState(state, gsr);
    }
    bool done = cdr.getSelfCollisions(req, res, gsr);
    if (!done)
    {
      done = cdr.getIntraGroupCollisions(req, res, gsr);
    }
    if (!done)
    {
      getEnvironmentCollisions(req, res, distance_field_cache_entry_->distance_field_, gsr);
    }
  }
  catch (const std::bad_cast& e)
  {
    ROS_ERROR_STREAM("Could not cast CollisionRobot to CollisionRobotDistanceField, " << e.what());
    return;
  }

  (const_cast<CollisionWorldDistanceField*>(this))->last_gsr_ = gsr;
}

void CollisionWorldDistanceField::checkCollision(const CollisionRequest& req, CollisionResult& res,
                                                 const CollisionRobot& robot, const robot_state::RobotState& state,
                                                 const AllowedCollisionMatrix& acm) const
{
  GroupStateRepresentationPtr gsr;
  checkCollision(req, res, robot, state, acm, gsr);
}

void CollisionWorldDistanceField::checkCollision(const CollisionRequest& req, CollisionResult& res,
                                                 const CollisionRobot& robot, const robot_state::RobotState& state,
                                                 const AllowedCollisionMatrix& acm,
                                                 GroupStateRepresentationPtr& gsr) const
{
  try
  {
    const CollisionRobotDistanceField& cdr = dynamic_cast<const CollisionRobotDistanceField&>(robot);
    if (!gsr)
    {
      cdr.generateCollisionCheckingStructures(req.group_name, state, &acm, gsr, true);
    }
    else
    {
      cdr.updateGroupStateRepresentationState(state, gsr);
    }
    bool done = cdr.getSelfCollisions(req, res, gsr);
    if (!done)
    {
      done = cdr.getIntraGroupCollisions(req, res, gsr);
    }
    if (!done)
    {
      getEnvironmentCollisions(req, res, distance_field_cache_entry_->distance_field_, gsr);
    }
  }
  catch (const std::bad_cast& e)
  {
    ROS_ERROR_STREAM("Could not cast CollisionRobot to CollisionRobotDistanceField, " << e.what());
    return;
  }

  (const_cast<CollisionWorldDistanceField*>(this))->last_gsr_ = gsr;
}

void CollisionWorldDistanceField::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                      const CollisionRobot& robot,
                                                      const robot_state::RobotState& state) const
{
  GroupStateRepresentationPtr gsr;
  checkRobotCollision(req, res, robot, state, gsr);
}

void CollisionWorldDistanceField::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                      const CollisionRobot& robot, const robot_state::RobotState& state,
                                                      GroupStateRepresentationPtr& gsr) const
{
  distance_field::DistanceFieldConstPtr env_distance_field = distance_field_cache_entry_->distance_field_;
  try
  {
    const CollisionRobotDistanceField& cdr = dynamic_cast<const CollisionRobotDistanceField&>(robot);
    DistanceFieldCacheEntryConstPtr dfce;
    if (!gsr)
    {
      cdr.generateCollisionCheckingStructures(req.group_name, state, NULL, gsr, false);
    }
    else
    {
      cdr.updateGroupStateRepresentationState(state, gsr);
    }
    getEnvironmentCollisions(req, res, env_distance_field, gsr);
    (const_cast<CollisionWorldDistanceField*>(this))->last_gsr_ = gsr;

    // checkRobotCollisionHelper(req, res, robot, state, &acm);
  }
  catch (const std::bad_cast& e)
  {
    ROS_ERROR_STREAM("Could not cast CollisionRobot to CollisionRobotDistanceField, " << e.what());
    return;
  }
}

void CollisionWorldDistanceField::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                      const CollisionRobot& robot, const robot_state::RobotState& state,
                                                      const AllowedCollisionMatrix& acm) const
{
  GroupStateRepresentationPtr gsr;
  checkRobotCollision(req, res, robot, state, acm, gsr);
}

void CollisionWorldDistanceField::checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
                                                      const CollisionRobot& robot, const robot_state::RobotState& state,
                                                      const AllowedCollisionMatrix& acm,
                                                      GroupStateRepresentationPtr& gsr) const
{
  distance_field::DistanceFieldConstPtr env_distance_field = distance_field_cache_entry_->distance_field_;
  try
  {
    const CollisionRobotDistanceField& cdr = dynamic_cast<const CollisionRobotDistanceField&>(robot);
    DistanceFieldCacheEntryPtr dfce;
    if (!gsr)
    {
      cdr.generateCollisionCheckingStructures(req.group_name, state, &acm, gsr, true);
    }
    else
    {
      cdr.updateGroupStateRepresentationState(state, gsr);
    }
    getEnvironmentCollisions(req, res, env_distance_field, gsr);
    (const_cast<CollisionWorldDistanceField*>(this))->last_gsr_ = gsr;

    // checkRobotCollisionHelper(req, res, robot, state, &acm);
  }
  catch (const std::bad_cast& e)
  {
    ROS_ERROR_STREAM("Could not cast CollisionRobot to CollisionRobotDistanceField, " << e.what());
    return;
  }
}

void CollisionWorldDistanceField::getCollisionGradients(const CollisionRequest& req, CollisionResult& res,
                                                        const CollisionRobot& robot,
                                                        const robot_state::RobotState& state,
                                                        const AllowedCollisionMatrix* acm,
                                                        GroupStateRepresentationPtr& gsr) const
{
  distance_field::DistanceFieldConstPtr env_distance_field = distance_field_cache_entry_->distance_field_;
  try
  {
    const CollisionRobotDistanceField& cdr = dynamic_cast<const CollisionRobotDistanceField&>(robot);
    if (!gsr)
    {
      cdr.generateCollisionCheckingStructures(req.group_name, state, acm, gsr, true);
    }
    else
    {
      cdr.updateGroupStateRepresentationState(state, gsr);
    }
    cdr.getSelfProximityGradients(gsr);
    cdr.getIntraGroupProximityGradients(gsr);
    getEnvironmentProximityGradients(env_distance_field, gsr);
  }
  catch (const std::bad_cast& e)
  {
    ROS_ERROR_STREAM("Could not cast CollisionRobot to CollisionRobotDistanceField, " << e.what());
    return;
  }

  (const_cast<CollisionWorldDistanceField*>(this))->last_gsr_ = gsr;
}

void CollisionWorldDistanceField::getAllCollisions(const CollisionRequest& req, CollisionResult& res,
                                                   const CollisionRobot& robot, const robot_state::RobotState& state,
                                                   const AllowedCollisionMatrix* acm,
                                                   GroupStateRepresentationPtr& gsr) const
{
  try
  {
    const CollisionRobotDistanceField& cdr = dynamic_cast<const CollisionRobotDistanceField&>(robot);
    if (!gsr)
    {
      cdr.generateCollisionCheckingStructures(req.group_name, state, acm, gsr, true);
    }
    else
    {
      cdr.updateGroupStateRepresentationState(state, gsr);
    }
    cdr.getSelfCollisions(req, res, gsr);
    cdr.getIntraGroupCollisions(req, res, gsr);
    distance_field::DistanceFieldConstPtr env_distance_field = distance_field_cache_entry_->distance_field_;
    getEnvironmentCollisions(req, res, env_distance_field, gsr);
  }
  catch (const std::bad_cast& e)
  {
    ROS_ERROR_STREAM("Could not cast CollisionRobot to CollisionRobotDistanceField, " << e.what());
    return;
  }

  (const_cast<CollisionWorldDistanceField*>(this))->last_gsr_ = gsr;
}

bool CollisionWorldDistanceField::getEnvironmentCollisions(
    const CollisionRequest& req, CollisionResult& res, const distance_field::DistanceFieldConstPtr& env_distance_field,
    GroupStateRepresentationPtr& gsr) const
{
  for (unsigned int i = 0; i < gsr->dfce_->link_names_.size() + gsr->dfce_->attached_body_names_.size(); i++)
  {
    bool is_link = i < gsr->dfce_->link_names_.size();
    std::string link_name = i < gsr->dfce_->link_names_.size() ? gsr->dfce_->link_names_[i] : "attached";
    if (is_link && !gsr->dfce_->link_has_geometry_[i])
    {
      continue;
    }

    const std::vector<CollisionSphere>* collision_spheres_1;
    const EigenSTL::vector_Vector3d* sphere_centers_1;

    if (is_link)
    {
      collision_spheres_1 = &(gsr->link_body_decompositions_[i]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->link_body_decompositions_[i]->getSphereCenters());
    }
    else
    {
      collision_spheres_1 =
          &(gsr->attached_body_decompositions_[i - gsr->dfce_->link_names_.size()]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->attached_body_decompositions_[i - gsr->dfce_->link_names_.size()]->getSphereCenters());
    }

    if (req.contacts)
    {
      std::vector<unsigned int> colls;
      bool coll = getCollisionSphereCollision(
          env_distance_field.get(), *collision_spheres_1, *sphere_centers_1, max_propogation_distance_,
          collision_tolerance_, std::min(req.max_contacts_per_pair, req.max_contacts - res.contact_count), colls);
      if (coll)
      {
        res.collision = true;
        for (unsigned int j = 0; j < colls.size(); j++)
        {
          Contact con;
          if (is_link)
          {
            con.pos = gsr->link_body_decompositions_[i]->getSphereCenters()[colls[j]];
            con.body_type_1 = BodyTypes::ROBOT_LINK;
            con.body_name_1 = gsr->dfce_->link_names_[i];
          }
          else
          {
            con.pos =
                gsr->attached_body_decompositions_[i - gsr->dfce_->link_names_.size()]->getSphereCenters()[colls[j]];
            con.body_type_1 = BodyTypes::ROBOT_ATTACHED;
            con.body_name_1 = gsr->dfce_->attached_body_names_[i - gsr->dfce_->link_names_.size()];
          }

          con.body_type_2 = BodyTypes::WORLD_OBJECT;
          con.body_name_2 = "environment";
          res.contact_count++;
          res.contacts[std::pair<std::string, std::string>(con.body_name_1, con.body_name_2)].push_back(con);
          gsr->gradients_[i].types[colls[j]] = ENVIRONMENT;
          // ROS_DEBUG_STREAM("Link " << dfce->link_names_[i] << " sphere " <<
          // colls[j] << " in env collision");
        }

        gsr->gradients_[i].collision = true;
        if (res.contact_count >= req.max_contacts)
        {
          return true;
        }
      }
    }
    else
    {
      bool coll = getCollisionSphereCollision(env_distance_field.get(), *collision_spheres_1, *sphere_centers_1,
                                              max_propogation_distance_, collision_tolerance_);
      if (coll)
      {
        res.collision = true;
        return true;
      }
    }
  }
  return (res.contact_count >= req.max_contacts);
}

bool CollisionWorldDistanceField::getEnvironmentProximityGradients(
    const distance_field::DistanceFieldConstPtr& env_distance_field, GroupStateRepresentationPtr& gsr) const
{
  bool in_collision = false;
  for (unsigned int i = 0; i < gsr->dfce_->link_names_.size(); i++)
  {
    bool is_link = i < gsr->dfce_->link_names_.size();

    if (is_link && !gsr->dfce_->link_has_geometry_[i])
    {
      continue;
    }

    const std::vector<CollisionSphere>* collision_spheres_1;
    const EigenSTL::vector_Vector3d* sphere_centers_1;
    if (is_link)
    {
      collision_spheres_1 = &(gsr->link_body_decompositions_[i]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->link_body_decompositions_[i]->getSphereCenters());
    }
    else
    {
      collision_spheres_1 =
          &(gsr->attached_body_decompositions_[i - gsr->dfce_->link_names_.size()]->getCollisionSpheres());
      sphere_centers_1 = &(gsr->attached_body_decompositions_[i - gsr->dfce_->link_names_.size()]->getSphereCenters());
    }

    bool coll = getCollisionSphereGradients(env_distance_field.get(), *collision_spheres_1, *sphere_centers_1,
                                            gsr->gradients_[i], ENVIRONMENT, collision_tolerance_, false,
                                            max_propogation_distance_, false);
    if (coll)
    {
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
  observer_handle_ =
      getWorld()->addObserver(boost::bind(&CollisionWorldDistanceField::notifyObjectChange, this, _1, _2));

  // get notifications any objects already in the new world
  getWorld()->notifyObserverAllObjects(observer_handle_, World::CREATE);
}

void CollisionWorldDistanceField::notifyObjectChange(CollisionWorldDistanceField* self, const ObjectConstPtr& obj,
                                                     World::Action action)
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

  ROS_DEBUG_NAMED("collision_distance_field", "Modifying object %s took %lf s", obj->id_.c_str(),
                  (ros::WallTime::now() - n).toSec());
}

void CollisionWorldDistanceField::updateDistanceObject(const std::string& id, DistanceFieldCacheEntryPtr& dfce,
                                                       EigenSTL::vector_Vector3d& add_points,
                                                       EigenSTL::vector_Vector3d& subtract_points)
{
  std::map<std::string, std::vector<PosedBodyPointDecompositionPtr>>::iterator cur_it =
      dfce->posed_body_point_decompositions_.find(id);
  if (cur_it != dfce->posed_body_point_decompositions_.end())
  {
    for (unsigned int i = 0; i < cur_it->second.size(); i++)
    {
      subtract_points.insert(subtract_points.end(), cur_it->second[i]->getCollisionPoints().begin(),
                             cur_it->second[i]->getCollisionPoints().end());
    }
  }

  World::ObjectConstPtr object = getWorld()->getObject(id);
  if (object)
  {
    ROS_DEBUG_STREAM("Updating/Adding Object '" << object->id_ << "' with " << object->shapes_.size()
                                                << " shapes  to CollisionWorldDistanceField");
    std::vector<PosedBodyPointDecompositionPtr> shape_points;
    for (unsigned int i = 0; i < object->shapes_.size(); i++)
    {
      shapes::ShapeConstPtr shape = object->shapes_[i];
      if (shape->type == shapes::OCTREE)
      {
        const shapes::OcTree* octree_shape = static_cast<const shapes::OcTree*>(shape.get());
        std::shared_ptr<const octomap::OcTree> octree = octree_shape->octree;

        shape_points.push_back(std::make_shared<PosedBodyPointDecomposition>(octree));
      }
      else
      {
        BodyDecompositionConstPtr bd = getBodyDecompositionCacheEntry(shape, resolution_);

        shape_points.push_back(std::make_shared<PosedBodyPointDecomposition>(bd, object->shape_poses_[i]));
      }

      add_points.insert(add_points.end(), shape_points.back()->getCollisionPoints().begin(),
                        shape_points.back()->getCollisionPoints().end());
    }

    dfce->posed_body_point_decompositions_[id] = shape_points;
  }
  else
  {
    ROS_DEBUG_STREAM("Removing Object '" << id << "' from CollisionWorldDistanceField");
    dfce->posed_body_point_decompositions_.erase(id);
  }
}

CollisionWorldDistanceField::DistanceFieldCacheEntryPtr CollisionWorldDistanceField::generateDistanceFieldCacheEntry()
{
  DistanceFieldCacheEntryPtr dfce(new DistanceFieldCacheEntry());
  dfce->distance_field_.reset(new distance_field::PropagationDistanceField(
      size_.x(), size_.y(), size_.z(), resolution_, origin_.x() - 0.5 * size_.x(), origin_.y() - 0.5 * size_.y(),
      origin_.z() - 0.5 * size_.z(), max_propogation_distance_, use_signed_distance_field_));

  EigenSTL::vector_Vector3d add_points;
  EigenSTL::vector_Vector3d subtract_points;
  for (World::const_iterator it = getWorld()->begin(); it != getWorld()->end(); ++it)
  {
    updateDistanceObject(it->first, dfce, add_points, subtract_points);
  }
  dfce->distance_field_->addPointsToField(add_points);
  return dfce;
}
}

#include <moveit/collision_distance_field/collision_detector_allocator_distance_field.h>
const std::string collision_detection::CollisionDetectorAllocatorDistanceField::NAME_("DISTANCE_FIELD");
