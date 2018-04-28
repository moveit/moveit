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

#ifndef MOVEIT_COLLISION_DISTANCE_FIELD_COLLISION_WORLD_DISTANCE_FIELD_
#define MOVEIT_COLLISION_DISTANCE_FIELD_COLLISION_WORLD_DISTANCE_FIELD_

#include <moveit/macros/class_forward.h>
#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_distance_field/collision_distance_field_types.h>
#include <moveit/collision_distance_field/collision_robot_distance_field.h>

namespace collision_detection
{
<<<<<<< HEAD
MOVEIT_CLASS_FORWARD(CollisionWorldDistanceField)

=======
>>>>>>> upstream/indigo-devel
class CollisionWorldDistanceField : public CollisionWorld
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
<<<<<<< HEAD

  MOVEIT_CLASS_FORWARD(DistanceFieldCacheEntry)
  struct DistanceFieldCacheEntry
  {
    std::map<std::string, std::vector<PosedBodyPointDecompositionPtr>> posed_body_point_decompositions_;
    distance_field::DistanceFieldPtr distance_field_;
=======
  struct DistanceFieldCacheEntry
  {
    std::map<std::string, std::vector<PosedBodyPointDecompositionPtr>> posed_body_point_decompositions_;
    boost::shared_ptr<distance_field::DistanceField> distance_field_;
>>>>>>> upstream/indigo-devel
  };

  CollisionWorldDistanceField(Eigen::Vector3d size = Eigen::Vector3d(DEFAULT_SIZE_X, DEFAULT_SIZE_Y, DEFAULT_SIZE_Z),
                              Eigen::Vector3d origin = Eigen::Vector3d(0, 0, 0),
                              bool use_signed_distance_field = DEFAULT_USE_SIGNED_DISTANCE_FIELD,
                              double resolution = DEFAULT_RESOLUTION,
                              double collision_tolerance = DEFAULT_COLLISION_TOLERANCE,
                              double max_propogation_distance = DEFAULT_MAX_PROPOGATION_DISTANCE);

  explicit CollisionWorldDistanceField(
      const WorldPtr& world, Eigen::Vector3d size = Eigen::Vector3d(DEFAULT_SIZE_X, DEFAULT_SIZE_Y, DEFAULT_SIZE_Z),
      Eigen::Vector3d origin = Eigen::Vector3d(0, 0, 0),
      bool use_signed_distance_field = DEFAULT_USE_SIGNED_DISTANCE_FIELD, double resolution = DEFAULT_RESOLUTION,
      double collision_tolerance = DEFAULT_COLLISION_TOLERANCE,
      double max_propogation_distance = DEFAULT_MAX_PROPOGATION_DISTANCE);

  CollisionWorldDistanceField(const CollisionWorldDistanceField& other, const WorldPtr& world);

  virtual ~CollisionWorldDistanceField();

  virtual void checkCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                              const robot_state::RobotState& state) const;

  virtual void checkCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
<<<<<<< HEAD
                              const robot_state::RobotState& state, GroupStateRepresentationPtr& gsr) const;
=======
                              const robot_state::RobotState& state,
                              boost::shared_ptr<GroupStateRepresentation>& gsr) const;
>>>>>>> upstream/indigo-devel

  virtual void checkCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                              const robot_state::RobotState& state, const AllowedCollisionMatrix& acm) const;

  virtual void checkCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                              const robot_state::RobotState& state, const AllowedCollisionMatrix& acm,
<<<<<<< HEAD
                              GroupStateRepresentationPtr& gsr) const;
=======
                              boost::shared_ptr<GroupStateRepresentation>& gsr) const;
>>>>>>> upstream/indigo-devel

  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                   const robot_state::RobotState& state) const;

  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
<<<<<<< HEAD
                                   const robot_state::RobotState& state, GroupStateRepresentationPtr& gsr) const;
=======
                                   const robot_state::RobotState& state,
                                   boost::shared_ptr<GroupStateRepresentation>& gsr) const;
>>>>>>> upstream/indigo-devel

  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                   const robot_state::RobotState& state, const AllowedCollisionMatrix& acm) const;

  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                   const robot_state::RobotState& state, const AllowedCollisionMatrix& acm,
<<<<<<< HEAD
                                   GroupStateRepresentationPtr& gsr) const;
=======
                                   boost::shared_ptr<GroupStateRepresentation>& gsr) const;
>>>>>>> upstream/indigo-devel

  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                   const robot_state::RobotState& state1, const robot_state::RobotState& state2) const
  {
  }
  virtual void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                                   const robot_state::RobotState& state1, const robot_state::RobotState& state2,
                                   const AllowedCollisionMatrix& acm) const
  {
  }
  virtual void checkWorldCollision(const CollisionRequest& req, CollisionResult& res,
                                   const CollisionWorld& other_world) const
  {
  }
  virtual void checkWorldCollision(const CollisionRequest& req, CollisionResult& res, const CollisionWorld& other_world,
                                   const AllowedCollisionMatrix& acm) const
  {
  }

<<<<<<< HEAD
  virtual double distanceRobot(const CollisionRobot& robot, const robot_state::RobotState& state,
                               bool verbose = false) const
=======
  virtual double distanceRobot(const CollisionRobot& robot, const robot_state::RobotState& state) const
>>>>>>> upstream/indigo-devel
  {
    return 0.0;
  }
  virtual double distanceRobot(const CollisionRobot& robot, const robot_state::RobotState& state,
<<<<<<< HEAD
                               const AllowedCollisionMatrix& acm, bool verbose = false) const
  {
    return 0.0;
  }
  virtual double distanceWorld(const CollisionWorld& world, bool verbose = false) const
  {
    return 0.0;
  }
  virtual double distanceWorld(const CollisionWorld& world, const AllowedCollisionMatrix& acm,
                               bool verbose = false) const
=======
                               const AllowedCollisionMatrix& acm) const
  {
    return 0.0;
  }
  virtual double distanceWorld(const CollisionWorld& world) const
  {
    return 0.0;
  }
  virtual double distanceWorld(const CollisionWorld& world, const AllowedCollisionMatrix& acm) const
>>>>>>> upstream/indigo-devel
  {
    return 0.0;
  }

  virtual void setWorld(const WorldPtr& world);

  void generateEnvironmentDistanceField(bool redo = true);

<<<<<<< HEAD
  distance_field::DistanceFieldConstPtr getDistanceField() const
=======
  boost::shared_ptr<const distance_field::DistanceField> getDistanceField() const
>>>>>>> upstream/indigo-devel
  {
    return distance_field_cache_entry_->distance_field_;
  }

<<<<<<< HEAD
  collision_detection::GroupStateRepresentationConstPtr getLastGroupStateRepresentation() const
=======
  boost::shared_ptr<const collision_detection::GroupStateRepresentation> getLastGroupStateRepresentation() const
>>>>>>> upstream/indigo-devel
  {
    return last_gsr_;
  }

  void getCollisionGradients(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                             const robot_state::RobotState& state, const AllowedCollisionMatrix* acm,
<<<<<<< HEAD
                             GroupStateRepresentationPtr& gsr) const;

  void getAllCollisions(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                        const robot_state::RobotState& state, const AllowedCollisionMatrix* acm,
                        GroupStateRepresentationPtr& gsr) const;

protected:
  DistanceFieldCacheEntryPtr generateDistanceFieldCacheEntry();

  void updateDistanceObject(const std::string& id, CollisionWorldDistanceField::DistanceFieldCacheEntryPtr& dfce,
                            EigenSTL::vector_Vector3d& add_points, EigenSTL::vector_Vector3d& subtract_points);

  bool getEnvironmentCollisions(const CollisionRequest& req, CollisionResult& res,
                                const distance_field::DistanceFieldConstPtr& env_distance_field,
                                GroupStateRepresentationPtr& gsr) const;

  bool getEnvironmentProximityGradients(const distance_field::DistanceFieldConstPtr& env_distance_field,
                                        GroupStateRepresentationPtr& gsr) const;
=======
                             boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  void getAllCollisions(const CollisionRequest& req, CollisionResult& res, const CollisionRobot& robot,
                        const robot_state::RobotState& state, const AllowedCollisionMatrix* acm,
                        boost::shared_ptr<GroupStateRepresentation>& gsr) const;

protected:
  boost::shared_ptr<DistanceFieldCacheEntry> generateDistanceFieldCacheEntry();

  void updateDistanceObject(const std::string& id,
                            boost::shared_ptr<CollisionWorldDistanceField::DistanceFieldCacheEntry>& dfce,
                            EigenSTL::vector_Vector3d& add_points, EigenSTL::vector_Vector3d& subtract_points);

  bool getEnvironmentCollisions(const CollisionRequest& req, CollisionResult& res,
                                const boost::shared_ptr<const distance_field::DistanceField>& env_distance_field,
                                boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  bool
  getEnvironmentProximityGradients(const boost::shared_ptr<const distance_field::DistanceField>& env_distance_field,
                                   boost::shared_ptr<GroupStateRepresentation>& gsr) const;
>>>>>>> upstream/indigo-devel

  static void notifyObjectChange(CollisionWorldDistanceField* self, const ObjectConstPtr& obj, World::Action action);

  Eigen::Vector3d size_;
  Eigen::Vector3d origin_;
  bool use_signed_distance_field_;
  double resolution_;
  double collision_tolerance_;
  double max_propogation_distance_;

  mutable boost::mutex update_cache_lock_;
<<<<<<< HEAD
  DistanceFieldCacheEntryPtr distance_field_cache_entry_;
  GroupStateRepresentationPtr last_gsr_;
=======
  boost::shared_ptr<DistanceFieldCacheEntry> distance_field_cache_entry_;
  boost::shared_ptr<collision_detection::GroupStateRepresentation> last_gsr_;
>>>>>>> upstream/indigo-devel
  World::ObserverHandle observer_handle_;
};
}

#endif
