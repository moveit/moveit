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

#ifndef MOVEIT_COLLISION_DISTANCE_FIELD_COLLISION_ROBOT_DISTANCE_FIELD_
#define MOVEIT_COLLISION_DISTANCE_FIELD_COLLISION_ROBOT_DISTANCE_FIELD_

#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_distance_field/collision_distance_field_types.h>
#include <moveit/collision_distance_field/collision_common_distance_field.h>

#include <boost/thread/mutex.hpp>

namespace collision_detection
{

static const double DEFAULT_SIZE_X = 3.0;
static const double DEFAULT_SIZE_Y = 3.0;
static const double DEFAULT_SIZE_Z = 4.0;
static const bool DEFAULT_USE_SIGNED_DISTANCE_FIELD = false;
static const double DEFAULT_RESOLUTION = .02;
static const double DEFAULT_COLLISION_TOLERANCE = 0.0;
static const double DEFAULT_MAX_PROPOGATION_DISTANCE = .25;

class CollisionRobotDistanceField : public CollisionRobot
{
  
  friend class CollisionWorldDistanceField;

public:

  CollisionRobotDistanceField(const robot_model::RobotModelConstPtr& kmodel);

  CollisionRobotDistanceField(const robot_model::RobotModelConstPtr& kmodel, 
                              const std::map<std::string, std::vector<CollisionSphere> >& link_body_decompositions,
                              double size_x = DEFAULT_SIZE_X, 
                              double size_y = DEFAULT_SIZE_Y,
                              double size_z = DEFAULT_SIZE_Z,
                              bool use_signed_distance_field = DEFAULT_USE_SIGNED_DISTANCE_FIELD,
                              double resolution = DEFAULT_RESOLUTION,
                              double collision_tolerance = DEFAULT_COLLISION_TOLERANCE,
                              double max_propogation_distance = DEFAULT_MAX_PROPOGATION_DISTANCE,
                              double padding = 0.0, 
                              double scale = 1.0);

  CollisionRobotDistanceField(const CollisionRobotDistanceField& other); 

  void initialize(const std::map<std::string, std::vector<CollisionSphere> >& link_body_decompositions,
                  double size_x, 
                  double size_y,
                  double size_z,
                  bool use_signed_distance_field,
                  double resolution,
                  double collision_tolerance,
                  double max_propogation_distance);

  virtual void checkSelfCollision(const collision_detection::CollisionRequest &req, 
                                  collision_detection::CollisionResult &res, 
                                  const robot_state::RobotState &state) const;

  void checkSelfCollision(const collision_detection::CollisionRequest &req, 
                          collision_detection::CollisionResult &res, 
                          const robot_state::RobotState &state,
                          boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  virtual void checkSelfCollision(const collision_detection::CollisionRequest &req, 
                                  collision_detection::CollisionResult &res, 
                                  const robot_state::RobotState &state,
                                  const collision_detection::AllowedCollisionMatrix &acm) const;

  void checkSelfCollision(const collision_detection::CollisionRequest &req, 
                          collision_detection::CollisionResult &res, 
                          const robot_state::RobotState &state,
                          const collision_detection::AllowedCollisionMatrix &acm,
                          boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  virtual void checkSelfCollision(const collision_detection::CollisionRequest &req, 
                                  collision_detection::CollisionResult &res, 
                                  const robot_state::RobotState &state1, 
                                  const robot_state::RobotState &state2) const
  {
    logWarn("Not implemented");
  };
  
  virtual void checkSelfCollision(const collision_detection::CollisionRequest &req, 
                                  collision_detection::CollisionResult &res, 
                                  const robot_state::RobotState &state1, 
                                  const robot_state::RobotState &state2, 
                                  const collision_detection::AllowedCollisionMatrix &acm) const
  {
    logWarn("Not implemented");
  };
  
  virtual void checkOtherCollision(const collision_detection::CollisionRequest &req, 
                                   collision_detection::CollisionResult &res, 
                                   const robot_state::RobotState &state,
                                   const CollisionRobot &other_robot, 
                                   const robot_state::RobotState &other_state) const
  {
    logWarn("Not implemented");
  };

  virtual void checkOtherCollision(const collision_detection::CollisionRequest &req, 
                                   collision_detection::CollisionResult &res, 
                                   const robot_state::RobotState &state,
                                   const CollisionRobot &other_robot, 
                                   const robot_state::RobotState &other_state,
                                   const collision_detection::AllowedCollisionMatrix &acm) const
  {
    logWarn("Not implemented");
  };

  virtual void checkOtherCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, 
                                   const robot_state::RobotState &state1, 
                                   const robot_state::RobotState &state2,
                                   const CollisionRobot &other_robot, 
                                   const robot_state::RobotState &other_state1, 
                                   const robot_state::RobotState &other_state2) const
  {
    logWarn("Not implemented");
  };

  virtual void checkOtherCollision(const collision_detection::CollisionRequest &req, 
                                   collision_detection::CollisionResult &res, 
                                   const robot_state::RobotState &state1, 
                                   const robot_state::RobotState &state2,
                                   const CollisionRobot &other_robot, 
                                   const robot_state::RobotState &other_state1, 
                                   const robot_state::RobotState &other_state2,
                                   const collision_detection::AllowedCollisionMatrix &acm) const
  {
    logWarn("Not implemented");
  };
  
  virtual double distanceSelf(const robot_state::RobotState &state) const
  {
    return 0.0;
  };
  virtual double distanceSelf(const robot_state::RobotState &state, const collision_detection::AllowedCollisionMatrix &acm) const
  {
    return 0.0;
  };
  virtual double distanceOther(const robot_state::RobotState &state,
                               const CollisionRobot &other_robot, const robot_state::RobotState &other_state) const
  {
    return 0.0;
  };
  virtual double distanceOther(const robot_state::RobotState &state, const CollisionRobot &other_robot,
                               const robot_state::RobotState &other_state, const collision_detection::AllowedCollisionMatrix &acm) const
  {
    return 0.0;
  };

  boost::shared_ptr<const DistanceFieldCacheEntry> getLastDistanceFieldEntry() const {
    return distance_field_cache_entry_;
  } 

  // void getSelfCollisionsGradients(const collision_detection::CollisionRequest &req, 
  //                                 collision_detection::CollisionResult &res, 
  //                                 const robot_state::RobotState &state, 
  //                                 const collision_detection::AllowedCollisionMatrix &acm) const;  
protected:

  bool getSelfProximityGradients(boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  bool getIntraGroupProximityGradients(boost::shared_ptr<GroupStateRepresentation>& gsr) const;
  
  bool getSelfCollisions(const collision_detection::CollisionRequest& req,
                         collision_detection::CollisionResult& res,
                         boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  bool getIntraGroupCollisions(const collision_detection::CollisionRequest& req,
                               collision_detection::CollisionResult& res,
                               boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  void checkSelfCollisionHelper(const collision_detection::CollisionRequest& req,
                                collision_detection::CollisionResult& res,
                                const robot_state::RobotState& state,
                                const collision_detection::AllowedCollisionMatrix *acm,
                                boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  void updateGroupStateRepresentationState(const robot_state::RobotState& state,
                                           boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  void generateCollisionCheckingStructures(const std::string& group_name,
                                           const robot_state::RobotState& state,
                                           const collision_detection::AllowedCollisionMatrix *acm,
                                           boost::shared_ptr<GroupStateRepresentation>& gsr,
                                           bool generate_distance_field) const;

  boost::shared_ptr<const DistanceFieldCacheEntry> 
  getDistanceFieldCacheEntry(const std::string& group_name,
                             const robot_state::RobotState& state,
                             const collision_detection::AllowedCollisionMatrix *acm) const;
  

  boost::shared_ptr<DistanceFieldCacheEntry> 
  generateDistanceFieldCacheEntry(const std::string& group_name,
                                  const robot_state::RobotState& state,
                                  const collision_detection::AllowedCollisionMatrix *acm,
                                  bool generate_distance_field) const;
    
  void addLinkBodyDecompositions(double resolution);

  void addLinkBodyDecompositions(double resolution,
                                 const std::map<std::string, std::vector<CollisionSphere> >& link_body_decompositions);

  PosedBodySphereDecompositionPtr 
  getPosedLinkBodySphereDecomposition(const robot_state::LinkState* ls,
                                      unsigned int ind) const;

  PosedBodyPointDecompositionPtr getPosedLinkBodyPointDecomposition(const robot_state::LinkState* ls) const;

  void getGroupStateRepresentation(const boost::shared_ptr<const DistanceFieldCacheEntry>& dfce, 
                                   const robot_state::RobotState& state,
                                   boost::shared_ptr<GroupStateRepresentation>& gsr) const;

  bool compareCacheEntryToState(const boost::shared_ptr<const DistanceFieldCacheEntry>& dfce, 
                                const robot_state::RobotState& state) const;

  bool compareCacheEntryToAllowedCollisionMatrix(const boost::shared_ptr<const DistanceFieldCacheEntry>& dfce, 
                                                 const collision_detection::AllowedCollisionMatrix& acm) const;

  virtual void updatedPaddingOrScaling(const std::vector<std::string> &links)
  {};
  
  double size_x_;
  double size_y_;
  double size_z_;
  bool use_signed_distance_field_;
  double resolution_;
  double collision_tolerance_;
  double max_propogation_distance_;

  std::vector<BodyDecompositionConstPtr> link_body_decomposition_vector_;
  std::map<std::string, unsigned int> link_body_decomposition_index_map_;

  mutable boost::mutex update_cache_lock_;
  boost::shared_ptr<DistanceFieldCacheEntry> distance_field_cache_entry_;  
  std::map<std::string, std::map<std::string, bool> > in_group_update_map_;
  std::map<std::string, boost::shared_ptr<GroupStateRepresentation> > pregenerated_group_state_representation_map_;
};

}

#endif
