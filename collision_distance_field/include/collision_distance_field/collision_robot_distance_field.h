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

#ifndef COLLISION_ROBOT_DISTANCE_FIELD_H_
#define COLLISION_ROBOT_DISTANCE_FIELD_H_

#include <collision_detection/collision_robot.h>
#include <collision_distance_field/collision_distance_field_types.h>

#include <boost/thread/mutex.hpp>

namespace collision_distance_field 
{

class CollisionRobotDistanceField : public collision_detection::CollisionRobot
{
public:
  
  CollisionRobotDistanceField(const planning_models::KinematicModelConstPtr& kmodel, 
                              double size_x = 3.0, 
                              double size_y = 3.0,
                              double size_z = 4.0,
                              bool use_signed_distance_field = true,
                              double resolution = .02,
                              double collision_tolerance = 0.0,
                              double max_propogation_distance = .25,
                              double padding = 0.0, 
                              double scale = 1.0);

  //CollisionRobotDistanceField(const CollisionRobotDistanceField& other);

  virtual void checkSelfCollision(const collision_detection::CollisionRequest &req, 
                                  collision_detection::CollisionResult &res, 
                                  const planning_models::KinematicState &state) const;

  virtual void checkSelfCollision(const collision_detection::CollisionRequest &req, 
                                  collision_detection::CollisionResult &res, 
                                  const planning_models::KinematicState &state, 
                                  const collision_detection::AllowedCollisionMatrix &acm) const
  {};

  virtual void checkSelfCollision(const collision_detection::CollisionRequest &req, 
                                  collision_detection::CollisionResult &res, 
                                  const planning_models::KinematicState &state1, 
                                  const planning_models::KinematicState &state2) const
  {};
  
  virtual void checkSelfCollision(const collision_detection::CollisionRequest &req, 
                                  collision_detection::CollisionResult &res, 
                                  const planning_models::KinematicState &state1, 
                                  const planning_models::KinematicState &state2, 
                                  const collision_detection::AllowedCollisionMatrix &acm) const
  {};
  
  virtual void checkOtherCollision(const collision_detection::CollisionRequest &req, 
                                   collision_detection::CollisionResult &res, 
                                   const planning_models::KinematicState &state,
                                   const CollisionRobot &other_robot, 
                                   const planning_models::KinematicState &other_state) const
  {};

  virtual void checkOtherCollision(const collision_detection::CollisionRequest &req, 
                                   collision_detection::CollisionResult &res, 
                                   const planning_models::KinematicState &state,
                                   const CollisionRobot &other_robot, 
                                   const planning_models::KinematicState &other_state,
                                   const collision_detection::AllowedCollisionMatrix &acm) const
  {};

  virtual void checkOtherCollision(const collision_detection::CollisionRequest &req, collision_detection::CollisionResult &res, 
                                   const planning_models::KinematicState &state1, 
                                   const planning_models::KinematicState &state2,
                                   const CollisionRobot &other_robot, 
                                   const planning_models::KinematicState &other_state1, 
                                   const planning_models::KinematicState &other_state2) const
  {};

  virtual void checkOtherCollision(const collision_detection::CollisionRequest &req, 
                                   collision_detection::CollisionResult &res, 
                                   const planning_models::KinematicState &state1, 
                                   const planning_models::KinematicState &state2,
                                   const CollisionRobot &other_robot, 
                                   const planning_models::KinematicState &other_state1, 
                                   const planning_models::KinematicState &other_state2,
                                   const collision_detection::AllowedCollisionMatrix &acm) const
  {};
  
  virtual double distanceSelf(const planning_models::KinematicState &state) const
  {
    return 0.0;
  };
  virtual double distanceSelf(const planning_models::KinematicState &state, const collision_detection::AllowedCollisionMatrix &acm) const
  {
    return 0.0;
  };
  virtual double distanceOther(const planning_models::KinematicState &state,
                               const CollisionRobot &other_robot, const planning_models::KinematicState &other_state) const
  {
    return 0.0;
  };
  virtual double distanceOther(const planning_models::KinematicState &state, const CollisionRobot &other_robot,
                               const planning_models::KinematicState &other_state, const collision_detection::AllowedCollisionMatrix &acm) const
  {
    return 0.0;
  };
  
protected:

  struct DistanceFieldCacheEntry {
    std::string group_name_;
    boost::shared_ptr<planning_models::KinematicState> state_;
    collision_detection::AllowedCollisionMatrix acm_;
    distance_field::DistanceField<distance_field::PropDistanceFieldVoxel>* distance_field_;
    std::vector<std::string> link_names_;
    std::vector<unsigned int> link_state_indices_;
    std::vector<std::string> attached_body_names_;
    std::vector<unsigned int> attached_body_link_state_indices_;
    std::vector<PosedBodyDecompositionPtr> link_body_decompositions_;
    std::vector<PosedBodyDecompositionVectorPtr> attached_body_decompositions_;
  };

  void checkSelfCollisionHelper(const collision_detection::CollisionRequest& req,
                                collision_detection::CollisionResult& res,
                                const planning_models::KinematicState& state,
                                const collision_detection::AllowedCollisionMatrix *acm) const;

  boost::shared_ptr<const DistanceFieldCacheEntry> 
  getDistanceFieldCacheEntry(const std::string& group_name,
                             const planning_models::KinematicState& state,
                             const collision_detection::AllowedCollisionMatrix *acm) const;
  

  boost::shared_ptr<DistanceFieldCacheEntry> 
  generateDistanceFieldCacheEntry(const std::string& group_name,
                                  const planning_models::KinematicState& state,
                                  const collision_detection::AllowedCollisionMatrix *acm) const;
    

  virtual void updatedPaddingOrScaling(const std::vector<std::string> &links)
  {};
  
  double size_x_;
  double size_y_;
  double size_z_;
  bool use_signed_distance_field_;
  double resolution_;
  double collision_tolerance_;
  double max_propogation_distance_;

  mutable boost::mutex update_cache_lock_;
  boost::shared_ptr<DistanceFieldCacheEntry> distance_field_cache_entry_;
};

}

#endif
