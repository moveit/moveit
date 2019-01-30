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

#ifndef MOVEIT_COLLISION_DISTANCE_FIELD_COLLISION_ROBOT_HYBRID_H_
#define MOVEIT_COLLISION_DISTANCE_FIELD_COLLISION_ROBOT_HYBRID_H_

#include <moveit/collision_detection_fcl/collision_robot_fcl.h>
#include <moveit/collision_distance_field/collision_distance_field_types.h>
#include <moveit/collision_distance_field/collision_common_distance_field.h>
#include <moveit/collision_distance_field/collision_robot_distance_field.h>

#include <boost/thread/mutex.hpp>

namespace collision_detection
{
class CollisionRobotHybrid : public collision_detection::CollisionRobotFCL
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CollisionRobotHybrid(const robot_model::RobotModelConstPtr& kmodel);

  CollisionRobotHybrid(const robot_model::RobotModelConstPtr& kmodel,
                       const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions,
                       double size_x = 3.0, double size_y = 3.0, double size_z = 4.0,
                       bool use_signed_distance_field = false, double resolution = .02,
                       double collision_tolerance = 0.0, double max_propogation_distance = .25, double padding = 0.0,
                       double scale = 1.0);

  CollisionRobotHybrid(const CollisionRobotHybrid& other);

  void initializeRobotDistanceField(const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions,
                                    double size_x, double size_y, double size_z, bool use_signed_distance_field,
                                    double resolution, double collision_tolerance, double max_propogation_distance)
  {
    crobot_distance_->initialize(link_body_decompositions, Eigen::Vector3d(size_x, size_y, size_z),
                                 Eigen::Vector3d(0, 0, 0), use_signed_distance_field, resolution, collision_tolerance,
                                 max_propogation_distance);
  }

  void checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                       collision_detection::CollisionResult& res,
                                       const robot_state::RobotState& state) const;

  void checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                       collision_detection::CollisionResult& res, const robot_state::RobotState& state,
                                       GroupStateRepresentationPtr& gsr) const;

  void checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                       collision_detection::CollisionResult& res, const robot_state::RobotState& state,
                                       const collision_detection::AllowedCollisionMatrix& acm) const;

  void checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                       collision_detection::CollisionResult& res, const robot_state::RobotState& state,
                                       const collision_detection::AllowedCollisionMatrix& acm,
                                       GroupStateRepresentationPtr& gsr) const;
  const CollisionRobotDistanceFieldConstPtr getCollisionRobotDistanceField() const
  {
    return crobot_distance_;
  }

protected:
  CollisionRobotDistanceFieldPtr crobot_distance_;
};
}

#endif
