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

#include <moveit/collision_distance_field/collision_robot_hybrid.h>

namespace collision_detection
{
CollisionRobotHybrid::CollisionRobotHybrid(const robot_model::RobotModelConstPtr& kmodel) : CollisionRobotFCL(kmodel)
{
  crobot_distance_.reset(new collision_detection::CollisionRobotDistanceField(kmodel));
}

CollisionRobotHybrid::CollisionRobotHybrid(
    const robot_model::RobotModelConstPtr& kmodel,
    const std::map<std::string, std::vector<CollisionSphere>>& link_body_decompositions, double size_x, double size_y,
    double size_z, bool use_signed_distance_field, double resolution, double collision_tolerance,
    double max_propogation_distance, double padding, double scale)
  : CollisionRobotFCL(kmodel)
{
  crobot_distance_.reset(new collision_detection::CollisionRobotDistanceField(
      kmodel, link_body_decompositions, size_x, size_y, size_z, use_signed_distance_field, resolution,
      collision_tolerance, max_propogation_distance, padding, scale));
}

CollisionRobotHybrid::CollisionRobotHybrid(const CollisionRobotHybrid& other) : CollisionRobotFCL(other)
{
  crobot_distance_.reset(
      new collision_detection::CollisionRobotDistanceField(*other.getCollisionRobotDistanceField().get()));
}

void CollisionRobotHybrid::checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                                           collision_detection::CollisionResult& res,
                                                           const robot_state::RobotState& state) const
{
  crobot_distance_->checkSelfCollision(req, res, state);
}

void CollisionRobotHybrid::checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                                           collision_detection::CollisionResult& res,
                                                           const robot_state::RobotState& state,
                                                           GroupStateRepresentationPtr& gsr) const
{
  crobot_distance_->checkSelfCollision(req, res, state, gsr);
}

void CollisionRobotHybrid::checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                                           collision_detection::CollisionResult& res,
                                                           const robot_state::RobotState& state,
                                                           const collision_detection::AllowedCollisionMatrix& acm) const
{
  crobot_distance_->checkSelfCollision(req, res, state, acm);
}

void CollisionRobotHybrid::checkSelfCollisionDistanceField(const collision_detection::CollisionRequest& req,
                                                           collision_detection::CollisionResult& res,
                                                           const robot_state::RobotState& state,
                                                           const collision_detection::AllowedCollisionMatrix& acm,
                                                           GroupStateRepresentationPtr& gsr) const
{
  crobot_distance_->checkSelfCollision(req, res, state, acm, gsr);
}
}
