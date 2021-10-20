/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, MakinaRocks, Inc.
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
 *   * Neither the name of MakinaRocks nor the names of its
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

/* Author: Vinnam Kim */

#pragma once

#include <moveit/collision_detection_fcl/collision_env_fcl.h>

namespace collision_detection
{
/** \brief FCL implementation of the CollisionEnv */
class CollisionEnvMRX : public CollisionEnvFCL
{
public:
  CollisionEnvMRX() = delete;

  CollisionEnvMRX(const moveit::core::RobotModelConstPtr& model, double padding = 0.0, double scale = 1.0);

  CollisionEnvMRX(const moveit::core::RobotModelConstPtr& model, const WorldPtr& world, double padding = 0.0,
                  double scale = 1.0);

  CollisionEnvMRX(const CollisionEnvMRX& other, const WorldPtr& world);

  ~CollisionEnvMRX() override;

  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res,
                          const moveit::core::RobotState& state) const override;

  void checkSelfCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
                          const AllowedCollisionMatrix& acm) const override;

  void distanceSelf(const DistanceRequest& req, DistanceResult& res,
                    const moveit::core::RobotState& state) const override;

  //   void checkRobotCollision(const CollisionRequest& req, CollisionResult& res,
  //                            const moveit::core::RobotState& state) const override;

  //   void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state,
  //                            const AllowedCollisionMatrix& acm) const override;

  //   void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state1,
  //                            const moveit::core::RobotState& state2, const AllowedCollisionMatrix& acm) const override;

  //   void checkRobotCollision(const CollisionRequest& req, CollisionResult& res, const moveit::core::RobotState& state1,
  //                            const moveit::core::RobotState& state2) const override;

  //   void distanceRobot(const DistanceRequest& req, DistanceResult& res,
  //                      const moveit::core::RobotState& state) const override;

  void setWorld(const WorldPtr& world) override;

protected:
  /** \brief Updates the FCL collision geometry and objects saved in the CollisionRobotFCL members to reflect a new
   *   padding or scaling of the robot links.
   *
   *   It searches for the link through the pointed-to robot model of the CollisionRobot and then constructs new FCL
   *   collision objects and geometries depending on the changed robot model.
   *
   *   \param links The names of the links which have been updated in the robot model */
  void updatedPaddingOrScaling(const std::vector<std::string>& links) override;

  /** \brief Bundles the different checkSelfCollision functions into a single function */
  void constructFCLObjectRobot(const moveit::core::RobotState& state, FCLObject& fcl_obj,
                               const std::vector<bool>& unpadded_geometry_flags) const;

  void allocSelfCollisionBroadPhase(const moveit::core::RobotState& state, FCLManager& manager,
                                    const std::vector<bool>& unpadded_geometry_flags) const;

  void checkSelfCollisionHelper(const CollisionRequest& req, CollisionResult& res,
                                const moveit::core::RobotState& state, const AllowedCollisionMatrix* acm) const;

private:
  void initPaddedRobotObjects(double padding, double scale);

  template <class Request>
  std::vector<bool> getUnpaddedGeometryFlags(const Request& req) const
  {
    return getUnpaddedGeometryFlags(req.group_name);
  }

  std::vector<bool> getUnpaddedGeometryFlags(const std::string& group_name) const;

  /** \brief Vector of shared pointers to the FCL geometry for the objects in fcl_objs_. */
  std::vector<FCLGeometryConstPtr> padded_robot_geoms_;

  /** \brief Vector of shared pointers to the FCL collision objects which make up the robot */
  std::vector<FCLCollisionObjectConstPtr> padded_robot_fcl_objs_;
};
}  // namespace collision_detection
