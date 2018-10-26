/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, OMRON SINIC X Corporation
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
 *   * Neither the name of OMRON SINIC X Corp. nor the names of its
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

/* This service is used for constraints that are applied to frames on
 * objects attached to the robot (e.g. "tip of screw driver").
 * It makes these constraints valid by transforming them from the frame
 * on the object to a frame on the robot. The planning request is not
 * valid unless the constraints are applied to a robot link.
 *
 * This service might become a PlanningRequestAdapter in the near future.
 *
 * Author: Felix von Drigalski
 */

#ifndef MOVEIT_MOVE_GROUP_CONSTRAINT_VALIDATION_SERVICE_CAPABILITY_
#define MOVEIT_MOVE_GROUP_CONSTRAINT_VALIDATION_SERVICE_CAPABILITY_

#include <moveit/move_group/move_group_capability.h>
#include <moveit_msgs/GetConstraintValidity.h>

namespace move_group
{
class MoveGroupConstraintValidationService : public MoveGroupCapability
{
public:
  MoveGroupConstraintValidationService();

  virtual void initialize();

private:
  bool computeService(moveit_msgs::GetConstraintValidity::Request& req,
                      moveit_msgs::GetConstraintValidity::Response& res);

  ros::ServiceServer constraint_validity_service_;
};
}

#endif
