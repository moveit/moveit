/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#ifndef KINEMATIC_CONSTRAINTS_UTILS_
#define KINEMATIC_CONSTRAINTS_UTILS_

#include <moveit_msgs/MotionPlanRequest.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <planning_models/kinematic_state.h>
#include <limits>

namespace kinematic_constraints
{

    /** \brief Merge two sets of constraints into one. This just does appending of all constraints except joint constraints. For joint constraints,
        the bounds specified in \e first take precedence over \e second */
    moveit_msgs::Constraints mergeConstraints(const moveit_msgs::Constraints &first, const moveit_msgs::Constraints &second);

    moveit_msgs::Constraints constructGoalConstraints(const planning_models::KinematicState::JointStateGroup *jsg,
                                                      double tolerance_below, double tolerance_above);

    moveit_msgs::Constraints constructGoalConstraints(const planning_models::KinematicState::JointStateGroup *jsg,
                                                      double tolerance = std::numeric_limits<double>::epsilon());

    moveit_msgs::Constraints constructGoalConstraints(const std::string &link_name, const geometry_msgs::PoseStamped &pose, double tolerance_pos = 1e-3, double tolerance_angle = 1e-2);
    moveit_msgs::Constraints constructGoalConstraints(const std::string &link_name, const geometry_msgs::QuaternionStamped &quat, double tolerance = 1e-2);
    moveit_msgs::Constraints constructGoalConstraints(const std::string &link_name, const geometry_msgs::PointStamped &point, double tolerance = 1e-3);
}

#endif
