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

/* Author: Ioan Sucan, Sachin Chitta */

#ifndef MOVEIT_PICK_PLACE_MANIPULATION_PLAN_
#define MOVEIT_PICK_PLACE_MANIPULATION_PLAN_

#include <boost/shared_ptr.hpp>
#include <moveit/robot_state/robot_state.h>
#include <moveit/constraint_samplers/constraint_sampler.h>
#include <manipulation_msgs/Grasp.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/Constraints.h>
#include <string>
#include <vector>

namespace pick_place
{


struct ManipulationPlan
{
  // The grasp that is attempted
  manipulation_msgs::Grasp grasp_;
  
  std::string planning_group_;
  
  std::string end_effector_group_;
  
  std::string ik_link_name_;
  
  ros::WallTime timeout_;
  
  moveit_msgs::Constraints goal_constraints_;
  constraint_samplers::ConstraintSamplerPtr goal_sampler_;
  unsigned int sampling_attempts_;
  std::vector<robot_state::RobotStatePtr> possible_goal_states_;

  robot_state::RobotStatePtr approach_state_;
  
  // The sequence of trajectories produced for execution
  std::vector<robot_trajectory::RobotTrajectoryPtr> trajectories_;
  
  // String descriptors of the trajectories
  std::vector<std::string> trajectory_descriptions_;
  
  // An error code reflecting what went wrong (if anything)
  moveit_msgs::MoveItErrorCodes error_code_;
  
  // The processing stage that was last working on this plan, or was about to work on this plan
  std::size_t processing_stage_;
};

typedef boost::shared_ptr<ManipulationPlan> ManipulationPlanPtr;
typedef boost::shared_ptr<const ManipulationPlan> ManipulationPlanConstPtr;

}

#endif
