/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
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

#ifndef _PLACE_EVALUATOR_H_
#define _PLACE_EVALUATOR_H_

#include <ros/ros.h>

#include <moveit_manipulation_msgs/PlaceGoal.h>
#include <moveit_manipulation_msgs/PlaceLocationResult.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <planning_scene/planning_scene.h>
#include <planning_models/kinematic_state.h>

namespace grasp_place_evaluation {

//! Functionality for placing a grasped object at a given location
/*! As a mirror image of the classes that offer grasping functionality, placing implies:
  - a pre-place position (some cm above the final placing pose)
  - an interpolated trajectory from pre-place to place
  - a gripper retreat after placing, done by interpolating "back" along gripper approach direction

  This class will first compute that both place and retreat interpolated trajectories
  are possible, then proceed to execute the place trajectory.
*/

struct PlaceExecutionInfo {

  PlaceExecutionInfo() {
    place_pose_.setIdentity();
    preplace_pose_.setIdentity();
    retreat_pose_.setIdentity();
  }

  trajectory_msgs::JointTrajectory approach_trajectory_; 
  trajectory_msgs::JointTrajectory retreat_trajectory_; 
  Eigen::Affine3d place_pose_;
  Eigen::Affine3d preplace_pose_;
  Eigen::Affine3d retreat_pose_;
  planning_scene::PlanningScenePtr detached_object_diff_scene_;
  moveit_manipulation_msgs::PlaceLocationResult result_;
};

inline static std::string convertPlaceResultToStringStatus(const moveit_manipulation_msgs::PlaceLocationResult& pr) {
  if(pr.result_code == moveit_manipulation_msgs::PlaceLocationResult::SUCCESS) {
    return "Success";
  } else if(pr.result_code == moveit_manipulation_msgs::PlaceLocationResult::PLACE_OUT_OF_REACH) {
    return "Place out of reach";
  } else if(pr.result_code == moveit_manipulation_msgs::PlaceLocationResult::PLACE_IN_COLLISION) {
    return "Place in collision";
  } else if(pr.result_code == moveit_manipulation_msgs::PlaceLocationResult::PLACE_UNFEASIBLE) {
    return "Place unfeasible";
  } else if(pr.result_code == moveit_manipulation_msgs::PlaceLocationResult::PREPLACE_OUT_OF_REACH) {
    return "Preplace out of reach";
  } else if(pr.result_code == moveit_manipulation_msgs::PlaceLocationResult::PREPLACE_IN_COLLISION) {
    return "Preplace in collision";
  } else if(pr.result_code == moveit_manipulation_msgs::PlaceLocationResult::PREPLACE_UNFEASIBLE) {
    return "Preplace unfeasible";
  } else if(pr.result_code == moveit_manipulation_msgs::PlaceLocationResult::RETREAT_OUT_OF_REACH) {
    return "Retreat out of reach";
  } else if(pr.result_code == moveit_manipulation_msgs::PlaceLocationResult::RETREAT_IN_COLLISION) {
    return "Retreat in collision";
  } else if(pr.result_code == moveit_manipulation_msgs::PlaceLocationResult::RETREAT_UNFEASIBLE) {
    return "Retreat unfeasible";
  } 
  return "Unknown";
}

struct PlaceExecutionInfoVector : public std::vector<PlaceExecutionInfo> {
  moveit_manipulation_msgs::PlaceGoal place_goal_;
  std::vector<geometry_msgs::PoseStamped> place_locations_;
};


class PlaceEvaluator
{
public:
  PlaceEvaluator() {}
  
  //! Places a grasped object at a specified location
  virtual void testPlaceLocations(const planning_scene::PlanningSceneConstPtr& planning_scene,
                                  const planning_models::KinematicState* seed_state,
                                  const moveit_manipulation_msgs::PlaceGoal &place_goal, 
                                  const geometry_msgs::Vector3& retreat_direction,
                                  const std::vector<geometry_msgs::PoseStamped>& place_locations,
                                  PlaceExecutionInfoVector &execution_info_vector,
                                  bool return_on_first_hit) = 0;

};

} //namespace grasp_place_evaluation

#endif
