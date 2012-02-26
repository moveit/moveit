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

// Author(s): Matei Ciocarlie, E. Gil Jones

#ifndef _PLACE_TESTER_FAST_
#define _PLACE_TESTER_FAST_

#include "object_manipulator/place_execution/descend_retreat_place.h"
#include <arm_kinematics_constraint_aware/arm_kinematics_solver_constraint_aware.h>
//#include <pr2_arm_kinematics_constraint_aware/pr2_arm_ik_solver_constraint_aware.h>

#include <pluginlib/class_loader.h>

namespace object_manipulator {

class PlaceTesterFast : public PlaceTester
{
protected:

  geometry_msgs::Vector3 doNegate(const geometry_msgs::Vector3& vec) {
    geometry_msgs::Vector3 v;
    v.x = - vec.x;
    v.y = - vec.y;
    v.z = - vec.z;
    return v;
  }

  virtual void testPlace(const object_manipulation_msgs::PlaceGoal &placre_goal,
                         const geometry_msgs::PoseStamped &place_locations,
                         PlaceExecutionInfo &execution_info);
  
  //! Dynamic link padding to be used for grasp operation
  std::vector<arm_navigation_msgs::LinkPadding> 
  linkPaddingForPlace(const object_manipulation_msgs::PlaceGoal &place_goal);
  
  void getGroupLinks(const std::string& group_name,
                     std::vector<std::string>& group_links);

  bool getInterpolatedIK(const std::string& arm_name,
                         const tf::Transform& first_pose,
                         const tf::Vector3& direction,
                         const double& distance,
                         const std::vector<double>& ik_solution,
                         const bool& reverse, 
                         const bool& premultiply,
                         trajectory_msgs::JointTrajectory& traj);
    
  std::map<std::string, arm_kinematics_constraint_aware::ArmKinematicsSolverConstraintAware*> ik_solver_map_;
  //std::map<std::string, pr2_arm_kinematics::PR2ArmIKSolverConstraintAware*> ik_solver_map_;
  
  double consistent_angle_;
  unsigned int num_points_;
  unsigned int redundancy_;
  
  ros::Publisher vis_marker_array_publisher_;
  ros::Publisher vis_marker_publisher_;

  planning_environment::CollisionModels* getCollisionModels();
  planning_models::KinematicState* getPlanningSceneState();

  planning_environment::CollisionModels* cm_;
  planning_models::KinematicState* state_;

 public:
  //! Also adds a grasp marker at the pre-grasp location
  PlaceTesterFast(planning_environment::CollisionModels* cm = NULL,
		  const std::string& plugin_name="pr2_arm_kinematics/PR2ArmKinematicsPlugin");

  ~PlaceTesterFast();

  void setPlanningSceneState(planning_models::KinematicState* state) {
    state_ = state;
  }

  void testPlaces(const object_manipulation_msgs::PlaceGoal &place_goal,
                  const std::vector<geometry_msgs::PoseStamped> &place_locations,
                  std::vector<PlaceExecutionInfo> &execution_info,
                  bool return_on_first_hit);

  pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader_;

};

} //namespace grasp_execution

#endif
