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

// Author(s): E. Gil Jones

#ifndef _GRASP_EVALUATOR_FAST_
#define _GRASP_EVALUATOR_FAST_

#include <grasp_evaluator.h>
#include <kinematics_constraint_aware/kinematics_solver_constraint_aware.h>

//#include <pr2_arm_kinematics_constraint_aware/pr2_arm_ik_solver_constraint_aware.h>

namespace grasp_place_evaluation {

//! Uses an interpolated IK approach from pregrasp to final grasp
/*! Initial check consists of IK for the pre-grasp, followed by generation
  of an interpolated IK path from pre-grasp to grasp. This is then followed
  by computation of an interpolated IK path from grasp to lift.

  Execution consists of using move arm to go to the pre-grasp, then execution of 
  the interpolated IK paths for grasp. Lift is executed separately, called from
  the higher level executor.

  Note that we do not use the pre-grasp from the database directly; rather, the
  pre-grasp is obtained by backing up the grasp by a pre-defined amount. This
  allows us more flexibility in choosing the pre-grasp. However, we can no longer
  always assume the pre-grasp is collision free, which we could if we used the 
  pre-grasp from the database directly.

  In the most recent database version, the pre-grasp was obtained by backing up 
  10 cm, so we know at least that that is not colliding with the object. 
*/

inline geometry_msgs::Vector3 doNegate(const geometry_msgs::Vector3& vec) {
  geometry_msgs::Vector3 v;
  v.x = - vec.x;
  v.y = - vec.y;
  v.z = - vec.z;
  return v;
}

class GraspTesterFast : public GraspTester
{
protected:
  virtual void testGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal,
                         const object_manipulation_msgs::Grasp &grasp,
                         GraspExecutionInfo &execution_info);  
  
  //! Dynamic link padding to be used for grasp operation
  virtual std::vector<arm_navigation_msgs::LinkPadding> 
    linkPaddingForGrasp(const object_manipulation_msgs::PickupGoal &pickup_goal);

  bool getInterpolatedIK(const std::string& arm_name,
                         const tf::Transform& first_pose,
                         const tf::Vector3& direction,
                         const double& distance,
                         const std::vector<double>& ik_solution,
                         const bool& reverse, 
                         const bool& premultiply,
                         trajectory_msgs::JointTrajectory& traj);

  //arm_kinematics_constraint_aware::ArmKinematicsSolverConstraintAware* right_arm_solver_;

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

  pluginlib::ClassLoader<kinematics::KinematicsBase> kinematics_loader_;

  //! Also adds a grasp marker at the pre-grasp location
  GraspTesterFast(planning_environment::CollisionModels* cm = NULL,
		  const std::string& plugin_name="pr2_arm_kinematics/PR2ArmKinematicsPlugin");

  ~GraspTesterFast();

  void setPlanningSceneState(planning_models::KinematicState* state) {
    state_ = state;
  }

  void getGroupJoints(const std::string& group_name,
                      std::vector<std::string>& group_links);
  
  void getGroupLinks(const std::string& group_name,
                     std::vector<std::string>& group_links);

  virtual void testGrasps(const object_manipulation_msgs::PickupGoal &pickup_goal,
                          const std::vector<object_manipulation_msgs::Grasp> &grasps,
                          std::vector<GraspExecutionInfo> &execution_info,
                          bool return_on_first_hit);
};

} //namespace grasp_execution

#endif
